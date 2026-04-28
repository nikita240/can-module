#![no_std]
#![no_main]

use core::cell::RefCell;

use ads1x1x::{Ads1x1x, ComparatorQueue, FullScaleRange};
use defmt::info;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::pwm::{self, Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, InterruptHandler as I2cInterruptHandler},
    peripherals::{I2C1, SPI0},
    spi::{self, Spi},
};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex};
use embassy_time::{Delay, Duration, Timer};
use fixed::traits::ToFixed;
use mcp2515::{CanSpeed, MCP2515, McpSpeed, Settings, regs::OpMode};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------
type Spi0Bus = Mutex<NoopRawMutex, RefCell<Spi<'static, SPI0, spi::Blocking>>>;
type Adc = Ads1x1x<i2c::I2c<'static, I2C1, i2c::Async>, ads1x1x::ic::Ads1115, ads1x1x::ic::Resolution16Bit, ads1x1x::mode::OneShot>;

// ---------------------------------------------------------------------------
// Interrupt bindings
// ---------------------------------------------------------------------------
bind_interrupts!(struct Irqs {
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});

// ---------------------------------------------------------------------------
// Totem-pole PWM helpers
// ---------------------------------------------------------------------------

/// Compute (divider, top) for a target PWM frequency from clk_sys.
fn pwm_freq_params(freq_hz: u32) -> (u16, u16) {
    let clock_freq = embassy_rp::clocks::clk_sys_freq();
    let divider: u16 = (clock_freq / (freq_hz * 65535) + 1) as u16;
    let top = (clock_freq / (freq_hz * divider as u32)) as u16 - 1;
    (divider, top)
}

/// Convert duty in parts-per-thousand (0..=1000) to the slice's compare value.
fn duty_ppt_to_compare(duty_ppt: u16, top: u16) -> u16 {
    let compare = (duty_ppt as u32 * (top as u32 + 1)) / 1000;
    compare.min(top as u32 + 1) as u16
}

/// Build a complementary (VREF / GND) PWM config for totem-pole gauge outputs.
/// Use for analog *voltage* signals (3-wire ratiometric sensors, etc).
///   - Channel A = PWM_HI (drives PMOS high-side via level-shifter)
///   - Channel B = PWM_LO (drives NMOS low-side, inverted for anti-phase)
///   - duty    0 ‰ → output stuck at GND
///   - duty 1000 ‰ → output stuck at VREF
fn totem_pole_config(freq_hz: u32, duty_ppt: u16) -> PwmConfig {
    let mut cfg = PwmConfig::default();
    let (divider, top) = pwm_freq_params(freq_hz);

    cfg.divider = divider.to_fixed();
    cfg.top = top;
    cfg.invert_a = false;
    cfg.invert_b = true;

    let compare = duty_ppt_to_compare(duty_ppt, top);
    cfg.compare_a = compare;
    cfg.compare_b = compare;
    cfg
}

/// Build a NMOS-only (GND / Open) PWM config for resistance-to-ground gauges
/// (fuel sender, oil pressure sender, coolant temp sender, etc).
///   - Channel A = PWM_HI: pinned LOW so the PMOS stays OFF (no VREF on output)
///   - Channel B = PWM_LO: drives NMOS directly
///   - duty    0 ‰ → NMOS always off → pin floating (R_eff = ∞)
///   - duty 1000 ‰ → NMOS always on  → pin grounded (R_eff = 0)
fn resistance_pwm_config(freq_hz: u32, duty_ppt: u16) -> PwmConfig {
    let mut cfg = PwmConfig::default();
    let (divider, top) = pwm_freq_params(freq_hz);

    cfg.divider = divider.to_fixed();
    cfg.top = top;
    cfg.invert_a = false;
    cfg.invert_b = false;

    cfg.compare_a = 0; // PWM_HI always LOW → level shifter off → PMOS off
    cfg.compare_b = duty_ppt_to_compare(duty_ppt, top);
    cfg
}


/// Quadratic fit of empirical gauge calibration data:
///   duty_ppt = (3·pct² + 200·pct + 47500) / 100
fn gauge_pct_to_duty_ppt(pct: u32) -> u16 {
    // Duty -> Gauge table
    // 475 -> 0%
    // 540 -> 25%
    // 650 -> 50%
    // 830 -> 75%
    // 975 -> 100%

    let pct = pct.min(100);
    let duty = (3 * pct * pct + 200 * pct + 47_500) / 100;
    duty.min(1000) as u16
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("CAN module starting");

    // -----------------------------------------------------------------------
    // Status LED — GPIO0
    // -----------------------------------------------------------------------
    // let _status_led = Output::new(p.PIN_0, Level::High);
    let status_led = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_0, pwm::Config::default());

    // -----------------------------------------------------------------------
    // I2C1 — ADS1115 16-bit ADC
    //   GPIO2 = SDA, GPIO3 = SCL, GPIO5 = ALERT/RDY
    // -----------------------------------------------------------------------
    let i2c_config = i2c::Config::default();
    let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, i2c_config);

    let adc_drdy = Input::new(p.PIN_5, Pull::Up);

    // ADS1115: ADDR pin to GND → default address 0x48
    let mut adc = Ads1x1x::new_ads1115(i2c, ads1x1x::TargetAddr::default());
    // R1=440Ω, R2=0..68Ω → V_max = 5 × 68/508 = 0.669V → use ±1.024V PGA range
    let _ = adc.set_full_scale_range(FullScaleRange::Within1_024V);
    // Enable ALERT/RDY as conversion-ready pin (work around crate bug
    // where use_alert_rdy_pin_as_ready() skips enabling the comparator).
    let _ = adc.set_comparator_queue(ComparatorQueue::One);
    let _ = adc.set_low_threshold_raw(0);
    let _ = adc.set_high_threshold_raw(-32768); // 0x8000

    // -----------------------------------------------------------------------
    // SPI0 — shared bus for two MCP25625 CAN controllers
    //   GPIO18 = SCK, GPIO19 = MOSI, GPIO20 = MISO
    // -----------------------------------------------------------------------
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 2_000_000;
    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, spi_config);
    static SPI_BUS: StaticCell<Spi0Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi.into()));

    // -----------------------------------------------------------------------
    // CAN 0 — MCP25625 on SPI0
    //   GPIO14=CS, GPIO15=INT, GPIO16=STBY, GPIO17=RESET
    // -----------------------------------------------------------------------
    let can0_cs = Output::new(p.PIN_14, Level::High);
    let _can0_int = Input::new(p.PIN_15, Pull::Up);
    let _can0_stby = Output::new(p.PIN_16, Level::Low);
    let mut can0_reset = Output::new(p.PIN_17, Level::Low);
    can0_reset.set_high();

    let can0_spi = SpiDevice::new(spi_bus, can0_cs);
    let mut can0 = MCP2515::new(can0_spi);
    let mut delay = Delay;
    match can0.init(
        &mut delay,
        Settings {
            mode: OpMode::Normal,           // normal operation after init
            can_speed: CanSpeed::Kbps500,   // 500 kbps — standard automotive
            mcp_speed: McpSpeed::MHz16,     // 16 MHz crystal on MCP25625
            clkout_en: true, // can0 CLKOUT feeds OSC1 on can1 so that they share crystal
        },
    ) {
        Ok(_) => info!("CAN0 initialized"),
        Err(_) => info!("CAN0 init failed (expected before board is built)"),
    }

    // -----------------------------------------------------------------------
    // CAN 1 — MCP25625 on SPI0
    //   GPIO21=CS, GPIO22=INT, GPIO23=STBY, GPIO24=RESET
    // -----------------------------------------------------------------------
    let can1_cs = Output::new(p.PIN_21, Level::High);
    let _can1_int = Input::new(p.PIN_22, Pull::Up);
    let _can1_stby = Output::new(p.PIN_23, Level::Low);
    let mut can1_reset = Output::new(p.PIN_24, Level::Low);
    can1_reset.set_high();

    let can1_spi = SpiDevice::new(spi_bus, can1_cs);
    let mut can1 = MCP2515::new(can1_spi);
    match can1.init(
        &mut delay,
        Settings {
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps500,
            mcp_speed: McpSpeed::MHz16,
            clkout_en: false,
        },
    ) {
        Ok(_) => info!("CAN1 initialized"),
        Err(_) => info!("CAN1 init failed (expected before board is built)"),
    }

    // -----------------------------------------------------------------------
    // Totem-pole PWM outputs — 4 channels
    //
    // Slice 3: GPIO6 /GPIO7   → Totem 0  (resistance emulation, NMOS-only)
    // Slice 4: GPIO8 /GPIO9   → Totem 1  (complementary VREF/GND)
    // Slice 5: GPIO10/GPIO11  → Totem 2  (complementary VREF/GND)
    // Slice 6: GPIO12/GPIO13  → Totem 3  (complementary VREF/GND)
    // -----------------------------------------------------------------------
    const PWM_FREQ_HZ: u32 = 1000;

    // Totem 0 — resistance-to-ground gauge emulator.
    let totem0_cfg = resistance_pwm_config(PWM_FREQ_HZ, 0);
    let totem0 = Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, totem0_cfg);

    // Totems 1–3 — complementary VREF/GND drive for voltage-style gauges.
    let voltage_cfg = totem_pole_config(PWM_FREQ_HZ, 0);
    let _totem1 = Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, voltage_cfg.clone());
    let _totem2 = Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, voltage_cfg.clone());
    let _totem3 = Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_12, p.PIN_13, voltage_cfg);

    info!("All peripherals initialized — starting demo sweep");

    spawner.spawn(animate_status_led(status_led)).unwrap();
    spawner.spawn(animate_fuel_gauge(totem0)).unwrap();
    spawner.spawn(read_adc(adc, adc_drdy)).unwrap();
}

#[embassy_executor::task]
pub async fn animate_fuel_gauge(mut totem0: Pwm<'static>) {
    const PWM_FREQ_HZ: u32 = 1000;
    const SWEEP_STEPS: u32 = 100;
    const SWEEP_DURATION_MS: u64 = 4_000;
    const SWEEP_STEP_MS: u64 = SWEEP_DURATION_MS / SWEEP_STEPS as u64;
    const HOLD_MS: u64 = 1_000;
    const RAMP_STEPS: u32 = 25;
    const RAMP_DURATION_MS: u64 = 500;
    const RAMP_STEP_MS: u64 = RAMP_DURATION_MS / RAMP_STEPS as u64;

    loop {
        // Phase 1: continuous sweep 100 → 0
        for step in 0..=SWEEP_STEPS {
            let gauge_percent = 100 - (100 * step) / SWEEP_STEPS;
            let duty_ppt = gauge_pct_to_duty_ppt(gauge_percent);
            totem0.set_config(&resistance_pwm_config(PWM_FREQ_HZ, duty_ppt));
            Timer::after(Duration::from_millis(SWEEP_STEP_MS)).await;
        }

        // Phase 2: hold at 0 % for 1 s, then ramp-and-hold through 25 → 50 → 75 → 100
        // (smooth 0.5 s interpolation between each level, then 1 s hold)
        Timer::after(Duration::from_millis(HOLD_MS)).await;
        let mut prev_pct: u32 = 0;
        for next_pct in [25u32, 50, 75, 100] {
            for step in 1..=RAMP_STEPS {
                let pct = prev_pct + ((next_pct - prev_pct) * step) / RAMP_STEPS;
                let duty_ppt = gauge_pct_to_duty_ppt(pct);
                totem0.set_config(&resistance_pwm_config(PWM_FREQ_HZ, duty_ppt));
                Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
            }
            Timer::after(Duration::from_millis(HOLD_MS)).await;
            prev_pct = next_pct;
        }
    }
}

#[embassy_executor::task]
pub async fn read_adc(mut adc: Adc, mut drdy: Input<'static>) {
    // Voltage divider: V_adc = Vref × R2 / (R1 + R2)
    // Solving for R2:  R2 = V_adc × R1 / (Vref - V_adc)
    const R1_MOHMS: i64 = 470_000;
    const VREF_UV: i64 = 5_000_000; // 5V in µV

    loop {
        // Trigger conversion (first call returns WouldBlock)
        let _ = adc.read(ads1x1x::channel::SingleA0);
        // Wait for ALERT/RDY pin to signal conversion complete
        drdy.wait_for_falling_edge().await;
        // Read the result (now ready)
        let raw = match adc.read(ads1x1x::channel::SingleA0) {
            Ok(val) => val,
            Err(_) => {
                info!("ADC read failed");
                Timer::after(Duration::from_millis(500)).await;
                continue;
            }
        };
        // ±1.024V FSR → 1 LSB = 1024000 µV / 32768 = 31.25 µV
        let v_uv = raw as i64 * 125 / 4;
        let r2_mohms = if v_uv > 0 && v_uv < VREF_UV {
            v_uv * R1_MOHMS / (VREF_UV - v_uv)
        } else {
            0
        };
        info!("ADC: raw={}, V={}uV, R2={}mOhm", raw, v_uv, r2_mohms);
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
pub async fn animate_status_led(mut status_led: Pwm<'static>) {
    const MILLIS_PER_UPDATE: i32 = 25;
    const PULSE_DURATION_MS: i32 = 2000;
    let max_duty = status_led.max_duty_cycle() / 100;
    let mut duty: i32 = max_duty as i32;
    let mut dt = MILLIS_PER_UPDATE * 2 * (max_duty as i32) / PULSE_DURATION_MS;

    loop {
        status_led.set_duty_cycle(duty as u16).unwrap();
        embassy_time::Timer::after(Duration::from_millis(MILLIS_PER_UPDATE as u64)).await;

        if duty <= 0 || duty >= max_duty as i32 {
            dt = -dt;
        }
        duty += dt;
        duty = duty.clamp(0, max_duty as i32);
    }
}