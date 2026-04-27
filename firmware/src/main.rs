#![no_std]
#![no_main]

use core::cell::RefCell;

use ads1x1x::Ads1x1x;
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
use embassy_time::{Delay, Duration};
use fixed::traits::ToFixed;
use mcp2515::{CanSpeed, MCP2515, McpSpeed, Settings, regs::OpMode};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------
type Spi0Bus = Mutex<NoopRawMutex, RefCell<Spi<'static, SPI0, spi::Blocking>>>;

// ---------------------------------------------------------------------------
// Interrupt bindings
// ---------------------------------------------------------------------------
bind_interrupts!(struct Irqs {
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});

// ---------------------------------------------------------------------------
// Totem-pole PWM helpers
// ---------------------------------------------------------------------------

/// Build a complementary PWM config for totem-pole gauge outputs.
///   - Channel A = PWM_HI (drives PMOS high-side via level-shifter)
///   - Channel B = PWM_LO (drives NMOS low-side, inverted for anti-phase)
fn totem_pole_config(freq_hz: u32, duty_percent: f32) -> PwmConfig {
    let mut cfg = PwmConfig::default();

    let clock_freq = embassy_rp::clocks::clk_sys_freq();
    let divider: u16 = (clock_freq / (freq_hz * 65535) + 1) as u16;
    let top = (clock_freq / (freq_hz * divider as u32)) as u16 - 1;

    cfg.divider = divider.to_fixed();
    cfg.top = top;
    cfg.invert_a = false;
    cfg.invert_b = true;

    let compare = ((duty_percent / 100.0) * (top as f32 + 1.0)) as u16;
    cfg.compare_a = compare;
    cfg.compare_b = compare;
    cfg
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

    let _adc_alert = Input::new(p.PIN_5, Pull::Up);

    // ADS1115: ADDR pin to GND → default address 0x48
    let mut adc = Ads1x1x::new_ads1115(i2c, ads1x1x::TargetAddr::default());
    match nb::block!(adc.read(ads1x1x::channel::SingleA0)) {
        Ok(val) => info!("ADS1115 AIN0 = {}", val),
        Err(_) => info!("ADS1115 not responding (expected before board is built)"),
    }

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
    // Totem-pole PWM outputs — 4 channels, complementary A/B pairs
    //
    // Slice 3: GPIO6 /GPIO7   → Totem 0
    // Slice 4: GPIO8 /GPIO9   → Totem 1
    // Slice 5: GPIO10/GPIO11  → Totem 2
    // Slice 6: GPIO12/GPIO13  → Totem 3
    // -----------------------------------------------------------------------
    let gauge_cfg = totem_pole_config(200, 0.0);

    let _totem0 = Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, gauge_cfg.clone());
    let _totem1 = Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, gauge_cfg.clone());
    let _totem2 = Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, gauge_cfg.clone());
    let _totem3 = Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_12, p.PIN_13, gauge_cfg);

    info!("All peripherals initialized");

    spawner.spawn(animate_status_led(status_led)).unwrap();

    loop {
        embassy_futures::yield_now().await;
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