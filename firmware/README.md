```bash
cargo flash --chip RP2040 --release && probe-rs attach --chip RP2040 target/thumbv6m-none-eabi/release/can-module
```