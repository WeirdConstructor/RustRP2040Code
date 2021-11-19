# Rust Raspberry Pi Pico / Rust rp2040 Resources

Setup your Rust installation for rp2040 development:

```
# Add the rp2040 CPU architecture:
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi

# Useful to creating UF2 images for the RP2040 USB Bootloader
cargo install elf2uf2-rs --locked

# Useful for flashing over the SWD pins using a supported JTAG probe
cargo install --git https://github.com/rp-rs/probe-run.git --branch rp2040-support 

# flip-link detect stack-overflows on the first core
# (which is the only supported target for now.)
cargo install flip-link
```

A broad overview of the rp2040 Rust ecosystem:

- [Github rp-rs org](https://github.com/rp-rs/)
  - [rp2040-project-template](https://github.com/rp-rs/rp2040-project-template)
    You will find infos to setup Rust, probe-run and flip-link here.
  - [rp-hal - rp2040 Hardware Abstraction Layer](https://github.com/rp-rs/rp-hal)
    You will find other useful setup information here.
    - [pico Rust examples](https://github.com/rp-rs/rp-hal/tree/main/boards/pico/examples)
    - [rp2040-hal examples](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples)
- elf2uf2 converter: `cargo install elf2uf2-rs --locked`
- [Raspberry Pi Pico DapperMime CMSIS-DAP Debug Probe UF2 Image](https://github.com/majbthrd/DapperMime/releases/download/20210225/raspberry_pi_pico-DapperMime.uf2)
  - See also: https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md
- [heapless - statically allocated data structures](https://docs.rs/heapless/0.7.8/heapless/)
- [Cortex-M allocator](https://crates.io/crates/alloc-cortex-m)
Use with care, and "it's probably safer to reserve heap space with linker scripts!"

## Assorted Projects / Crates

- [ithinuel ws2812-pio-rs - ws2812 driver that uses the PIO peripheral](https://github.com/ithinuel/ws2812-pio-rs/)
  - There is an example for this here: https://github.com/rp-rs/rp-hal/blob/main/boards/feather_rp2040/examples/feather_neopixel_rainbow.rs
- [WeirdConstructor's collection of Rust stuff](https://github.com/WeirdConstructor/RustRP2040Code)
  - Code for a PWM driven DAC that sythesizes sine and saw waveforms: https://github.com/WeirdConstructor/RustRP2040Code/blob/master/pwm_dac_saw_sampling/src/main.rs
