#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::duration::*;
use embedded_time::rate::Extensions;
use embedded_hal::timer::CountDown;
use panic_probe as _;

use pico::hal::{
    pac,
    clocks::init_clocks_and_plls,
    sio::Sio,
    gpio,
    i2c::I2C,
    watchdog::Watchdog,
    timer::Timer,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self { buf: [0; 64], ptr: 0 }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let len = s.len();
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(s.as_bytes());
        self.ptr += len;
        Ok(())
    }
}


#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac      = pac::Peripherals::take().unwrap();
    let _core         = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio          = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks =
        init_clocks_and_plls(
            external_xtal_freq_hz,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog)
        .ok()
        .unwrap();

    let pins = pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio16.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let mut count = 0;

    let mut image = [0.0f32; 128 * 64];
    draw_grayscale_image(&mut image);

    let mut time = 0;
    let mut mode = 0;
    loop {
        time += 1;

        if mode == 0 {
            if time > 20 {
                mode = 1;
                time = 0;
            }

            count += 1;
            display.clear();
            Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            let mut buf = FmtBuf::new();
            core::fmt::write(&mut buf, format_args!("counter: {}", count)).unwrap();

            Text::with_baseline(buf.as_str(), Point::new(0, 32), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            display.flush().unwrap();

            delay.start(500.milliseconds());
            let _ = nb::block!(delay.wait());

        } else {
            count += 1;

            if time > 100 {
                mode = 0;
                time = 0;
            }

            display.clear();
            for x in 0..128 {
                for y in 0..64 {
                    let sin = pico::hal::rom_data::float_funcs::fsin();

                    let thres = sin(((count % 30) as f32 / 30.0) * 2.0 * core::f32::consts::PI);
                    let thres_01 = (thres + 1.0) * 0.5;

                    if image[x + y * 128] > (30.0 + thres_01 * 80.0) {
                        display.set_pixel(x as u32, y as u32, true);
                    }
                }
            }
            display.flush().unwrap();

            delay.start(16.milliseconds());
            let _ = nb::block!(delay.wait());
        }
    }
}

fn draw_grayscale_image(out: &mut [f32; 64 * 128]) {
    let sin = pico::hal::rom_data::float_funcs::fsin();
    let cos = pico::hal::rom_data::float_funcs::fcos();

    for x in 0..128 {
        for y in 0..64 {
            let xf = x as f32;
            let yf = y as f32;

            let sf = 0.2; // scale factor

            let r = 64.0
                + 63.0
                    * sin(xf / (sf * (37.0 + 15.0 * cos(yf / (sf * 74.0)))))
                    * cos(yf / (sf * (31.0 + 11.0 * sin(xf / (sf * 57.0)))));

            out[x + y * 128] = r;
        }
    }
}
