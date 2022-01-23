#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::duration::*;
use embedded_hal::timer::CountDown;
use panic_probe as _;

use embedded_hal::adc::OneShot;
use rp_pico::hal::{
    pac,
    clocks::{Clock, init_clocks_and_plls},
    sio::Sio,
    adc::Adc,
    watchdog::Watchdog,
    timer::Timer,
};

use rp_pico::hal::pio::PIOExt;
//use rp2040_hal::pio::PIOExt;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const LEN : usize = 100;

struct MovingAvg {
    buf:    [u16; 30],
    ptr:    u16,
}

impl MovingAvg {
    pub fn new() -> Self {
        Self {
            buf: [0xFFFF; 30],
            ptr: 0,
        }
    }

    pub fn next(&mut self, input: u16) -> u16 {
        self.buf[self.ptr as usize] = input;
        self.ptr = ((self.ptr as usize + 1) % self.buf.len()) as u16;
        let mut sum : usize = 0;
        for b in &self.buf {
            sum += *b as usize;
        }
        (sum / self.buf.len()) as u16
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r_, g_, b_) =
        if        hue >= 0.0   && hue < 60.0 {
            (c, x, 0.0)
        } else if hue >= 60.0  && hue < 120.0 {
            (x, c, 0.0)
        } else if hue >= 120.0 && hue < 180.0 {
            (0.0, c, x)
        } else if hue >= 180.0 && hue < 240.0 {
            (0.0, x, c)
        } else if hue >= 240.0 && hue < 300.0 {
            (x, 0.0, c)
        } else { // if hue >= 300.0 && hue < 360.0 {
            (c, 0.0, x)
        };
//    println!("in: h={}, s={}, v={}, r:{}, g:{}, b: {}", hue, sat, val,
//        (r_ + m) * 255.0,
//        (g_ + m) * 255.0,
//        (b_ + m) * 255.0);
    (r_ + m, g_ + m, b_ + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);
    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8
    )
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio18.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio16.into_mode::<rp_pico::hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio17.into_mode::<rp_pico::hal::gpio::FunctionUart>(),
    );

    let mut uart = rp_pico::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            rp_pico::hal::uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

//    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
//    let mut adc_pin_0 = pins.gpio26.into_floating_input();
//
//
//    let mut n: u8 = 0;
//    let mut leds_off : [RGB8; LEN] = [(0,0,0).into(); LEN];
//
//    for i in 0..LEN {
//        leds[i] = (255, 128, 64).into();
//    }
////    leds[149] = (255, 255, 255).into();
////    leds[70] = (255, 0, 255).into();
////    leds[100] = (255, 0, 0).into();
//
//    let colors : [RGB8; 3] = [
//        hsv2rgb_u8(0.0, 1.0, 1.0).into(),
//        (0, 255, 0).into(),
//        (0, 0, 255).into(),
//    ];
//
//    let amperes = 8.0;
//    let all_on_amp = (LEN as f32 * 3.0 * 60.0) / 1000.0;
//
//    let vbrightness = ((amperes / all_on_amp) * 255.0) as u8;
//    info!("brightness={} / 255", vbrightness);
//    ws.write(brightness(leds_off.iter().copied(), vbrightness)).unwrap();
//
////    for i in 0..LEN {
////        leds[i] = colors[1];
////    }
////    info!("LEDS={}", leds.len());
//
//    let mut cnt = 0;
//
//    let mut j = 0;
//
//    loop {
//        cnt += 1;
//        if cnt > 400 {
//            cnt = 0;
//        }
//
//        let clr = hsv2rgb_u8((cnt as f32 / 400.0) * 360.0, 0.0, 1.0);
////        info!("[{}] clr : {}", cnt, clr);
//        for i in 0..LEN {
//            leds[i] = clr.into();
//        }
//
//    }

    let max_ma = 500.0f32;
    let mut leds : [RGB8; LEN] = [(0,0,0).into(); LEN];

    let real_len = 9;
    loop {
        if uart.uart_is_readable() {
            let mut buf = [0u8; 100];
            if let Ok(len) = uart.read_raw(&mut buf) {
                let s = core::str::from_utf8(&buf[0..len]).unwrap();
                info!("Recv: [{}]", s);
            }

            if buf[0] == b'r' {
                for l in leds.iter_mut() {
                    *l = hsv2rgb_u8(0.0, 1.0, 1.0).into();
                }
            } else if buf[0] == b'y' {
                for l in leds.iter_mut() {
                    *l = hsv2rgb_u8(60.0, 1.0, 1.0).into();
                }
            } else if buf[0] == b'w' {
                for l in leds.iter_mut() {
                    *l = hsv2rgb_u8(0.0, 0.0, 1.0).into();
                }
            } else {
                for l in leds.iter_mut() {
                    *l = hsv2rgb_u8(90.0, 1.0, 1.0).into();
                }
            }

            let vbrightness = limit_to_milliamp(&leds[0..real_len], max_ma);
            info!("Bright={}", vbrightness);

            ws.write(brightness(leds.iter().copied(), vbrightness)).unwrap();
        }
        delay.start(16.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}

/// Returns the brightness from 0 to 255.
pub fn limit_to_milliamp(leds: &[RGB8], max_milliamp: f32) -> u8 {
    let mut sum_ma : f32 = 0.0;

    for l in leds.iter() {
        sum_ma += (l.r as f32 / 255.0) * 60.0;
        sum_ma += (l.g as f32 / 255.0) * 60.0;
        sum_ma += (l.b as f32 / 255.0) * 60.0;
    }

    let factor = max_milliamp / sum_ma;
    if factor > 1.0 { 255u8 }
    else            { (255.0 * factor) as u8 }
}

//pub fn rot(slice: &mut [RGB8]) {
//    let first = slice[0];
//    for i in 1..slice.len() {
//        slice[i - 1] = slice[i];
//    }
//    slice[slice.len() - 1] = first;
//}