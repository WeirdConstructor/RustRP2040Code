#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::duration::*;
use embedded_hal::timer::CountDown;
use panic_probe as _;

use embedded_hal::adc::OneShot;
use pico::hal::{
    pac,
    clocks::{Clock, init_clocks_and_plls},
    sio::Sio,
    adc::Adc,
    watchdog::Watchdog,
    timer::Timer,
};

use pico::hal::pio::PIOExt;
//use rp2040_hal::pio::PIOExt;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio16.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();


    let mut n: u8 = 0;
    let mut leds : [RGB8; 7] = [
        (0, 0, 0).into(),
        (0, 0, 0).into(),
        (255, 0, 255).into(),
        (0, 255, 255).into(),
        (255, 255, 0).into(),
        (  0, 255, 0).into(),
        (255, 0, 0).into(),
    ];

    let mut avg = MovingAvg::new();

    let mut timer_cnt : usize = 0;
    let mut state_on = false;

    let mut r : u16 = avg.next(adc.read(&mut adc_pin_0).unwrap());
    loop {
        timer_cnt = timer_cnt.wrapping_add(1);
        n = n.wrapping_add(1);

        if n == 255 {
            rot(&mut leds[2..]);
        }

        if n % 8 == 0 {
            r = avg.next(adc.read(&mut adc_pin_0).unwrap());
            info!("adc={}", r);
        }

        if state_on && timer_cnt > (10000 / 16) {
            state_on = false;

        } else if r <= 1000 {
            state_on = true;
            timer_cnt = 0;
        }

        let b = if n > 128 { 128 - (n - 128) } else { n };
        let vbrightness = if !state_on { 0 } else { 64 + (b / 3) };

        ws.write(brightness(leds.iter().copied(), vbrightness)).unwrap();
        delay.start(16.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}

pub fn rot(slice: &mut [RGB8]) {
    let first = slice[0];
    for i in 1..slice.len() {
        slice[i - 1] = slice[i];
    }
    slice[slice.len() - 1] = first;
}
