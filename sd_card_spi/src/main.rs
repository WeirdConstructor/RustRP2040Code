#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::duration::*;
use embedded_time::rate::*;
use embedded_hal::timer::CountDown;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use embedded_sdmmc;

use rp2040_hal as rphal;

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

const LEN : usize = 302;

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

pub struct DummyTimesource {
}

impl DummyTimesource {
    pub fn new() -> Self {
        Self {
        }
    }
}

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac      = pac::Peripherals::take().unwrap();
    let _core        = pac::CorePeripherals::take().unwrap();
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

    let _spi_sclk = pins.gpio2.into_mode::<rphal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<rphal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<rphal::gpio::FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output();
    let spi = rphal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut sdspi = embedded_sdmmc::SdMmcSpi::new(spi, spi_cs);
    let block = sdspi.acquire().unwrap();
    info!("Aquire BlockDevice ok!");

    let mut cont =
        embedded_sdmmc::Controller::new(block, DummyTimesource::new());
    info!("Init SD card...");

    info!("OK!\nCard size...");
    match cont.device().card_size_bytes() {
        Ok(size) => info!("card size={}", size),
        Err(e) => info!("Err: 1"),
    }

    info!("Volume 0...");
    match cont.get_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(mut v) => {
            info!("VOl!");
            match cont.open_root_dir(&v) {
                Ok(dir) => {
                    info!("Root!");
                    cont.iterate_dir(&v, &dir, |ent| {
                        use core::fmt;
                        info!("/{}.{}",
                            core::str::from_utf8(ent.name.base_name()).unwrap(),
                            core::str::from_utf8(ent.name.extension()).unwrap());
                    });

                    let mut file =
                        cont.open_file_in_dir(
                            &mut v, &dir, "O.TST",
                            embedded_sdmmc::filesystem::Mode::ReadOnly).unwrap();
                    let mut buf = [0u8; 32];
                    let nr = cont.read(&mut v, &mut file, &mut buf).unwrap();
                    info!("READ {} bytes: {}", nr, buf);
                    cont.close_file(&v, file);

                    let mut file =
                        cont.open_file_in_dir(
                            &mut v, &dir, "O.TST",
//                            embedded_sdmmc::filesystem::Mode::ReadWriteCreateOrAppend).unwrap();
                            embedded_sdmmc::filesystem::Mode::ReadWriteCreateOrTruncate).unwrap();
                    cont.write(&mut v, &mut file, b"foobar123\n").unwrap();
                    cont.close_file(&v, file);
                },
                Err(e) => {
                    info!("Err: Root {}", defmt::Debug2Format(&e));
                },
            }
        },
        Err(e) => info!("Err: {}", defmt::Debug2Format(&e)),
    }

    cont.free();


    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin_0 = pins.gpio26.into_floating_input();


    let mut n: u8 = 0;
    let mut leds_off : [RGB8; LEN] = [(0,0,0).into(); LEN];
    let mut leds : [RGB8; LEN] = [(0,0,0).into(); LEN];

    for i in 0..LEN {
        leds[i] = (255, 128, 64).into();
    }
//    leds[149] = (255, 255, 255).into();
//    leds[70] = (255, 0, 255).into();
//    leds[100] = (255, 0, 0).into();

    let colors : [RGB8; 3] = [
        hsv2rgb_u8(0.0, 1.0, 1.0).into(),
        (0, 255, 0).into(),
        (0, 0, 255).into(),
    ];

    let amperes = 8.0;
    let all_on_amp = (LEN as f32 * 3.0 * 60.0) / 1000.0;

    let vbrightness = ((amperes / all_on_amp) * 255.0) as u8;
    info!("brightness={} / 255", vbrightness);
    ws.write(brightness(leds_off.iter().copied(), vbrightness)).unwrap();

//    for i in 0..LEN {
//        leds[i] = colors[1];
//    }
//    info!("LEDS={}", leds.len());

    let mut cnt = 0;

    let mut j = 0;

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        cnt += 1;
        if cnt > 400 {
            cnt = 0;
        }

        let clr = hsv2rgb_u8((cnt as f32 / 400.0) * 360.0, 0.0, 1.0);
//        info!("[{}] clr : {}", cnt, clr);
        for i in 0..LEN {
            leds[i] = clr.into();
        }

        ws.write(brightness(leds.iter().copied(), vbrightness)).unwrap();
        delay.start(16.milliseconds());
        let _ = nb::block!(delay.wait());

        info!("on!");
        led_pin.set_high().unwrap();
        delay.start(500.milliseconds());
        let _ = nb::block!(delay.wait());
        info!("off!");
        led_pin.set_low().unwrap();
        delay.start(500.milliseconds());
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
