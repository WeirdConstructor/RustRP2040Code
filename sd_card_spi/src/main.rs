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
use embedded_sdmmc::filesystem::Mode;

use rp_pico::hal::{
    pac,
    gpio,
    spi,
    clocks::{Clock, init_clocks_and_plls},
    sio::Sio,
    watchdog::Watchdog,
    timer::Timer,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[derive(Default)]
pub struct DummyTimesource {
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

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

//    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let _spi_sclk = pins.gpio2.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output();
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut sdspi = embedded_sdmmc::SdMmcSpi::new(spi, spi_cs);
    let block = sdspi.acquire().unwrap();
    info!("Aquire BlockDevice ok!");

    let mut cont =
        embedded_sdmmc::Controller::new(block, DummyTimesource::default());
    info!("Init SD card...");

    info!("OK!\nCard size...");
    match cont.device().card_size_bytes() {
        Ok(size) => info!("card size={}", size),
        Err(e) => info!("Err: {}", defmt::Debug2Format(&e)),
    }

    info!("Volume 0...");
    match cont.get_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(mut v) => {
            info!("VOl!");
            match cont.open_root_dir(&v) {
                Ok(dir) => {
                    info!("Root!");

                    cont.iterate_dir(&v, &dir, |ent| {
                        info!("/{}.{}",
                            core::str::from_utf8(ent.name.base_name()).unwrap(),
                            core::str::from_utf8(ent.name.extension()).unwrap());
                    }).unwrap();

                    let mut file =
                        cont.open_file_in_dir(
                            &mut v, &dir, "O.TST",
                            Mode::ReadOnly).unwrap();

                    let mut buf = [0u8; 32];
                    let nr = cont.read(&mut v, &mut file, &mut buf).unwrap();
                    cont.close_file(&v, file).unwrap();

                    info!("READ {} bytes: {}", nr, buf);

                    let mut file =
                        cont.open_file_in_dir(
                            &mut v, &dir, "O.TST",
                            Mode::ReadWriteCreateOrTruncate).unwrap();
                    cont.write(&mut v, &mut file, b"foobar123\n").unwrap();
                    cont.close_file(&v, file).unwrap();
                },
                Err(e) => {
                    info!("Err: Root {}", defmt::Debug2Format(&e));
                },
            }
        },
        Err(e) => info!("Err: {}", defmt::Debug2Format(&e)),
    }

    cont.free();

    let mut led_pin = pins.led.into_push_pull_output();

    loop {
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
