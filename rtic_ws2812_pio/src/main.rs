#![no_std]
#![no_main]

//use cortex_m_rt::entry;
//use embedded_time::duration::*;
//use embedded_hal::timer::CountDown;
use panic_probe as _;
//
//use embedded_hal::adc::OneShot;
//use pico::hal::{
//    pac,
//    clocks::{Clock, init_clocks_and_plls},
//    sio::Sio,
//    adc::Adc,
//    watchdog::Watchdog,
//    timer::Timer,
//};
//
//use pico::hal::pio::PIOExt;
////use rp2040_hal::pio::PIOExt;
//use smart_leds::{brightness, SmartLedsWrite, RGB8};
//use ws2812_pio::Ws2812;
//
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const LEN : usize = 302;

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);
    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8
    )
}

//#[entry]
//fn main() -> ! {
//    info!("Program start");
//    let mut pac      = pac::Peripherals::take().unwrap();
//    let _core         = pac::CorePeripherals::take().unwrap();
//    let mut watchdog = Watchdog::new(pac.WATCHDOG);
//    let sio          = Sio::new(pac.SIO);
//
//    // External high-speed crystal on the pico board is 12Mhz
//    let external_xtal_freq_hz = 12_000_000u32;
//    let clocks =
//        init_clocks_and_plls(
//            external_xtal_freq_hz,
//            pac.XOSC,
//            pac.CLOCKS,
//            pac.PLL_SYS,
//            pac.PLL_USB,
//            &mut pac.RESETS,
//            &mut watchdog)
//        .ok()
//        .unwrap();
//
//    let pins = pico::Pins::new(
//        pac.IO_BANK0,
//        pac.PADS_BANK0,
//        sio.gpio_bank0,
//        &mut pac.RESETS,
//    );
//
//    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
//    let mut delay = timer.count_down();
//
//    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
//    let mut ws = Ws2812::new(
//        pins.gpio16.into_mode(),
//        &mut pio,
//        sm0,
//        clocks.peripheral_clock.freq(),
//        timer.count_down(),
//    );
//
//    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
//    let mut adc_pin_0 = pins.gpio26.into_floating_input();
//
//
//    let mut n: u8 = 0;
//    let mut leds_off : [RGB8; LEN] = [(0,0,0).into(); LEN];
//    let mut leds : [RGB8; LEN] = [(0,0,0).into(); LEN];
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
//        ws.write(brightness(leds.iter().copied(), vbrightness)).unwrap();
//        delay.start(16.milliseconds());
//        let _ = nb::block!(delay.wait());
//    }
//}

//use panic_halt as _;

#[rtic::app(device = pico::hal::pac, peripherals = true)]
mod app {
    use super::*;

    use defmt::*;
    use defmt_rtt as _;

    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::duration::Extensions;
    use pico::{
        hal::{
            self,
            clocks::Clock,
            clocks::init_clocks_and_plls,
            watchdog::Watchdog,
            sio::Sio,
            pio::SM0,
            timer::Timer
        },

        XOSC_CRYSTAL_FREQ,
    };

    use smart_leds::{brightness, SmartLedsWrite, RGB8};
    use ws2812_pio::Ws2812Direct;
    use pico::hal::pio::PIOExt;

    const SCAN_TIME_US: u32 = 1000000;

    #[shared]
    struct Shared {
        timer: Timer,
        alarm: hal::timer::Alarm0,
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        ws: Ws2812Direct<pico::hal::pac::PIO0, SM0, hal::gpio::pin::bank0::Gpio4>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let ws = Ws2812Direct::new(
            pins.gpio4.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        (Shared { timer, alarm, led, ws }, Local {}, init::Monotonics())
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [timer, alarm, ws],
        local = [],
    )]
    fn timer_irq(mut c: timer_irq::Context) {
        let clr : RGB8 = (255, 0, 255).into();
        info!("Sched!");

        c.shared.ws.lock(|ws|
            ws.write([clr].iter().copied()).unwrap());

        let timer = c.shared.timer;
        let alarm = c.shared.alarm;
        (timer, alarm).lock(|t, a| {
            a.clear_interrupt(t);
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });
    }

    #[idle(local = [x: u32 = 0])]
    fn idle(cx: idle::Context) -> ! {
        // Locals in idle have lifetime 'static
        let _x: &'static mut u32 = cx.local.x;

//        hprintln!("idle").unwrap();
        info!("Idle started!");

//        debug::exit(debug::EXIT_SUCCESS); // Exit QEMU simulator

//        let timer = Timer::new(cx.device.TIMER, &mut pac.RESETS);
//        let mut delay = timer.count_down();

        loop {
            cortex_m::asm::nop();

//            delay.start(16.milliseconds());
//            let _ = nb::block!(delay.wait());
        }
    }
}
