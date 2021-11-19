#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use embedded_hal::PwmPin;
use embedded_time::duration::*;
use panic_probe as _;
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac      = pac::Peripherals::take().unwrap();
    let _core         = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio          = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks =
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

    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut cnt_down = timer.count_down();

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let pwm = &mut pwm_slices.pwm7;
    pwm.enable();
    pwm.set_div_int(1);
    pwm.set_div_frac(0);

    let pins =
        hal::gpio::Pins::new(
            pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let top = 255; // changing this changes the PWM frequency!
    pwm.set_top(top);

    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio15);

    let saw_mode = false;

    use num_traits::float::FloatCore;

    // Formular for RC filter cutoff calc (WLambda):
    // !fc = {!(r, c) = @; 1.0 / (2.0 * 3.14159265359 * r * c) };
    // >> fc 40.0 0.000000100
    // > 39788.735772971224

    use alloc;
    let x : Vec<f32> = alloc::vec::Vec::new();
    x.push(0.2);

    let sin =
        unsafe {
            core::mem::transmute::<_, fn(f32) -> f32>
                (rp2040_hal::rom_data::float_funcs::fsin())
        };

    let mut phase = 0.0;
    loop {
        cnt_down.start((25_u32).microseconds());

        let y =
            if saw_mode {
                phase
            } else {
                (sin(phase * 3.141 * 2.0) + 1.0) * 0.5
            };

        channel.set_duty((y * (top + 1) as f32).floor() as u16);

        phase = phase + (440.0 / 40000.0);
        while phase > 1.0 { phase -= 1.0; }

        let _ = nb::block!(cnt_down.wait());
    }
}


















//    let mut delay =
//        cortex_m::delay::Delay::new(
//            core.SYST,
//            clocks.system_clock.freq().integer());


//    let mut led_pin = pins.gpio25.into_push_pull_output();
//        info!("on!");
//        led_pin.set_high().unwrap();
//        info!("off!");
//        led_pin.set_low().unwrap();

//        duty = (duty + 1) % (steps + 1);
//        channel.set_duty(duty * tPs);
//        let h = rp2040_hal::rom_data::float_funcs::fsin(
//            phase * 3.14159265359 * 2.0);
