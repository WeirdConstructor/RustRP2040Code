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

pub fn hsvt2rgb_u8(hsv: (f32, f32, f32)) -> (u8, u8, u8) {
    let r = hsv2rgb(hsv.0, hsv.1, hsv.2);
    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8
    )
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);
    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8
    )
}

pub fn hex2u8(hx: &[u8]) -> Result<u8, ()> {
    if hx.len() < 2 {
        return Err(());
    }

    let mut res = 0;
    for (i, byte) in hx[0..2].iter().enumerate() {
        let val =
            match byte {
                b'0'        => 0,
                b'1'        => 1,
                b'2'        => 2,
                b'3'        => 3,
                b'4'        => 4,
                b'5'        => 5,
                b'6'        => 6,
                b'7'        => 7,
                b'8'        => 8,
                b'9'        => 9,
                b'a' | b'A' => 10,
                b'b' | b'B' => 11,
                b'c' | b'C' => 12,
                b'd' | b'D' => 13,
                b'e' | b'E' => 14,
                b'f' | b'F' => 15,
                _           => 0,
            };
        let val = if i <= 0 { val << 4 } else { val };
        res |= val;
    }

    Ok(res)
}

pub fn hex2u16(hx: &[u8]) -> Result<u16, ()> {
    if hx.len() < 4 {
        return Err(());
    }

    let mut res = 0u16;
    for (i, byte) in hx[0..4].iter().enumerate() {
        let val =
            match byte {
                b'0'        => 0,
                b'1'        => 1,
                b'2'        => 2,
                b'3'        => 3,
                b'4'        => 4,
                b'5'        => 5,
                b'6'        => 6,
                b'7'        => 7,
                b'8'        => 8,
                b'9'        => 9,
                b'a' | b'A' => 10,
                b'b' | b'B' => 11,
                b'c' | b'C' => 12,
                b'd' | b'D' => 13,
                b'e' | b'E' => 14,
                b'f' | b'F' => 15,
                _           => 0,
            };
        let val =
            match i {
                0     => val << 12,
                1     => val << 8,
                2     => val << 4,
                3 | _ => val,
            };
        res |= val;
    }

    Ok(res)
}

pub fn hex2f32(hx: &[u8]) -> Result<f32, ()> {
    Ok(hex2u8(hx)? as f32 / 255.0)
}

pub fn get_color(code: &[u8]) -> Result<((f32, f32, f32), usize), ()> {
    if code.len() < 6 {
        return Err(());
    }

    let h = hex2f32(&code[0..2])? * 360.0;
    let s = hex2f32(&code[2..4])?;
    let v = hex2f32(&code[4..6])?;

    Ok(((h, s, v), 6))
}

pub fn rd_val(code: &[u8], led_idx: u16, r: u32) -> Result<(u32, usize), ()> {
    let res =
        match code[0] {
            b'r' => (r, 1),
            b'l' => (led_idx as u32, 1),
            b'_' => (hex2u16(&code[1..])? as u32, 5),
            _    => (hex2u8(&code[0..])? as u32, 2),
        };
    Ok(res)
}

// code like:
// #c44ffff c00ffff ceeffff L0009; %l03!
//                               ; /l06 %r02 +r01 !
pub fn exec_wledcode(dt: f32, code: &[u8], strip: &mut [RGB8]) -> Result<(), ()> {
    let mut regs : [(f32, f32, f32); 256] = [(0.0, 0.0, 0.0); 256];
    let mut rp = 0;

    let mut pc : usize = 0;
    let mut sptr = 0;

    for i in 0..strip.len() {
        strip[i] = (0, 0, 0).into();
    }

    let mut slen = 0;

    while pc < code.len() {
        let op = code[pc];
        pc += 1;

        match op {
            // c HHHHHH - Set current color register to color (hex)
            b'c' => {
                let (clr, len) = get_color(&code[pc..])?;
                pc += len;
                regs[rp] = clr;
                rp += 1;
            },
            b'L' => {
                slen = hex2u16(&code[pc..])?;
            },
            b';' => { break; },
            _ => {},
        }
    }

    let start_pc = pc;

    for led_idx in 0..slen {
        let mut r : u32 = 0;

        pc = start_pc;
        while pc < code.len() {
            let op = code[pc];
            pc += 1;

            info!("[{}] {}", led_idx, core::char::from_u32(op as u32).unwrap());
            match op {
                b'%' => {
                    let (a, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    let (b, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    r = a % b;
                },
                b'+' => {
                    let (a, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    let (b, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    r = a.wrapping_add(b);
                },
                b'-' => {
                    let (a, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    let (b, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    r = a.wrapping_sub(b);
                },
                b'/' => {
                    let (a, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    let (b, len) = rd_val(&code[pc..], led_idx, r)?;
                    pc += len;
                    r = a / b;
                },
                b'r' => {
                    let (new_r, len) = rd_val(&code[pc..], led_idx, r)?;
                    r = new_r;
                    pc += len;
                },
                _ => { },
            }
        }

        strip[led_idx as usize] = hsvt2rgb_u8(regs[r as usize]).into();
    }

    Ok(())
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
    let mut code_accum : [u8; 1024] = [0u8; 1024];
    let mut wcode_ptr = 0;
    let mut wcode_len = 0;
    loop {
        if uart.uart_is_readable() {
            let mut buf = [0u8; 100];
            if let Ok(len) = uart.read_raw(&mut buf) {
                let s = core::str::from_utf8(&buf[0..len]).unwrap();
                info!("Recv({}): [{}]", len, s);

                for b in buf[0..len].iter() {
                    match *b {
                        b'#' => { wcode_len = 0; wcode_ptr = 0; },
                        b'!' => { wcode_len = wcode_ptr; },
                        _ => { code_accum[wcode_ptr] = *b; wcode_ptr += 1; }
                    }
                }

                if wcode_len == 0 {
                    continue;
                }
            }
        }

        if let Err(_) = exec_wledcode(0.0, &code_accum[0..wcode_len], &mut leds[..]) {
            info!("err!");
        }

        let vbrightness = limit_to_milliamp(&leds[0..real_len], max_ma);
        //d// info!("Bright={}", vbrightness);

        ws.write(brightness(leds.iter().copied(), vbrightness)).unwrap();
        delay.start((2 * 16).milliseconds());
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

//            if buf[0] == '%' {
//            match buf[0] {
//            }
//                wcode_ptr = 0;
//            } el
//
//            if buf[0] == b'r' {
//                for l in leds.iter_mut() {
//                    *l = hsv2rgb_u8(0.0, 1.0, 1.0).into();
//                }
//            } else if buf[0] == b'y' {
//                for l in leds.iter_mut() {
//                    *l = hsv2rgb_u8(60.0, 1.0, 1.0).into();
//                }
//            } else if buf[0] == b'w' {
//                for l in leds.iter_mut() {
//                    *l = hsv2rgb_u8(0.0, 0.0, 1.0).into();
//                }
//            } else if buf[0] == b'm' {
//                for l in leds.iter_mut() {
//                    *l = hsv2rgb_u8(300.0, 1.0, 1.0).into();
//                }
//            } else {
//                for l in leds.iter_mut() {
//                    *l = hsv2rgb_u8(90.0, 1.0, 1.0).into();
//                }
//            }
