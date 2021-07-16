#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

pub mod globals;
pub mod init;

extern crate alloc;
extern crate ba_driver_hv_bsp as hal;
extern crate panic_rtt_target;

use atsamd_hal::rtc::Datetime;
use ba_postcard_proto as proto;
use core::f32;
use num_traits::float::Float;

use alloc_cortex_m::CortexMHeap;
use atsamd_hal::gpio::{PinId, PinMode};
use atsamd_hal::timer::TimerCounter;
use core::alloc::Layout;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::{exception, ExceptionFrame};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::pwm::{Channel, TCC0Pinout, Tcc0Pwm};
use hal::rtc;
use rtt_target::{rprintln, rtt_init_print};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

fn set_duty_cycle<A: PinId, B: PinMode>(pwm0: &mut Tcc0Pwm<A, B>, percent: f32) {
    let max_duty = pwm0.get_max_duty();
    let value = (max_duty as f32 * percent.clamp(0f32, 1f32)).ceil() as u32;
    let channels = [
        Channel::_0,
        Channel::_1,
        Channel::_2,
        Channel::_3,
        Channel::_4,
        Channel::_5,
    ];
    for channel in channels {
        pwm0.set_duty(channel, value);
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let mut peripherals = Peripherals::take().unwrap();
    // Initialize the supply controller
    init::init_supc(&mut peripherals);

    let mut core = CorePeripherals::take().unwrap();
    let pins = hal::Pins::new(peripherals.PORT);
    let mut sets = pins.split();
    let mut led = sets.led.led_green.into_open_drain_output(&mut sets.port);
    let mut led_red = sets.led.led_red.into_open_drain_output(&mut sets.port);
    let mut led_blue = sets.led.led_blue.into_open_drain_output(&mut sets.port);
    let mut dim_en = sets.dim_en.into_push_pull_output(&mut sets.port);
    let fault = sets.fault;

    // Perform startup, ensure pin configuration is as needed
    led_blue.set_high().unwrap();
    dim_en.set_low().unwrap();

    // Start system clocks
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    let gclk = clocks.gclk0();

    // Configure the RTC. a 1024 Hz clock is configured for us when enabling our
    // main clock. Enable running in dbg mode to keep time
    peripherals
        .RTC
        .mode2_mut()
        .dbgctrl
        .write(|w| w.dbgrun().set_bit());
    let rtc = rtc::Rtc::clock_mode_noreset(peripherals.RTC, 1024.hz(), &mut peripherals.MCLK);
    unsafe {
        globals::RTC = Some(rtc);
    };
    // Setup system allocator
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, 16384) }

    // Configure a periodic PWM update timer
    let tc2_tc3 = clocks.tc2_tc3(&gclk).unwrap();
    let mut update_timer = TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK);
    update_timer.start(2.hz());

    let mut delay = Delay::new(core.SYST, &mut clocks);

    // Enable USB
    let allocator = sets.usb.init(
        peripherals.USB,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut sets.port,
    );
    init::init_usb(&mut core.NVIC, allocator);

    // Enable the PWM source for dimming
    let mut pwm0 = Tcc0Pwm::new(
        &clocks.tcc0_tcc1(&gclk).unwrap(),
        20.khz(),
        peripherals.TCC0,
        TCC0Pinout::Pb15(sets.dim.into_function_g(&mut sets.port)),
        &mut peripherals.MCLK,
    );
    set_duty_cycle(&mut pwm0, 0.02f32);

    delay.delay_ms(400u16);
    led_blue.set_low().unwrap();
    dim_en.set_high().unwrap();
    delay.delay_ms(100u16);

    let mut count: u32 = 0;
    loop {
        let seconds_in_day = (24 * 60 * 60) as u32;
        let sunrise = 8 * 60 * 60;
        let sunset = (8 + 12) * 60 * 60;
        let now = unsafe { globals::RTC.as_mut().map(|rtc| rtc.current_time()).unwrap() };

        let seconds_from_midnight =
            ((now.hours as u32) * 60 * 60) + ((now.minutes as u32) * 60) + (now.seconds as u32);

        rprintln!("seconds from midnight {}", seconds_from_midnight);

        let pwm_value = if seconds_from_midnight > sunrise && seconds_from_midnight < sunset {
            let angle_of_sun = (((seconds_from_midnight as f32) / (seconds_in_day as f32))
                * 2f32
                * core::f32::consts::PI)
                + core::f32::consts::PI;
            rprintln!("angle of sun {}", angle_of_sun);
            angle_of_sun.cos().clamp(0f32, 1f32)
        } else {
            0.00f32
        };
        rprintln!("computed pwm_value of {}", pwm_value);
        set_duty_cycle(&mut pwm0, pwm_value);

        delay.delay_ms(200u8);
        led.set_high().unwrap();
        match fault.is_high() {
            Ok(true) => {
                led_red.set_high().unwrap();
            }
            _ => {
                led_red.set_low().unwrap();
            }
        }
        delay.delay_ms(200u8);

        led.set_low().unwrap();
        count += 1;
    }
}

fn poll_usb() {
    unsafe {
        globals::USB_BUS.as_mut().map(|usb_dev| {
            globals::USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    // Check if we read a whole frame in this transaction
                    if buf[count] == 0u8 {
                        if let Ok(packet) = postcard::from_bytes_cobs::<proto::Command>(&mut buf) {
                            match packet {
                                proto::Command::QueryTime => {
                                    let now = globals::RTC
                                        .as_mut()
                                        .map(|rtc| rtc.current_time())
                                        .unwrap();
                                    let out_packet = proto::Response::TimeIsNow(proto::TimeIsNow {
                                        hours: now.hours,
                                        seconds: now.seconds,
                                        minutes: now.minutes,
                                    });
                                    let output =
                                        postcard::to_slice_cobs(&out_packet, &mut buf).unwrap();
                                    serial.write(output).unwrap();
                                }
                                proto::Command::SetTime(time) => {
                                    let value = Datetime {
                                        seconds: time.seconds,
                                        minutes: time.minutes,
                                        hours: time.hours,
                                        day: 1u8,
                                        month: 1u8,
                                        year: 1u8,
                                    };
                                    let _ = globals::RTC
                                        .as_mut()
                                        .map(|rtc| rtc.set_time(value))
                                        .unwrap();
                                    let now = globals::RTC
                                        .as_mut()
                                        .map(|rtc| rtc.current_time())
                                        .unwrap();
                                    let out_packet = proto::Response::TimeIsNow(proto::TimeIsNow {
                                        hours: now.hours,
                                        seconds: now.seconds,
                                        minutes: now.minutes,
                                    });
                                    let output =
                                        postcard::to_slice_cobs(&out_packet, &mut buf).unwrap();
                                    serial.write(output).unwrap();
                                }
                                _ => (),
                            }
                        }
                        //serial.write(&[c.clone()]).unwrap();
                        //let debug = format!("{:?}", now);
                        //serial.write(debug.as_bytes()).unwrap();
                        //serial.write("\n".as_bytes()).unwrap();
                    }
                };
            });
        });
    };
}

#[interrupt]
fn USB_OTHER() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT0() {
    poll_usb();
}

#[interrupt]
fn USB_TRCPT1() {
    poll_usb();
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
