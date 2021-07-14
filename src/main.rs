#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
extern crate ba_driver_hv_bsp as hal;
extern crate panic_rtt_target;

use alloc::format;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;

use core::f32;
use num_traits::float::FloatCore;

use atsamd_hal::gpio::{PinId, PinMode};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::{exception, ExceptionFrame};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::pac::{interrupt, CorePeripherals, Peripherals, SUPC};
use hal::prelude::*;
use hal::pwm::{Channel, TCC0Pinout, Tcc0Pwm};
use hal::rtc;
use hal::usb::UsbBus;
use hal::watchdog::{Watchdog, WatchdogTimeout};
use rtt_target::{rprintln, rtt_init_print};

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut RTC: Option<rtc::Rtc<rtc::ClockMode>> = None;
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

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

fn init_usb(nvic: &mut NVIC, allocator: UsbBusAllocator<UsbBus>) {
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(allocator);
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }
    unsafe {
        nvic.set_priority(interrupt::USB_OTHER, 1);
        nvic.set_priority(interrupt::USB_TRCPT0, 1);
        nvic.set_priority(interrupt::USB_TRCPT1, 1);
        NVIC::unmask(interrupt::USB_OTHER);
        NVIC::unmask(interrupt::USB_TRCPT0);
        NVIC::unmask(interrupt::USB_TRCPT1);
    }
}

fn spin_supc(supc: &SUPC) {
    loop {
        if supc.status.read().b33srdy().bit_is_set() {
            break;
        }
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    // Initialize the allocator BEFORE you use it
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    let mut peripherals = Peripherals::take().unwrap();

    // Configure the supply supervisor
    peripherals.SUPC.bod33.modify(|_, w| w.enable().clear_bit());
    spin_supc(&peripherals.SUPC);
    // Retain no backup ram in backup mode
    peripherals.PM.bkupcfg.write(|w| w.bramcfg().off());
    peripherals.SUPC.bbps.modify(|_, w| w.wakeen().set_bit());
    peripherals.SUPC.bod33.modify(|_, w| unsafe {
        w.action()
            .bkup()
            .level()
            .bits(0x1cu8)
            .runbkup()
            .set_bit()
            .vbatlevel()
            .bits(0x10u8)
    });
    spin_supc(&peripherals.SUPC);
    peripherals.SUPC.bod33.modify(|_, w| w.enable().set_bit());
    spin_supc(&peripherals.SUPC);

    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = hal::Pins::new(peripherals.PORT);
    let mut sets = pins.split();
    let mut led = sets.led.led_green.into_open_drain_output(&mut sets.port);
    let mut led_red = sets.led.led_red.into_open_drain_output(&mut sets.port);
    let mut led_blue = sets.led.led_blue.into_open_drain_output(&mut sets.port);
    let mut dim_en = sets.dim_en.into_push_pull_output(&mut sets.port);
    let fault = sets.fault;

    // Configure the RTC. a 1024 Hz clock is configured for us when enabling our
    // main clock. Enable running in dbg mode to keep time
    peripherals
        .RTC
        .mode2_mut()
        .dbgctrl
        .write(|w| w.dbgrun().set_bit());
    let rtc = rtc::Rtc::clock_mode_noreset(peripherals.RTC, 1024.hz(), &mut peripherals.MCLK);
    unsafe {
        RTC = Some(rtc);
    };

    let mut delay = Delay::new(core.SYST, &mut clocks);

    led_blue.set_high().unwrap();
    dim_en.set_low().unwrap();

    // Enable USB
    let allocator = sets.usb.init(
        peripherals.USB,
        &mut clocks,
        &mut peripherals.MCLK,
        &mut sets.port,
    );
    init_usb(&mut core.NVIC, allocator);

    // Enable the PWM source for dimming
    let gclk = clocks.gclk0();
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
    // Disable enabling the boost converter to allow debugging
    dim_en.set_high().unwrap();
    delay.delay_ms(100u16);

    //let mut wdt = Watchdog::new(peripherals.WDT);
    //wdt.start(WatchdogTimeout::Cycles256 as u8);

    let mut count: u32 = 0;
    loop {
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
        set_duty_cycle(&mut pwm0, ((count + 30) % 200) as f32 / 200.0f32);
        led.set_low().unwrap();
        count += 1;
        rprintln!("looped {}", count);
        unsafe {
            let now = RTC.as_mut().map(|rtc| rtc.current_time());
            rprintln!("now is {:?}", now);
        }
    }
}

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    for (i, c) in buf.iter().enumerate() {
                        if i >= count {
                            break;
                        }

                        serial.write(&[c.clone()]).unwrap();
                        let now = RTC.as_mut().map(|rtc| rtc.current_time());
                        let debug = format!("{:?}", now);
                        serial.write(debug.as_bytes());
                        serial.write("\n".as_bytes()).unwrap();
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
