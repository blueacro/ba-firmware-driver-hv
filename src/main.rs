#![no_std]
#![no_main]

extern crate blueacro_driver_hv_bsp as hal;
extern crate panic_halt;

use cortex_m::peripheral::NVIC;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::usb::UsbBus;
use hal::watchdog::{Watchdog, WatchdogTimeout};

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = hal::Pins::new(peripherals.PORT);
    let mut sets = pins.split();
    let mut led = sets.led.led_green.into_open_drain_output(&mut sets.port);

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(sets.usb.init(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.MCLK,
            &mut sets.port,
        ));
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
        core.NVIC.set_priority(interrupt::USB_OTHER, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT0, 1);
        core.NVIC.set_priority(interrupt::USB_TRCPT1, 1);
        NVIC::unmask(interrupt::USB_OTHER);
        NVIC::unmask(interrupt::USB_TRCPT0);
        NVIC::unmask(interrupt::USB_TRCPT1);
    }

    let mut delay = Delay::new(core.SYST, &mut clocks);
    delay.delay_ms(400u16);

    let mut wdt = Watchdog::new(peripherals.WDT);
    wdt.start(WatchdogTimeout::Cycles256 as u8);

    loop {
        delay.delay_ms(200u8);
        wdt.feed();
        led.set_high().unwrap();
        delay.delay_ms(200u8);
        wdt.feed();
        led.set_low().unwrap();
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
                        serial.write("A".as_bytes()).unwrap();
                        serial.write(&[c.clone()]).unwrap();
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
