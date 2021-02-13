#![no_std]
#![no_main]

extern crate ba_driver_hv_bsp as hal;
extern crate panic_rtt_target;

use rtt_target::{rtt_init_print, rprintln};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::{exception, ExceptionFrame};
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::entry;
use hal::pac::{interrupt, CorePeripherals, Peripherals};
use hal::pwm::{Channel, TCC0Pinout, Tcc0Pwm};
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
    rtt_init_print!();
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );


    let mut delay = Delay::new(core.SYST, &mut clocks);

    let pins = hal::Pins::new(peripherals.PORT);
    let mut sets = pins.split();
    let mut led = sets.led.led_green.into_open_drain_output(&mut sets.port);
    let mut led_red = sets.led.led_red.into_open_drain_output(&mut sets.port);
    let mut led_blue = sets.led.led_blue.into_open_drain_output(&mut sets.port);
    let mut dim_en = sets.dim_en.into_push_pull_output(&mut sets.port);
    let mut dim_pin = sets.dim.into_push_pull_output(&mut sets.port);
    let mut iset_pin = sets.dim_iset.into_push_pull_output(&mut sets.port);

    dim_pin.set_low().unwrap();
    led_blue.set_high().unwrap();

    iset_pin.set_low().unwrap();
    dim_en.set_low().unwrap();
    rprintln!("iset/dim/dim_en pin set low");
    
    delay.delay_ms(1000u16);
    
    iset_pin.set_high().unwrap();
    dim_en.set_high().unwrap();
    rprintln!("iset/dim_en pin set high");
    led_blue.set_low().unwrap();
    delay.delay_ms(400u16);
    dim_pin.set_high().unwrap();
    rprintln!("dim pin set high");
    /*
    let gclk = clocks.gclk0();
    let mut pwm0 = Tcc0Pwm::new(
        &clocks.tcc0_tcc1(&gclk).unwrap(),
        100.khz(),
        peripherals.TCC0,
        TCC0Pinout::Pb13(sets.dim_iset.into_function_g(&mut sets.port)),
        &mut peripherals.MCLK,
    );
    let max_duty = pwm0.get_max_duty();


    pwm0.set_duty(Channel::_1, max_duty - 10);

*/
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

        //let mut wdt = Watchdog::new(peripherals.WDT);
    //wdt.start(WatchdogTimeout::Cycles256 as u8);

    let mut count: u32 = 0;
    loop {
        delay.delay_ms(200u8);
        led.set_high().unwrap();
        led_red.set_high().unwrap();
        delay.delay_ms(200u8);
        led.set_low().unwrap();
        count += 1;
    rprintln!("looped {}", count);
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

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
