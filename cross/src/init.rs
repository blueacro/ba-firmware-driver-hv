use cortex_m::peripheral::NVIC;
use hal::pac::{interrupt, Peripherals, SUPC};
use hal::usb::UsbBus;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::globals;

pub fn init_usb(nvic: &mut NVIC, allocator: UsbBusAllocator<UsbBus>) {
    let bus_allocator = unsafe {
        globals::USB_ALLOCATOR = Some(allocator);
        globals::USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        globals::USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        globals::USB_BUS = Some(
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

pub fn init_supc(peripherals: &mut Peripherals) {
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
}
