//! This example test the RP Pico W on board LED.
//!
//! It does not work with the RP Pico board. See blinky.rs.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(async_fn_in_trait)]

use cc1101;
use cyw43_pio::PioSpi;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::peripherals::USB;
use embassy_rp::pio::{InterruptHandler, Pio};
use embedded_io_async::Write;
use static_cell::make_static;

bind_interrupts!(struct Pio0Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: embassy_rp::usb::Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());


    //
    // Set up logging to USB serial port.
    //

    let usb_driver = embassy_rp::usb::Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();


    //
    // Set up the Wifi.
    //

    let fw = include_bytes!("../submodules/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../submodules/embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Pio0Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let state = make_static!(cyw43::State::new());
    let (net_device, mut cyw43_control, cyw43_runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(cyw43_runner)));

    cyw43_control.init(clm).await;
    cyw43_control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Use a link-local address for communication without DHCP server
    // FIXME: should pick an unused link-local address here per the protocol
    let config = Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(169, 254, 1, 1), 16),
        dns_servers: heapless::Vec::new(),
        gateway: None,
    });

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    cyw43_control.start_ap_open("pico", 5).await;
    //control.start_ap_wpa2("pico", "password", 5).await;


    //
    // Set up SPI to the CC1101.
    //

    let cc1101_miso = p.PIN_4;
    let cc1101_mosi = p.PIN_3;
    let cc1101_clk = p.PIN_2;
    let cc1101_cs = p.PIN_1;

    let mut config = embassy_rp::spi::Config::default();
    config.frequency = 5_000_000;
    let cc1101_spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, cc1101_clk, cc1101_mosi, cc1101_miso, config);

    let cc1101_cs = Output::new(cc1101_cs, Level::Low);

    let mut cc1101_handle = cc1101::lowlevel::Cc1101::new(cc1101_spi, cc1101_cs).unwrap();


    //
    // Peek & poke the cc1101 some.
    //

    let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
    log::info!("MARCSTATE 0x{r:02x}");

    cc1101_handle.write_strobe(cc1101::lowlevel::registers::Command::SRES).unwrap();
    log::info!("SRES");
    embassy_time::Timer::after(embassy_time::Duration::from_millis(250)).await;
    let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
    log::info!("MARCSTATE {r} {r:#02x}");

    cc1101_handle.write_strobe(cc1101::lowlevel::registers::Command::SRX).unwrap();
    log::info!("SRX");
    let poll_delay = embassy_time::Duration::from_millis(100);
    for _ in 0..10 {
        let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
        log::info!("MARCSTATE {r} {r:#02x}");
        embassy_time::Timer::after(poll_delay).await;
    }

    cc1101_handle.write_strobe(cc1101::lowlevel::registers::Command::SRES).unwrap();
    log::info!("SRES");
    embassy_time::Timer::after(embassy_time::Duration::from_millis(250)).await;
    let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
    log::info!("MARCSTATE {r} {r:#02x}");

    cc1101_handle.write_strobe(cc1101::lowlevel::registers::Command::STX).unwrap();
    log::info!("STX");
    let poll_delay = embassy_time::Duration::from_millis(100);
    for _ in 0..10 {
        let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
        log::info!("MARCSTATE {r} {r:#02x}");
        embassy_time::Timer::after(poll_delay).await;
    }

    cc1101_handle.write_strobe(cc1101::lowlevel::registers::Command::SIDLE).unwrap();
    log::info!("SIDLE");
    embassy_time::Timer::after(embassy_time::Duration::from_millis(250)).await;
    let r = cc1101_handle.read_register(cc1101::lowlevel::registers::Status::MARCSTATE).unwrap();
    log::info!("MARCSTATE {r} {r:#02x}");


    // And now we can use it!

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        cyw43_control.gpio_set(0, false).await;
        log::info!("Listening on 169.254.1.1:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        log::info!("Received connection from {:?}", socket.remote_endpoint());
        cyw43_control.gpio_set(0, true).await;

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    log::warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            };

            log::info!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());

            match socket.write_all(&buf[..n]).await {
                Ok(()) => {}
                Err(e) => {
                    log::warn!("write error: {:?}", e);
                    break;
                }
            };
        }
    }
}
