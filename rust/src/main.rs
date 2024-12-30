//! This is a firmware for the Raspberry Pi Pico-W that talks to a CC1101
//! radio controller.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![allow(async_fn_in_trait)]

mod command_parser;
mod net_ui;

use cc1101;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::Pio;
use embassy_usb_logger::ReceiverHandler;
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Pio0Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

static USB_SERIAL_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    command_parser::Command,
    100,
> = embassy_sync::channel::Channel::new();

struct USBSerialHandler {
    parser: embassy_sync::mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        command_parser::Parser,
    >,
}

impl ReceiverHandler for USBSerialHandler {
    fn new() -> Self {
        Self {
            parser: embassy_sync::mutex::Mutex::new(command_parser::Parser::new()),
        }
    }

    async fn handle_data(&self, data: &[u8]) {
        let mut parser = self.parser.lock().await;
        for b in data {
            if let Some(cmd) = parser.ingest(*b) {
                USB_SERIAL_COMMAND_CHANNEL.send(cmd).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn logger_task(driver: embassy_rp::usb::Driver<'static, USB>) {
    embassy_usb_logger::run!(65535, log::LevelFilter::Info, driver, USBSerialHandler);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let mut rng: RoscRng = RoscRng;

    //
    // Set up logging to USB serial port.
    //

    let usb_driver = embassy_rp::usb::Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    //
    // Set up the Wifi.
    //

    let fw = include_bytes!("../../submodules/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../submodules/embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Pio0Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state: &mut cyw43::State = STATE.init(cyw43::State::new());
    let (net_device, mut cyw43_control, cyw43_runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(cyw43_runner)));

    cyw43_control.init(clm).await;
    cyw43_control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Use a link-local address for communication without DHCP server
    // FIXME: should pick an unused link-local address here per the protocol
    let net_config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(169, 254, 1, 1), 16),
        dns_servers: heapless::Vec::new(),
        gateway: None,
    });

    // Generate random seed
    let seed: u64 = rng.next_u64();

    // Init network stack
    static STACK_RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
    let r: &'static mut embassy_net::StackResources<3> =
        STACK_RESOURCES.init(embassy_net::StackResources::new());

    let (stack, runner) = embassy_net::new(net_device, net_config, r, seed);

    unwrap!(spawner.spawn(net_task(runner)));

    cyw43_control.start_ap_open("pico", 5).await;
    //control.start_ap_wpa2("pico", "password", 5).await;

    // Wait a bit to let the serial terminal reconnect.
    embassy_time::Timer::after(embassy_time::Duration::from_millis(1_500)).await;

    //
    // Set up SPI to the CC1101.
    //

    let cc1101_miso = p.PIN_4;
    let cc1101_mosi = p.PIN_3;
    let cc1101_clk = p.PIN_2;
    let cc1101_cs = p.PIN_1;

    let mut spi_config = embassy_rp::spi::Config::default();
    spi_config.frequency = 5_000_000;

    let cc1101_spi_bus = embassy_rp::spi::Spi::new_blocking(
        p.SPI0,
        cc1101_clk,
        cc1101_mosi,
        cc1101_miso,
        spi_config,
    );

    let cc1101_cs = Output::new(cc1101_cs, Level::Low);

    let cc1101_spi_device =
        embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(cc1101_spi_bus, cc1101_cs);

    let mut cc1101 = cc1101::Cc1101::new(cc1101_spi_device).unwrap();

    //
    // Configure the CC1101 for OOK at 433 MHz, 3 kbaud.
    //

    cc1101
        .set_fifo_threshold(true, 0, cc1101::FifoThreshold::TX_61_RX_4)
        .unwrap();

    // PACKET_LENGTH = 4 bytes: FIXME: seems low...
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::PKTLEN, 0x04)
        .unwrap();

    // PQT = 0: disable preamble quality estimator threshold, not because this application is TX only
    // CRC_AUTOFLUSH = 0: don't flush RX FIFO on CRC failure
    // APPEND_STATUS = 1: append RSSI and LQI to received packets, not used for this application
    // ADR_CHK = 0: no address check
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::PKTCTRL1, 0x04)
        .unwrap();

    // Packet format:
    // * whitening off
    // * use FIFOs for Rx and Tx
    // * disable CRC calculation on Tx and CRC check on Rx
    // * fixed packet length specified by PKTLEN register
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::PKTCTRL0, 0x00)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::ADDR, 0x00)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::CHANNR, 0x00)
        .unwrap();

    // Set IF (intermediate frequency).
    // FIXME

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCTRL1, 0x0c)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCTRL0, 0x00)
        .unwrap();

    // Input bandwidth, ~203 kHz
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MDMCFG4, 0x8c)
        .unwrap();

    cc1101.set_data_rate(115_200).unwrap();

    // Set modem to ASK/OOK
    // 0x30: ook, no preamble
    // 0x33: ook, preamble
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MDMCFG2, 0x33)
        .unwrap();

    // Disable FEC, 2 preamble bytes
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MDMCFG1, 0x22)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MDMCFG0, 0xf8)
        .unwrap();

    // Not used for OOK
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::DEVIATN, 0x62)
        .unwrap();

    // Main Radio Control State Machine Configuration 1:
    // * CCA_MODE=3 ??
    // * RXOFF_MODE=0 (go to IDLE after receiving a packet)
    // * TXOFF_MODE=0 (go to IDLE after transmitting a packet)
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MCSM1, 0x30)
        .unwrap();

    // Main Radio Control State Machine Configuration 0:
    // * FS_AUTOCAL=3 (calibrate every 4th time when going from RX or TX to IDLE)
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::MCSM0, 0x30)
        .unwrap();

    // Configure for OOK per Design Note DN022.
    // * AGCCTRL2 = 0x03 to 0x07
    // * AGCCTRL1 = 0x00
    // * AGCCTRL0 = 0x91 or 0x92

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::AGCCTRL2, 0x03)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::AGCCTRL1, 0x00)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::AGCCTRL0, 0x91)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCAL3, 0xea)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCAL2, 0x2a)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCAL1, 0x00)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FSCAL0, 0x1f)
        .unwrap();

    // FREND1:
    //     RX filter bandwidth > 101 kHz, FREND1 = 0xB6
    //     RX filter bandwidth ≤ 101 kHz, FREND1 = 0x56
    // TEST2:
    //     RX filter bandwidth > 325 kHz, TEST2 = 0x88
    //     RX filter bandwidth ≤ 325 kHz, TEST2 = 0x81
    // TEST1:
    //     RX filter bandwidth > 325 kHz, TEST1 = 0x31
    //     RX filter bandwidth ≤ 325 kHz, TEST1 = 0x35

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FREND1, 0xb6)
        .unwrap();
    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FREND0, 0x11)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::TEST2, 0x88)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::TEST1, 0x31)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::TEST0, 0x09)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::FOCCFG, 0x1d)
        .unwrap();

    cc1101
        .0
        .write_register(cc1101::lowlevel::registers::Config::BSCFG, 0x1c)
        .unwrap();

    let patable_values: [u8; 2] = [0x00, 0xc6];
    cc1101.write_patable(&patable_values).unwrap();

    // Flush the TX FIFO
    cc1101
        .0
        .write_cmd_strobe(cc1101::lowlevel::registers::Command::SFTX)
        .unwrap();

    // Calibrate the frequency synthesizer.
    cc1101.set_radio_mode(cc1101::RadioMode::Calibrate).unwrap();

    // Ok, now we're ready.

    // And now we can use it!

    static NET_COMMAND_CHANNEL: embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        command_parser::Command,
        100,
    > = embassy_sync::channel::Channel::new();

    spawner
        .spawn(net_ui::net_ui_task(stack, &NET_COMMAND_CHANNEL))
        .unwrap();

    loop {
        let command: command_parser::Command = match embassy_futures::select::select(
            NET_COMMAND_CHANNEL.receiver().receive(),
            USB_SERIAL_COMMAND_CHANNEL.receiver().receive(),
        )
        .await
        {
            embassy_futures::select::Either::First(command) => command,
            embassy_futures::select::Either::Second(command) => command,
        };

        match command {
            command_parser::Command::Ping => {
                log::info!("Pong");
            }

            command_parser::Command::TxPreambleBytes(num_preamble_bytes) => {
                log::info!("tx-preamble-bytes {}", num_preamble_bytes);

                let r = cc1101
                    .0
                    .read_register(cc1101::lowlevel::registers::Config::MDMCFG1)
                    .unwrap();
                let mdmcfg1 = cc1101::lowlevel::registers::MDMCFG1(r);
                let mut mdmcfg1 = mdmcfg1.modify();

                // Setting | Number of preamble bytes
                // 0 (000) | 2
                // 1 (001) | 3
                // 2 (010) | 4
                // 3 (011) | 6
                // 4 (100) | 8
                // 5 (101) | 12
                // 6 (110) | 16
                // 7 (111) | 24

                let num_preamble_field = match num_preamble_bytes {
                    x if x <= 2 => 0,
                    x if x == 3 => 1,
                    x if x == 4 => 2,
                    x if x <= 6 => 3,
                    x if x <= 8 => 4,
                    x if x <= 12 => 5,
                    x if x <= 16 => 6,
                    x if x <= 24 => 7,
                    _ => 7,
                };
                mdmcfg1.num_preamble(num_preamble_field);

                cc1101
                    .0
                    .write_register(cc1101::lowlevel::registers::Config::MDMCFG1, mdmcfg1.bits())
                    .unwrap();
            }

            command_parser::Command::SyncWordMSB(n) => {
                log::info!("sync-word-msb {}", n);
                let sync1 = cc1101::lowlevel::registers::SYNC1(n);
                cc1101
                    .0
                    .write_register(cc1101::lowlevel::registers::Config::SYNC1, sync1.bits())
                    .unwrap();
            }

            command_parser::Command::SyncWordLSB(n) => {
                log::info!("sync-word-lsb {}", n);
                let sync0 = cc1101::lowlevel::registers::SYNC0(n);
                cc1101
                    .0
                    .write_register(cc1101::lowlevel::registers::Config::SYNC0, sync0.bits())
                    .unwrap();
            }

            command_parser::Command::SyncMode(n) => {
                log::info!("sync-mode {}", n);

                let r = cc1101
                    .0
                    .read_register(cc1101::lowlevel::registers::Config::MDMCFG2)
                    .unwrap();
                let mdmcfg2 = cc1101::lowlevel::registers::MDMCFG2(r);
                let mut mdmcfg2 = mdmcfg2.modify();
                mdmcfg2.sync_mode(n & 0x7);

                cc1101
                    .0
                    .write_register(cc1101::lowlevel::registers::Config::MDMCFG2, mdmcfg2.bits())
                    .unwrap();
            }

            command_parser::Command::Frequency(n) => {
                log::info!("frequency {}", n);
                cc1101.set_frequency(n as u64).unwrap();
            }

            command_parser::Command::Baud(n) => {
                log::info!("baud {}", n);
                cc1101.set_data_rate(n as u64).unwrap();
            }

            command_parser::Command::PacketLength(n) => {
                log::info!("packet length {}", n);
                cc1101
                    .set_packet_length(cc1101::PacketLength::Fixed(n))
                    .unwrap();
            }

            command_parser::Command::Idle => {
                log::info!("idle");
                cc1101.set_radio_mode(cc1101::RadioMode::Idle).unwrap();
            }

            command_parser::Command::TxFifo(byte) => {
                log::info!("tx-fifo 0x{:02x}", byte);
                cc1101
                    .0
                    .write_register(cc1101::lowlevel::registers::MultiByte::FIFO, byte)
                    .unwrap();
            }

            command_parser::Command::DoTx => {
                log::info!("do-tx");
                cc1101.enable_tx().unwrap();
            }
        }
    }
}
