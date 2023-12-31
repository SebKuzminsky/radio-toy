//! This is a firmware for the Raspberry Pi Pico-W that talks to a CC1101
//! radio controller.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(async_fn_in_trait)]

use cc1101;
use cyw43_pio::PioSpi;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_net::{Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use embassy_rp::peripherals::USB;
use embassy_rp::pio::Pio;
use embedded_io_async::Write;
use static_cell::make_static;

bind_interrupts!(struct Pio0Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
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
    let net_config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::new(169, 254, 1, 1), 16),
        dns_servers: heapless::Vec::new(),
        gateway: None,
    });

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        net_config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    cyw43_control.start_ap_open("pico", 5).await;
    //control.start_ap_wpa2("pico", "password", 5).await;


    embassy_time::Timer::after(embassy_time::Duration::from_secs(3)).await;


    //
    // Set up SPI to the CC1101.
    //

    let cc1101_miso = p.PIN_4;
    let cc1101_mosi = p.PIN_3;
    let cc1101_clk = p.PIN_2;
    let cc1101_cs = p.PIN_1;

    let mut spi_config = embassy_rp::spi::Config::default();
    spi_config.frequency = 5_000_000;
    let cc1101_spi = embassy_rp::spi::Spi::new_blocking(p.SPI0, cc1101_clk, cc1101_mosi, cc1101_miso, spi_config);

    let cc1101_cs = Output::new(cc1101_cs, Level::Low);

    let mut cc1101_handle = cc1101::Cc1101::new(cc1101_spi, cc1101_cs).unwrap();


    //
    // Configure the CC1101 for OOK at 433 MHz, 3 kbaud.
    //

    // FIFOTHR:
    //     RX filter bandwidth > 325 kHz, FIFOTHR = 0x07
    //     RX filter bandwidth ≤ 325 kHz, FIFOTHR = 0x47

    // let r = cc1101_handle.0.read_register(cc1101::lowlevel::registers::Config::FIFOTHR).unwrap();
    // log::info!("FIFOTHR 0x{r:02x}");

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FIFOTHR, 0x40).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::PKTLEN, 0x04).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::PKTCTRL1, 0x04).unwrap();

    // Packet format:
    // * whitening off
    // * use FIFOs for Rx and Tx
    // * disable CRC calculation on Tx and CRC check on Rx
    // * fixed packet length specified by PKTLEN register
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::PKTCTRL0, 0x00).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::ADDR, 0x00).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::CHANNR, 0x00).unwrap();

    // Set IF (intermediate frequency).
    // FIXME

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCTRL1, 0x0c).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCTRL0, 0x00).unwrap();

    // Set the carrier frequency to 433.920 MHz.
    cc1101_handle.set_frequency(433_920_000u64).unwrap();

    // Input bandwidth, ~203 kHz
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG4, 0x8c).unwrap();

    cc1101_handle.set_data_rate(115200u64).unwrap();

    // Set modem to ASK/OOK
    // 0x30: ook, no preamble
    // 0x33: ook, preamble
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG2, 0x33).unwrap();

    // Disable FEC, 2 preamble bytes
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG1, 0x22).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG0, 0xf8).unwrap();

    // Not used for OOK
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::DEVIATN, 0x62).unwrap();

    // Main Radio Control State Machine Configuration 1:
    // * CCA_MODE=3 ??
    // * RXOFF_MODE=0 (go to IDLE after receiving a packet)
    // * TXOFF_MODE=0 (go to IDLE after transmitting a packet)
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MCSM1, 0x30).unwrap();

    // Main Radio Control State Machine Configuration 0:
    // * FS_AUTOCAL=3 (calibrate every 4th time when going from RX or TX to IDLE)
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MCSM0, 0x30).unwrap();

    // Configure for OOK per Design Note DN022.
    // * AGCCTRL2 = 0x03 to 0x07
    // * AGCCTRL1 = 0x00
    // * AGCCTRL0 = 0x91 or 0x92

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::AGCCTRL2, 0x03).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::AGCCTRL1, 0x00).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::AGCCTRL0, 0x91).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCAL3, 0xea).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCAL2, 0x2a).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCAL1, 0x00).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FSCAL0, 0x1f).unwrap();

    // FREND1:
    //     RX filter bandwidth > 101 kHz, FREND1 = 0xB6
    //     RX filter bandwidth ≤ 101 kHz, FREND1 = 0x56
    // TEST2:
    //     RX filter bandwidth > 325 kHz, TEST2 = 0x88
    //     RX filter bandwidth ≤ 325 kHz, TEST2 = 0x81
    // TEST1:
    //     RX filter bandwidth > 325 kHz, TEST1 = 0x31
    //     RX filter bandwidth ≤ 325 kHz, TEST1 = 0x35

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FREND1, 0xb6).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FREND0, 0x11).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::TEST2, 0x88).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::TEST1, 0x31).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::TEST0, 0x09).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::FOCCFG, 0x1d).unwrap();

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::BSCFG, 0x1c).unwrap();

    let patable_values: [u8; 2] = [0x00, 0xc6];
    cc1101_handle.write_patable(&patable_values).unwrap();

    // Flush the TX FIFO
    cc1101_handle.0.write_strobe(cc1101::lowlevel::registers::Command::SFTX).unwrap();

    let tx_bytes = cc1101_handle.0.read_register(cc1101::lowlevel::registers::Status::TXBYTES).unwrap();
    log::info!("{tx_bytes} bytes in TXFIFO");

    // printf("registers after configuration:\n");
    // cc1101_dump_registers(cc1101);

    // sleep_ms(2 * 1000);

    // Calibrate the frequency synthesizer.  We're in IDLE Mode so this
    // is allowed.
    cc1101_handle.set_radio_mode(cc1101::RadioMode::Calibrate).unwrap();

    // Ok, now we're ready.

    // "tx-preamble-bytes": 6

    // register!(MDMCFG1, 0b0010_0010, u8, {
    //     #[doc = "Enable Forward Error Correction"]
    //     fec_en @ 7,
    //     #[doc = "Sets the minimum number of preamble bytes to be transmitted"]
    //     num_preamble @ 4..6,
    //     #[doc = "Exponent of channel spacing"]
    //     chanspc_e @ 0..1,
    // });

    // 0x32
    // fec_en == 0 (0x00)
    // num_preamble == 3 (0x30)
    // _pad == 0 (0x00)
    // chanspc_en = 2 (0x02)

    let old_v = cc1101_handle.0.read_register(cc1101::lowlevel::registers::Config::MDMCFG1).unwrap();
    log::info!("MDMCFG1 old_v is {:02x}", old_v);

    // let mut v = cc1101::lowlevel::registers::MDMCFG1(old_v);
    // v.fec_en = 0;
    // v.num_preamble = cc1101::lowlevel::types::NumPreamble::N_6;
    // v.chanspc_e = 0;

    // log::info!("MDMCFG1 is {:02x}", v);

    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG1, 0x32).unwrap();

    // "sync-word-msb": 0,
    // "sync-word-lsb": 0,
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::SYNC1, 0x00).unwrap();
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::SYNC0, 0x00).unwrap();

    // "sync-mode": 3,
    let dem_dcfilt_off: u8 = 0;
    let mod_format: u8 = 3;
    let manchester_en: u8 = 0;
    let sync_mode: u8 = 3;
    let v: u8 = (dem_dcfilt_off << 7) | (mod_format << 4) | (manchester_en << 3) | (sync_mode);
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::MDMCFG2, v).unwrap();


    // "freq": 433920000,
    // (done above)

    // "baud": 3175,
    cc1101_handle.set_data_rate(3_175).unwrap();

    // "pktlen": 32
    cc1101_handle.0.write_register(cc1101::lowlevel::registers::Config::PKTLEN, 32).unwrap();


    // data
    let data: [u8; 32] = [
        0xe8, 0xe8, 0xe8, 0xe8,
        0xe8, 0xe8, 0xe8, 0xee,
        0x00, 0x00, 0x00, 0x00,
        0xe8, 0xe8, 0xe8, 0xe8,
        0xe8, 0xe8, 0xe8, 0xee,
        0x00, 0x00, 0x00, 0x00,
        0xe8, 0xe8, 0xe8, 0xe8,
        0xe8, 0xe8, 0xe8, 0xee,
    ];

    for byte in data {
        log::info!("writing to FIFO: 0x{:02x}", byte);
        cc1101_handle.0.write_register(cc1101::lowlevel::registers::Command::FIFO, byte).unwrap();
    }

    cc1101_handle.0.write_strobe(cc1101::lowlevel::registers::Command::STX).unwrap();
    cc1101_handle.await_machine_state(cc1101::lowlevel::types::MachineState::IDLE).unwrap();


//     while(1) {
//         read_serial(cc1101);
//     }


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
