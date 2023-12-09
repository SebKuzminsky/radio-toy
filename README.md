This is a project inspired by the [Flipper
Zero](https://flipperzero.one/) and the
[ook-remote](https://github.com/SebKuzminsky/ook-remote/tree/master).

The goal is to identify, record, & spoof RF remotes (like the WEN3410
air filter at the hack space).


# Design thoughts


## Radio chip

* [CC1101](https://www.ti.com/product/CC1101).  Lots of modules available.
I have a D-Sun CC1101 and an Ebyte E07-M1101D, they seem about equivalent.
SMA connector, 2x5 male pin header, 2 mounting holes near the antenna.
The D-SUN mounting holes are slightly more convenient.  There's also one
called AS07-M1101S.  For building into a portable product, a module with
solderable through holes instead of the pin header may be preferable.

* [Semtech Sx1276](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276)


## Tethered or untethered?


### Tethered

Connect by USB-C to a laptop or an Android phone (serial terminal or
custom app).

Pros:
* simple to build (no battery & charging)
* simple to talk to (minicom or pySerial)

Cons:
* Conspicuous to have a dongle out and connected.


### Untethered

LiPo battery & USB charging.

LiPo battery options:
* 1x18650: 2200 mAh, 19 mm x 68 mm
* 2x18650: 4400 mAh, 38 mm x 66 mm
* Adafruit Feather battery: 400 mAh, 17.5 mm x 37 mm (fits under a Feather or Pico)

Cons:
* battery & charger
* Bulkier


#### Web UI

The device is a featureless brick, like a USB "power bank".  It has a
USB port (for charging the battery) and an on/off switch.

You connect by Bluetooth or Wifi to a UI on a laptop or phone.  This is
very inconspicuous - you can leave the device in a pocket or backpack
and just play with your phone or laptop.

Or web UI over IP-over-USB,
e.g. <https://github.com/maxnet/pico-webserver>?

I don't know of an RP2040 board with both LiPo and Wifi.  I could
use a Pico W and and external LiPo charger/power supply like this one:
* <https://www.adafruit.com/product/5612>
* <https://shop.pimoroni.com/products/pico-lipo-shim>

BOM:
* Pico W (21 mm wide, 51 mm long)
* CC1101 module of some sort (FIXME)
* Pimoroni Pico LiPo Power Shim (fits under Pico W, solders onto its headers)
* LiPo battery
* 3d printed case

RP2040 board options:
* Pico W
* Seeed Wio RP2040 <https://www.seeedstudio.com/Wio-RP2040-mini-Dev-Board-p-4933.html>
* Arduino Nano RP2040 Connect


# Wiring

```
    Pico GPIO          | Pico Pin | D-Sun CC1101 module
    -------------------+----------+--------------------
    3V3 Out            | 36       | VCC
    GND                | 38       | GND
    -------------------+----------+--------------------
    GPIO 2 (SPI0 SCK)  | 4        | SCK
    GPIO 3 (SPI0 TX)   | 5        | MOSI
                       |          | GDO2
    GPIO 4 (SPI0 RX)   | 6        | MISO
    GPIO 1 (SPI0 CSn)  | 2        | CSN
                       |          | GDO0
    -------------------+----------+--------------------
```


# Firmware

C or Rust?

I really want to use Rust but i'm not sure about software support.

<https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry>
<https://www.alexdwilson.dev/learning-in-public/running-rust-code-on-raspberry-pi-pico>

<https://github.com/rp-rs/rp2040-project-template>
<https://github.com/rp-rs/rp-hal>
<https://github.com/rp-rs/rp-hal-boards/>

Install rustup and make a personal toolchain.  :-/

```
# curl | sh craziness to install rustup
$ rustup toolchain install stable
$ rustup target add thumbv6m-none-eabi
$ cargo install elf2uf2-rs --locked
$ cargo install flip-link
```

Edit .cargo/config.toml to switch to the elf2uf2-rs runner.

Reset/power-cycle the pico while holding the Bootsel button.  Mount the
pico anywhere.

`$ cargo run --release`

`$ sudo mount /dev/sda1 /mnt -o umask=0 && cargo run; sync; sudo umount /mnt`

`$ cargo doc --open`


# To do

console on usb?

Unknown:

    Embassy?

    Pico SPI?

    Web server/GUI?

        egui?

            <https://github.com/emilk/egui>

            <https://www.egui.rs/#demo>

    Does embassy-net do mdns?  it does igmp but doesnt look like mdns

Working:

    Pico W wifi chip (Infineon CYW43439):

        <https://lib.rs/crates/cyw43>

        <https://github.com/embassy-rs/embassy>

        <https://github.com/embassy-rs/embassy/tree/main/cyw43>

    Pico USB console

        <https://github.com/rp-rs/rp-hal/issues/652#issuecomment-1639975385>

        embassy-usb-logger


# Enclosure

The enclosure holds a Raspberry Pi Pico W, a D-SUN CC1101 module, a
Pimoroni "LiPo SHIM for Pico", and a LiPo 18650 battery.  The FreeCAD
files (and STLs) are in the `enclosure/` directory.

M2 wood screws to hold the radio and the Pico-W to their standoffs.

M2.6 wood screws to hold the two parts of the enclosure together.
Predrill \#42 (2.375 mm).


# CC1101 info & projects

<https://docs.rs/cc1101/latest/cc1101/>

<https://github.com/28757B2/cc1101-driver>

<https://github.com/mengguang/cc1101>

<https://github.com/mcore1976/cc1101-tool>

<https://github.com/LSatan/SmartRC-CC1101-Driver-Lib>

<https://github.com/gusgorman402/RFmoggy>

<https://github.com/ea/srxe_cc1101>

<https://www.printables.com/model/537529-raspberry-pi-pico-rp2040-cc1101-tool-chassis>
