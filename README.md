This is a project inspired by the [Flipper
Zero](https://flipperzero.one/) and the
[ook-remote](https://github.com/SebKuzminsky/ook-remote/tree/master).


# Desired features

* Record & replay RF remotes (like the WEN3401 air filter at the hack
space).

* Read & clone RFID tags (like the card that gets me into the hack space).

* Read & clone the NFC stuff in credit cards.


# Design thoughts

Raspberry Pi Pico, Micro-SD card.


## Radio chip

* [CC1101](https://www.ti.com/product/CC1101).  Lots of modules available.
I have a D-Sun CC1101 and an Ebyte E07-M1101D, they seem about equivalent.
SMA connector, 2x5 male pin header, 2 mounting holes near the antenna.
There's also one called AS07-M1101S.  For building into a portable
product, a module with solderable through holes instead of the pin header
may be preferable.

* [Semtech Sx1276](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276)


## Tethered or untethered?

Tethered: Connect by USB-C to a laptop or an Android phone (serial
terminal or custom app).

Untethered: LiPo battery & USB-C charging.  Connect by Bluetooth to a
UI on a phone, or have a screen & buttons for a UI,


# Wiring

```
    Pico GPIO          | Pico Pin | D-Sun CC1101 module
    -------------------+----------+--------------------
    3V3 Out            | 36       | VCC
    GND                | 38       | GND
    GPIO 10 (SPI1 SCK) | 14       | SCK
    GPIO 11 (SPI1 TX)  | 15       | MOSI
                       |          | GDO2
    GPIO 12 (SPI1 RX)  | 16       | MISO
    GPIO 13 (SPI1 CSn) | 17       | CSN
                       |          | GDO0
    -------------------+----------+--------------------
```


# CC1101 info & projects

<https://github.com/mengguang/cc1101>

<https://github.com/mcore1976/cc1101-tool>

<https://github.com/LSatan/SmartRC-CC1101-Driver-Lib>

<https://github.com/gusgorman402/RFmoggy>
