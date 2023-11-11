This is a project inspired by the [Flipper
Zero](https://flipperzero.one/) and the
[ook-remote](https://github.com/SebKuzminsky/ook-remote/tree/master).


# Design thoughts

Raspberry Pi Pico, Micro-SD card.

## Radio chip

* [CC1101](https://www.ti.com/product/CC1101)

* [Semtech Sx1276](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276)


## Tethered or untethered?

Tethered: Connect by USB-C to a laptop or an Android phone (serial
terminal or custom app).

Untethered: LiPo battery & USB-C charging.  Connect by Bluetooth to a
UI on a phone, or have a screen & buttons for a UI,


# Desired features

* Record & replay RF remotes (like the WEN3401 air filter at the hack
space).

* Read & clone RFID tags (like the card that gets me into the hack space).

* Read & clone the NFC stuff in credit cards.
