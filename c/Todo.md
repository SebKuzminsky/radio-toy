maybe add all the cc1101's config variables to cc1101_t, have functions
to compute & set them, and a function to flush/sync the config to the chip
(maybe using burst writes)


>64 bytes tx - can the cc1101 signal "low tx fifo" on one of its gpios,
so it can interrupt the rp2040?


What's the smallest rp2040 board with spi & mounting holes?

    Seeed xiao rp2040 is small but doesn't have mounting holes

    Waveshare(?) rp2040-zero is small but doesn't have mounting holes


add spectrum analyser & radio capture/sniffing
