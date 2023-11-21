#ifndef __CC1101_H
#define __CC1101_H

#define FIFOTHR  (0x03)
#define FSCTRL1  (0x0b)

#define FREQ2    (0x0d)
#define FREQ1    (0x0e)
#define FREQ0    (0x0f)

#define MDMCFG2  (0x12)
#define MCSM1    (0x17)
#define AGCCTRL2 (0x1b)
#define AGCCTRL1 (0x1c)
#define AGCCTRL0 (0x1d)
#define FREND1   (0x21)
#define TEST2    (0x2c)
#define TEST1    (0x2d)

#define SCAL     (0x33)
#define STX      (0x35)
#define FIFO     (0x3f)


static char const * status_decode(uint8_t const status) {
    static char s[100];

    char const * const states[] = { "IDLE", "RX", "TX", "FSTXON", "CALIBRATE", "SETTLING", "RXFIFO_OVERFLOW", "TXFIFO_OVERFLOW" };
    bool chip_rdyn = status & 0x80;
    int state = (status >> 4) & 0x7;
    int fifo_bytes_available = status & 0xf;

    snprintf(
        s,
        sizeof(s),
        "[%s %s fifo=%d]",
        chip_rdyn ? "!CHIP_RDY" : "CHIP_RDY",
        states[state],
        fifo_bytes_available
    );

    return s;
}



#endif // __CC1101_H
