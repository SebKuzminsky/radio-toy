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
#define FREND0   (0x22)
#define TEST2    (0x2c)
#define TEST1    (0x2d)

#define SCAL     (0x33)
#define STX      (0x35)
#define PATABLE  (0x3e)
#define FIFO     (0x3f)


static char const * cc1101_register_name[] = {
    "IOCFG2",
    "IOCFG1",
    "IOCFG0",
    "FIFOTHR",
    "SYNC1",
    "SYNC0",
    "PKTLEN",
    "PKTCTRL1",
    "PKTCTRL0",
    "ADDR",
    "CHANNR",
    "FSCTRL1",
    "FSCTRL0",
    "FREQ2",
    "FREQ1",
    "FREQ0",
    "MDMCFG4",
    "MDMCFG3",
    "MDMCFG2",
    "MDMCFG1",
    "MDMCFG0",
    "DEVIATN",
    "MCSM2",
    "MCSM1",
    "MCSM0",
    "FOCCFG",
    "BSCFG",
    "AGCCTRL2",
    "AGCCTRL1",
    "AGCCTRL0",
    "WOREVT1",
    "WOREVT0",
    "WORCTRL",
    "FREND1",
    "FREND0",
    "FSCAL3",
    "FSCAL2",
    "FSCAL1",
    "FSCAL0",
    "RCCTRL1",
    "RCCTRL0",
    "FSTEST",
    "PTEST",
    "AGCTEST",
    "TEST2",
    "TEST1",
    "TEST0"
};


//
// Read one byte from the register `addr`.  Return the status byte in
// `status` and the register value in `value`.  Returns True on success,
// False on failure.
//

static bool cc1101_read_register(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status, uint8_t * value) {
    int r;
    uint8_t out = 0x80 | (addr & 0x3f);  // read register, no burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, status, sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    r = spi_read_blocking(spi, 0x00, value, sizeof(*value));
    if (r != 1) {
        printf("failed spi read: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    // printf("addr 0x%02x: 0x%02x (status=0x%02x)\n", addr, in, status);

    return true;
}


//
// Read `count` bytes in burst mode from the register `addr`.  Return the
// one status byte in `status` and the `count` register values in `value`.
// Returns True on success, False on failure.
//

static bool cc1101_read_registers(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status, uint8_t * value, size_t count) {
    int r;
    uint8_t out = 0xc0 | (addr & 0x3f);  // read register with burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, status, sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    r = spi_read_blocking(spi, 0x00, value, count);
    if (r != (int)count) {
        printf("failed spi read: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    // printf("addr 0x%02x: 0x%02x (status=0x%02x)\n", addr, in, status);

    return true;
}


//
// Write one byte to the register `addr`.  Return the status byte in
// `status`.  Returns True on success, False on failure.
//

static bool cc1101_write_register(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t const value, uint8_t * status0, uint8_t * status1) {
    int r;
    uint8_t out = addr & 0x3f;  // write register, no burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, status0, sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    r = spi_write_read_blocking(spi, &value, status1, sizeof(value));
    if (r != 1) {
        printf("failed spi register data write: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    // printf("addr 0x%02x: 0x%02x (status=0x%02x)\n", addr, in, status);

    return true;
}


//
// Write a Command Strobe to the register `addr`.  Return the status byte in
// `status`.  Returns True on success, False on failure.
//

static bool cc1101_command_strobe(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status0) {
    int r;
    uint8_t out = addr & 0x3f;  // write register, no burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, status0, sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    // printf("addr 0x%02x: 0x%02x (status=0x%02x)\n", addr, in, status);

    return true;
}




static char const * cc1101_status_decode(uint8_t const status) {
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


static void cc1101_dump_registers(spi_inst_t * spi, uint csn_gpio) {
    bool r;
    uint8_t addr;
    uint8_t status0;
    uint8_t value;

    for (addr = 0x00; addr < 0x30; ++addr) {
        r = cc1101_read_register(spi1, csn_gpio, addr, &status0, &value);
        if (r) {
            printf("%s(0x%02x): 0x%02x (status=%s)\n", cc1101_register_name[addr], addr, value, cc1101_status_decode(status0));
        } else {
            printf("failed to read register %s(0x%02x)\n", cc1101_register_name[addr], addr);
        }
    }

    uint8_t patable[8];
    r = cc1101_read_registers(spi1, csn_gpio, addr, &status0, patable, 8);
    if (r) {
        printf("patable (status=%s):\n", cc1101_status_decode(status0));
        for (int i = 0; i < 8; ++i) {
            printf("    %d: 0x%02x \n", i, patable[i]);
        }
    } else {
        printf("failed to read patable\n");
    }
}


#endif // __CC1101_H
