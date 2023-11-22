#ifndef __CC1101_H
#define __CC1101_H

#define IOCFG2   (0x00)
#define IOCFG1   (0x01)
#define IOCFG0   (0x02)
#define FIFOTHR  (0x03)
#define SYNC1    (0x04)
#define SYNC0    (0x05)
#define PKTLEN   (0x06)
#define PKTCTRL1 (0x07)
#define PKTCTRL0 (0x08)
#define ADDR     (0x09)
#define CHANNR   (0x0a)
#define FSCTRL1  (0x0b)
#define FSCTRL0  (0x0c)
#define FREQ2    (0x0d)
#define FREQ1    (0x0e)
#define FREQ0    (0x0f)
#define MDMCFG4  (0x10)
#define MDMCFG3  (0x11)
#define MDMCFG2  (0x12)
#define MDMCFG1  (0x13)
#define MDMCFG0  (0x14)
#define DEVIATN  (0x15)
#define MCSM2    (0x16)
#define MCSM1    (0x17)
#define MCSM0    (0x18)
#define FOCCFG   (0x19)
#define BSCFG    (0x1a)
#define AGCCTRL2 (0x1b)
#define AGCCTRL1 (0x1c)
#define AGCCTRL0 (0x1d)
#define WOREVT1  (0x1e)
#define WOREVT0  (0x1f)
#define WORCTRL  (0x20)
#define FREND1   (0x21)
#define FREND0   (0x22)
#define FSCAL3   (0x23)
#define FSCAL2   (0x24)
#define FSCAL1   (0x25)
#define FSCAL0   (0x26)
#define RCCTRL1  (0x27)
#define RCCTRL0  (0x28)
#define FSTEST   (0x29)
#define PTEST    (0x2a)
#define AGCTEST  (0x2b)
#define TEST2    (0x2c)
#define TEST1    (0x2d)
#define TEST0    (0x2e)

#define PARTNUM        (0x30)
#define VERSION        (0x31)
#define FREQEST        (0x32)
#define LQI            (0x33)
#define RSSI           (0x34)
#define MARCSTATE      (0x35)
#define WORTIME1       (0x36)
#define WORTIME0       (0x37)
#define PKTSTATUS      (0x38)
#define VCO_VC_DAC     (0x39)
#define TXBYTES        (0x3a)
#define RXBYTES        (0x3b)
#define RCCTRL1_STATUS (0x3c)
#define RCCTRL0_STATUS (0x3d)
#define PATABLE        (0x3e)
#define FIFO           (0x3f)

// Command Strobes
#define SRES    (0x30)
#define SFSTXON (0x31)
#define SXOFF   (0x32)
#define SCAL    (0x33)
#define SRX     (0x34)
#define STX     (0x35)
#define SIDLE   (0x36)
// unused       (0x37)
#define SWOR    (0x38)
#define SPWD    (0x39)
#define SFRX    (0x3a)
#define SFTX    (0x3b)
#define SWORRST (0x3c)
#define SNOP    (0x3d)


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
    "TEST0",
    "(unused)",
    "PARTNUM",
    "VERSION",
    "FREQEST",
    "LQI",
    "RSSI",
    "MARCSTATE",
    "WORTIME1",
    "WORTIME0",
    "PKTSTATUS",
    "VCO_VC_DA",
    "TXBYTES",
    "RXBYTES",
    "RCCTRL1_STATUS",
    "RCCTRL0_STATUS",
    "PATABLE",
    "FIFO"
};


// The index here is `addr - 0x30`.
static char const * cc1101_strobe_name[] = {
    "SRES",
    "SFSTXON",
    "SXOFF",
    "SCAL",
    "SRX",
    "STX",
    "SIDLE",
    "(unused)",
    "SWOR",
    "SPWD",
    "SFRX",
    "SFTX",
    "SWORRST",
    "SNOP",
};


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

    printf("read addr %s(0x%02x)=0x%02x (status=%s)\n", cc1101_register_name[addr], addr, *value, cc1101_status_decode(*status));

    return true;
}


//
// Read `count` bytes in burst mode from the register `addr`.  Return the
// one status byte in `status` and the `count` register values in `value`.
// Returns True on success, False on failure.
//

static bool cc1101_read_registers(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status, uint8_t * value, size_t count) {
    int r;
    uint8_t out = 0x80 | 0x40 | (addr & 0x3f);  // read register with burst

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

    printf("burst read %u bytes from addr %s(0x%02x) (status=%s):\n", count, cc1101_register_name[addr], addr, cc1101_status_decode(*status));
    {
        size_t i;
        for (i = 0; i < count; ++i) {
            if (i % 8 == 0) printf("    ");
            printf("0x%02x ", value[i]);
            if (i % 4 == 3) printf(" ");
            if (i % 8 == 7) printf("\n");
        }
        if (i % 8 != 0) printf("\n");
    }

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

    printf("write addr %s(0x%02x)=0x%02x (status=%s)\n", cc1101_register_name[addr], addr, value, cc1101_status_decode(*status0));

    return true;
}


//
// Write `count` bytes to the register `addr`.  Return the `count`+1
// status bytes in `status` (one for writing the address, plus `count`
// for writing the data).  Returns True on success, False on failure.
//

static bool cc1101_write_registers(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t const * value, uint8_t * status, size_t count) {
    int r;
    uint8_t out = 0x40 | (addr & 0x3f);  // write register with burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, &status[0], sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    r = spi_write_read_blocking(spi, value, &status[1], count);
    if (r != (int)count) {
        printf("failed spi register data write: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    printf("burst write %u bytes to addr %s(0x%02x) (status=%s):\n", count, cc1101_register_name[addr], addr, cc1101_status_decode(*status));
    size_t i = 0;
    for (size_t i = 0; i < count; ++i) {
        if (i % 8 == 0) printf("    ");
        printf("0x%02x ", value[i]);
        if (i % 4 == 3) printf(" ");
        if (i % 8 == 7) printf("\n");
    }
    if (i % 8 != 7) printf("\n");

    return true;
}


//
// Write a Command Strobe to the register `addr`.  Return the status byte in
// `status`.  Returns True on success, False on failure.
//

static bool cc1101_command_strobe(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status) {
    int r;
    uint8_t out = addr & 0x3f;  // write register, no burst

    gpio_put(csn_gpio, 0);
    // printf("cc1101 SO pin is %d\n", gpio_get(spi_rx_gpio));

    r = spi_write_read_blocking(spi, &out, status, sizeof(out));
    if (r != 1) {
        printf("failed spi register address write: %d\n", r);
        return false;
    }

    gpio_put(csn_gpio, 1);

    printf("command strobe %s(0x%02x) (status=%s)\n", cc1101_strobe_name[addr-0x30], addr, cc1101_status_decode(*status));

    return true;
}


static void cc1101_dump_registers(spi_inst_t * spi, uint csn_gpio) {
    bool r;
    uint8_t addr;
    uint8_t status0;
    uint8_t value;

    for (addr = 0x00; addr < 0x2f; ++addr) {
        r = cc1101_read_register(spi1, csn_gpio, addr, &status0, &value);
        if (r) {
            // printf("%s(0x%02x): 0x%02x (status=%s)\n", cc1101_register_name[addr], addr, value, cc1101_status_decode(status0));
        } else {
            printf("failed to read register %s(0x%02x)\n", cc1101_register_name[addr], addr);
        }
    }

    uint8_t patable[8];
    r = cc1101_read_registers(spi1, csn_gpio, PATABLE, &status0, patable, 8);
    if (r) {
        // printf("patable (status=%s):\n", cc1101_status_decode(status0));
        // for (int i = 0; i < 8; ++i) {
            // printf("    %d: 0x%02x \n", i, patable[i]);
        // }
    } else {
        printf("failed to read patable\n");
    }
}


#endif // __CC1101_H
