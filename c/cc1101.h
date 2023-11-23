#ifndef __CC1101_H
#define __CC1101_H

#include "pico.h"
#include "hardware/spi.h"
#include "hardware/timer.h"


// Register addresses
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

// Addresses of the multibyte registers.
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

// Addresses of Command Strobes
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


// Names of the registers (normal and multi-byte), indexed by address.
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


// Names of command strobes.  The index here is `addr - 0x30`.
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


typedef struct {
    spi_inst_t * spi;
    uint sck_gpio;
    uint tx_gpio;
    uint rx_gpio;
    uint csn_gpio;

    bool debug;
    bool panic_on_error;
} cc1101_t;


#define cc1101_debug(cc1101, fmt, ...) \
    do { if (cc1101->debug) printf(fmt, ## __VA_ARGS__); } while (0)


void cc1101_set_debug(cc1101_t * cc1101, bool debug) {
    cc1101->debug = debug;
}


void cc1101_set_panic_on_error(cc1101_t * cc1101, bool panic_on_error) {
    cc1101->panic_on_error = panic_on_error;
}


cc1101_t * cc1101_init(
    spi_inst_t * spi,
    uint spi_sck_gpio,
    uint spi_tx_gpio,
    uint spi_rx_gpio,
    uint spi_csn_gpio
) {
    cc1101_t * cc1101 = (cc1101_t*)calloc(1, sizeof(cc1101_t));
    if (cc1101 == nullptr) {
        return nullptr;
    }

    cc1101->spi = spi;
    cc1101->sck_gpio = spi_sck_gpio;
    cc1101->tx_gpio = spi_tx_gpio;
    cc1101->rx_gpio = spi_rx_gpio;
    cc1101->csn_gpio = spi_csn_gpio;

    gpio_init(cc1101->sck_gpio);
    gpio_set_function(cc1101->sck_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(cc1101->sck_gpio, true);  // output

    gpio_init(cc1101->tx_gpio);
    gpio_set_function(cc1101->tx_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(cc1101->tx_gpio, true);  // output

    gpio_init(cc1101->rx_gpio);
    gpio_set_function(cc1101->rx_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(cc1101->rx_gpio, false);  // input

    gpio_init(cc1101->csn_gpio);
    gpio_set_function(cc1101->csn_gpio, GPIO_FUNC_SIO);
    gpio_set_dir(cc1101->csn_gpio, true);  // output

    spi_init(cc1101->spi, 5 * 1000 * 1000);

    // spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    cc1101->debug = false;
    cc1101->panic_on_error = false;

    return cc1101;
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


//
// Read one byte from the register `addr`.  Return the status byte in
// `status` and the register value in `value`.  Returns True on success,
// False on failure.
//

static bool cc1101_read_register(cc1101_t * cc1101, uint8_t const addr, uint8_t * status, uint8_t * value) {
    int r;
    uint8_t out = 0x80 | (addr & 0x3f);  // read register, no burst

    gpio_put(cc1101->csn_gpio, 0);

    r = spi_write_read_blocking(cc1101->spi, &out, status, sizeof(out));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register address write: %d\n", r);
        } else {
            printf("failed spi register address write: %d\n", r);
            return false;
        }
    }

    r = spi_read_blocking(cc1101->spi, 0x00, value, sizeof(*value));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi read: %d\n", r);
        } else {
            printf("failed spi read: %d\n", r);
            return false;
        }
    }

    gpio_put(cc1101->csn_gpio, 1);

    cc1101_debug(cc1101, "read addr %s(0x%02x)=0x%02x (status=%s)\n", cc1101_register_name[addr], addr, *value, cc1101_status_decode(*status));

    return true;
}


//
// Read `count` bytes in burst mode from the register `addr`.  Return the
// one status byte in `status` and the `count` register values in `value`.
// Returns True on success, False on failure.
//

static bool cc1101_read_registers(cc1101_t * cc1101, uint8_t const addr, uint8_t * status, uint8_t * value, size_t count) {
    int r;
    uint8_t out = 0x80 | 0x40 | (addr & 0x3f);  // read register with burst

    gpio_put(cc1101->csn_gpio, 0);

    r = spi_write_read_blocking(cc1101->spi, &out, status, sizeof(out));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register address write: %d\n", r);
        } else {
            printf("failed spi register address write: %d\n", r);
            return false;
        }
    }

    r = spi_read_blocking(cc1101->spi, 0x00, value, count);
    if (r != (int)count) {
        if (cc1101->panic_on_error) {
            panic("failed spi read: %d\n", r);
        } else {
            printf("failed spi read: %d\n", r);
            return false;
        }
    }

    gpio_put(cc1101->csn_gpio, 1);

    cc1101_debug(cc1101, "burst read %u bytes from addr %s(0x%02x) (status=%s):\n", count, cc1101_register_name[addr], addr, cc1101_status_decode(*status));
    if (cc1101->debug) {
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

static bool cc1101_write_register(cc1101_t * cc1101, uint8_t const addr, uint8_t const value, uint8_t * status0, uint8_t * status1) {
    int r;
    uint8_t out = addr & 0x3f;  // write register, no burst

    gpio_put(cc1101->csn_gpio, 0);

    r = spi_write_read_blocking(cc1101->spi, &out, status0, sizeof(out));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register address write: %d\n", r);
        } else {
            printf("failed spi register address write: %d\n", r);
            return false;
        }
    }

    r = spi_write_read_blocking(cc1101->spi, &value, status1, sizeof(value));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register data write: %d\n", r);
        } else {
            printf("failed spi register data write: %d\n", r);
            return false;
        }
    }

    gpio_put(cc1101->csn_gpio, 1);

    cc1101_debug(cc1101, "write addr %s(0x%02x)=0x%02x (status=%s)\n", cc1101_register_name[addr], addr, value, cc1101_status_decode(*status0));

    return true;
}


//
// Write `count` bytes to the register `addr`.  Return the `count`+1
// status bytes in `status` (one for writing the address, plus `count`
// for writing the data).  Returns True on success, False on failure.
//

static bool cc1101_write_registers(cc1101_t * cc1101, uint8_t const addr, uint8_t const * value, uint8_t * status, size_t count) {
    int r;
    uint8_t out = 0x40 | (addr & 0x3f);  // write register with burst

    gpio_put(cc1101->csn_gpio, 0);

    r = spi_write_read_blocking(cc1101->spi, &out, &status[0], sizeof(out));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register address write: %d\n", r);
        } else {
            printf("failed spi register address write: %d\n", r);
            return false;
        }
    }

    r = spi_write_read_blocking(cc1101->spi, value, &status[1], count);
    if (r != (int)count) {
        if (cc1101->panic_on_error) {
            panic("failed spi register data write: %d\n", r);
        } else {
            printf("failed spi register data write: %d\n", r);
            return false;
        }
    }

    gpio_put(cc1101->csn_gpio, 1);

    cc1101_debug(cc1101, "burst write %u bytes to addr %s(0x%02x) (status=%s):\n", count, cc1101_register_name[addr], addr, cc1101_status_decode(*status));
    if (cc1101->debug) {
        size_t i = 0;
        for (size_t i = 0; i < count; ++i) {
            if (i % 8 == 0) printf("    ");
            printf("0x%02x ", value[i]);
            if (i % 4 == 3) printf(" ");
            if (i % 8 == 7) printf("\n");
        }
        if (i % 8 != 7) printf("\n");
    }

    return true;
}


//
// Write a Command Strobe to the register `addr`.  Return the status byte in
// `status`.  Returns True on success, False on failure.
//

static bool cc1101_command_strobe(cc1101_t * cc1101, uint8_t const addr, uint8_t * status) {
    int r;
    uint8_t out = addr & 0x3f;  // write register, no burst

    gpio_put(cc1101->csn_gpio, 0);

    r = spi_write_read_blocking(cc1101->spi, &out, status, sizeof(out));
    if (r != 1) {
        if (cc1101->panic_on_error) {
            panic("failed spi register address write: %d\n", r);
        } else {
            printf("failed spi register address write: %d\n", r);
            return false;
        }
    }

    gpio_put(cc1101->csn_gpio, 1);

    cc1101_debug(cc1101, "command strobe %s(0x%02x) (status=%s)\n", cc1101_strobe_name[addr-0x30], addr, cc1101_status_decode(*status));

    return true;
}


static void cc1101_dump_registers(cc1101_t * cc1101) {
    bool r;
    uint8_t addr;
    uint8_t status0;
    uint8_t value;

    for (addr = 0x00; addr < 0x2f; ++addr) {
        r = cc1101_read_register(cc1101, addr, &status0, &value);
        if (!r) {
            printf("failed to read register %s(0x%02x)\n", cc1101_register_name[addr], addr);
            return;
        }
        printf("read addr %s(0x%02x)=0x%02x (status=%s)\n", cc1101_register_name[addr], addr, value, cc1101_status_decode(status0));
    }

    uint8_t patable[8];
    r = cc1101_read_registers(cc1101, PATABLE, &status0, patable, 8);
    if (!r) {
        printf("failed to read patable\n");
        return;
    }

    printf("patable (status=%s):\n", cc1101_status_decode(status0));
    for (int i = 0; i < 8; ++i) {
        printf("    %d: 0x%02x \n", i, patable[i]);
    }
}


static void cc1101_wait_for_idle(cc1101_t * cc1101) {
    // Wait for status to go back to IDLE
    while (true) {
        int r;
        uint8_t statuses[2];
        uint8_t value;
        r = cc1101_read_registers(cc1101, MARCSTATE, statuses, &value, 1);
        if (r && ((value & 0x1f) == 0x1)) {
            return;
        }
        sleep_us(100);
    }
}


//
// Set the "base frequency" aka "carrier frequency".
//
// Returns true if all went well, false if there was a problem (unless
// panic_on_error is set, in which case it panics if there's a problem).
//

static bool cc1101_set_base_frequency(cc1101_t * cc1101, uint32_t frequency_hz) {
    // f_carrier = (f_xosc/2**16) * FREQ
    // 1/FREQ = (f_xosc/2**16) / f_carrier
    //
    // FREQ =  f_carrier / (f_xosc/2**16)
    //
    // FREQ = 433_920_000 / (26_000_000/2**16)
    // FREQ = 1093745

    constexpr uint32_t f_xosc = 26 * 1000 * 1000;
    constexpr uint32_t divisor = f_xosc / pow(2, 16);
    uint32_t const freq = frequency_hz / divisor;

    int r;
    uint8_t value;
    uint8_t status0, status1;

    value = 0x3f & (freq >> 16);
    r = cc1101_write_register(cc1101, FREQ2, value, &status0, &status1);
    if (!r) {
        return false;
    }

    value = 0xff & (freq >> 8);
    r = cc1101_write_register(cc1101, FREQ1, value, &status0, &status1);
    if (!r) {
        return false;
    }

    value = 0xff & freq;
    r = cc1101_write_register(cc1101, FREQ0, value, &status0, &status1);
    if (!r) {
        return false;
    }

    cc1101_wait_for_idle(cc1101);

    // Calibrate the frequency synthesizer.  We're in IDLE Mode so this
    // is allowed.
    r = cc1101_command_strobe(cc1101, SCAL, &status0);
    if (!r) {
        return false;
    }

    return true;
}


#endif // __CC1101_H
