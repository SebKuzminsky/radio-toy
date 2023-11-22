#include <cstdio>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"

#include "cc1101.h"


// Rising edge on this GPIO indicates beginning of a transmission,
// used to trigger a scope for debugging.
uint const debug_gpio = 1;


bool debug_flag = false;

#define debug(fmt, ...) \
    do { if (debug_flag) printf(fmt, ## __VA_ARGS__); } while (0)


// Process commands received from the computer via USB serial.
// Valid input strings:
//     'txN': N is a 32-bit hex string.  Transmit the hex string.
void process_input(char const * in) {
    if (strncasecmp("tx", in, 2) == 0) {
#if 0
        uint32_t data = 0;
        int r;
        r = sscanf(&in[2], "%lx", &data);
        if (r != 1) {
            printf("failed to parse '%s'\n", in);
            return;
        }
        debug("tx 0x%08lx\n", data);
        gpio_put(debug_gpio, 1);
        ook_send(data);
        gpio_put(debug_gpio, 0);
#endif

    } else if (strncasecmp("us-per-bit", in, 10) == 0) {
#if 0
        uint us_per_bit = 0;
        int r;
        r = sscanf(&in[10], "%u", &us_per_bit);
        if (r != 1) {
            printf("failed to parse '%s'\n", in);
            return;
        }
        debug("us-per-bit %u\n", us_per_bit);
        ook_start(us_per_bit);
#endif

    } else if (strncasecmp("debug", in, 5) == 0) {
#if 0
        debug_flag = !debug_flag;
        debug("debug is %d\n", debug_flag);
#endif

    } else {
        printf("unknown input '%s'\n", in);
    }
}


void read_serial(void) {
    static char in[16];
    static int index = 0;

    // Block waiting for an input character.
    in[index] = getchar();
    if (isspace(in[index])) {
        in[index] = '\0';
        process_input(in);
        index = 0;
        return;
    }
    ++index;
    if (index == sizeof(in)) {
        printf("input buffer overflow, discarding input '%s'\n", in);
        index = 0;
    }
}


int main() {
    stdio_init_all();
    clocks_init();
    sleep_ms(3*1000);


    //
    // Set up SPI.
    //

    uint const spi_sck_gpio = 10;
    uint const spi_tx_gpio = 11;
    uint const spi_rx_gpio = 12;
    uint const spi_csn_gpio = 13;

    gpio_init(spi_sck_gpio);
    gpio_set_function(spi_sck_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(spi_sck_gpio, true);  // output

    gpio_init(spi_tx_gpio);
    gpio_set_function(spi_tx_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(spi_tx_gpio, true);  // output

    gpio_init(spi_rx_gpio);
    gpio_set_function(spi_rx_gpio, GPIO_FUNC_SPI);
    gpio_set_dir(spi_rx_gpio, true);  // output

    gpio_set_function(spi_csn_gpio, GPIO_FUNC_SIO);
    gpio_set_dir(spi_csn_gpio, true);  // output

    spi_init(spi1, 5 * 1000 * 1000);

    // spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    // spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    bool r;
    uint8_t addr;
    uint8_t status0, status1;
    uint8_t value;

    printf("registers at power on:\n");
    cc1101_dump_registers(spi1, spi_csn_gpio);

    // Set modem to ASK/OOK
    addr = MDMCFG2;
    value = 0x30;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    // Configure for OOK per Design Note DN022.
    // * AGCCTRL2 = 0x03 to 0x07
    // * AGCCTRL1 = 0x00
    // * AGCCTRL0 = 0x91 or 0x92

    addr = AGCCTRL2;
    value = 0x03;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = AGCCTRL1;
    value = 0x00;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = AGCCTRL0;
    value = 0x91;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    // Set IF frequency.
    //
    // FIXME: i think this is the bandwidth of the receiver, not the
    // frequency of the carrier wave?
    //
    // We want to receive a 3 kHz signal.
    // 3_000 = (26_000_000/2**10) * x
    // 1/x = (26_000_000/2**10)/3_000
    // x = 3_000/(26_000_000/2**10)
    // x = 0.118, round up to 1

    addr = FSCTRL1;
    value = 0x01;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    // FREND1:
    //     RX filter bandwidth > 101 kHz, FREND1 = 0xB6
    //     RX filter bandwidth ≤ 101 kHz, FREND1 = 0x56
    // TEST2:
    //     RX filter bandwidth > 325 kHz, TEST2 = 0x88
    //     RX filter bandwidth ≤ 325 kHz, TEST2 = 0x81
    // TEST1:
    //     RX filter bandwidth > 325 kHz, TEST1 = 0x31
    //     RX filter bandwidth ≤ 325 kHz, TEST1 = 0x35
    // FIFOTHR:
    //     RX filter bandwidth > 325 kHz, FIFOTHR = 0x07
    //     RX filter bandwidth ≤ 325 kHz, FIFOTHR = 0x47

    addr = FREND1;
    value = 0x56;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = TEST2;
    value = 0x81;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = TEST1;
    value = 0x35;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = FIFOTHR;
    value = 0x47;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = FREND0;
    value = 0x11;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }


    // Set the base frequency.
    //
    // f_carrier = (f_xosc/2**16) * FREQ
    // 1/FREQ = (f_xosc/2**16) / f_carrier
    // FREQ =  f_carrier / (f_xosc/2**16)
    //
    // FREQ = 433_920_000 / (26_000_000/2**16)
    // FREQ = 1093745

    uint32_t divisor = 26000000 / pow(2, 16);
    uint32_t freq = 433920000 / divisor;

    addr = FREQ2;
    value = 0x3f & (freq >> 16);
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = FREQ1;
    value = 0xff & (freq >> 8);
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = FREQ0;
    value = 0xff & freq;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = SCAL;
    r = cc1101_command_strobe(spi1, spi_csn_gpio, addr, &status0);
    if (r) {
        printf("command strobe addr 0x%02x (status0=0x%02x)\n", addr, status0);
    } else {
        printf("failed to write command strobe register 0x%02x\n", addr);
    }

    addr = MCSM1;
    value = 0x32;
    r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
    if (r) {
        printf("write addr 0x%02x=0x%02x (status0=0x%02x, status1=0x%02x)\n", addr, value, status0, status1);
    } else {
        printf("failed to write register 0x%02x\n", addr);
    }

    addr = STX;
    r = cc1101_command_strobe(spi1, spi_csn_gpio, addr, &status0);
    if (r) {
        printf("command strobe addr 0x%02x (status0=0x%02x %s)\n", addr, status0, cc1101_status_decode(status0));
    } else {
        printf("failed to write command strobe register 0x%02x\n", addr);
    }

    sleep_ms(2 * 1000);

    printf("registers after configuration:\n");
    cc1101_dump_registers(spi1, spi_csn_gpio);

    addr = FIFO;
    value = 0x00;
    for (;;) {
        r = cc1101_write_register(spi1, spi_csn_gpio, addr, value, &status0, &status1);
        if (r) {
            printf("write addr 0x%02x=0x%02x\n", addr, value);
            printf("    status0=%s\n", cc1101_status_decode(status0));
            printf("    status1=%s\n", cc1101_status_decode(status1));
        } else {
            printf("failed to write register 0x%02x\n", addr);
        }
        ++value;

        // read_serial();
        sleep_ms(10);
    }
}
