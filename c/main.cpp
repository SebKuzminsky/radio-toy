#include <cstdio>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"


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


//
// Read one byte from the register `addr`.  Return the status byte in
// `status` and the register value in `value`.  Returns True on success,
// False on failure.
//

bool read_register(spi_inst_t * spi, uint const csn_gpio, uint8_t const addr, uint8_t * status, uint8_t * value) {
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
    uint8_t status;
    uint8_t value;

    while (1) {
        for (uint8_t addr = 0x00; addr < 0x30; ++addr) {
            r = read_register(spi1, spi_csn_gpio, addr, &status, &value);
            if (r) {
                printf("addr 0x%02x: 0x%02x (status=0x%02x)\n", addr, value, status);
            } else {
                printf("failed to read register 0x%02x\n", addr);
            }
            sleep_ms(1);
        }
        sleep_ms(3*1000);
    }

    for (;;) {
        read_serial();
    }
}
