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


bool debug_flag = true;

#define debug(fmt, ...) \
    do { if (debug_flag) printf(fmt, ## __VA_ARGS__); } while (0)


// Process commands received from the computer via USB serial.
// Valid input strings:
//     'txN': N is a 32-bit hex string.  Transmit the hex string.
void process_input(cc1101_t * cc1101, char const * in) {
    if (strncasecmp("tx", in, 2) == 0) {
        uint32_t data;
        uint8_t status0, status1;
        int r;
        r = sscanf(&in[2], "%lx", &data);
        if (r != 1) {
            printf("failed to parse '%s'\n", in);
            return;
        }
        debug("tx 0x%08lx\n", data);

        cc1101_write_register(cc1101, FIFO, 0xff & (data >> 24), &status0, &status1);
        cc1101_write_register(cc1101, FIFO, 0xff & (data >> 16), &status0, &status1);
        cc1101_write_register(cc1101, FIFO, 0xff & (data >> 8), &status0, &status1);
        cc1101_write_register(cc1101, FIFO, 0xff & data, &status0, &status1);

        // Start the TX.
        cc1101_command_strobe(cc1101, STX, &status0);

        cc1101_wait_for_idle(cc1101);

#if 0
        gpio_put(debug_gpio, 1);
        ook_send(data);
        gpio_put(debug_gpio, 0);
#endif

#if 0
    } else if (strncasecmp("us-per-bit", in, 10) == 0) {
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

    } else if (strncasecmp("baud", in, 4) == 0) {
        uint32_t baud;
        int r;
        r = sscanf(&in[4], "%lu", &baud);
        if (r != 1) {
            printf("failed to parse '%s'\n", in);
            return;
        }
        debug("baud %lu\n", baud);
        cc1101_set_baudrate(cc1101, baud);

    } else if (strncasecmp("freq", in, 4) == 0) {
        uint32_t freq_hz;
        int r;
        r = sscanf(&in[4], "%lu", &freq_hz);
        if (r != 1) {
            printf("failed to parse '%s'\n", in);
            return;
        }
        debug("freq %lu\n", freq_hz);
        cc1101_set_base_frequency(cc1101, freq_hz);

    } else if (strncasecmp("debug", in, 5) == 0) {
        debug_flag = !debug_flag;
        debug("debug is %d\n", debug_flag);

    } else {
        printf("unknown input '%s'\n", in);
    }
}


void read_serial(cc1101_t * cc1101) {
    static char in[100];
    static int index = 0;

    // Block waiting for an input character.
    in[index] = getchar();
    if (isspace(in[index])) {
        in[index] = '\0';
        process_input(cc1101, in);
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
    uint8_t values[100];
    uint8_t statuses[100];

    stdio_init_all();
    clocks_init();

    sleep_ms(3*1000);  // wait for minicom to reconnect to the pico's serial port


    //
    // Set up SPI.
    //

    constexpr uint spi_sck_gpio = 10;
    constexpr uint spi_tx_gpio = 11;
    constexpr uint spi_rx_gpio = 12;
    constexpr uint spi_csn_gpio = 13;

    cc1101_t * cc1101 = cc1101_init(spi1, spi_sck_gpio, spi_tx_gpio, spi_rx_gpio, spi_csn_gpio);
    if (cc1101 == nullptr) {
        panic("can't initialize spi");
    }
    cc1101_set_debug(cc1101, false);
    cc1101_set_panic_on_error(cc1101, true);


    printf("registers at power on:\n");
    cc1101_dump_registers(cc1101);

    //
    // Configure the CC1101 for OOK at 433 MHz, 3 kbaud.
    //

    bool r;
    uint8_t status0, status1;
    uint8_t value;

    // FIFOTHR:
    //     RX filter bandwidth > 325 kHz, FIFOTHR = 0x07
    //     RX filter bandwidth ≤ 325 kHz, FIFOTHR = 0x47
    cc1101_write_register(cc1101, FIFOTHR, 0x40, &status0, &status1);

    cc1101_write_register(cc1101, PKTLEN, 0x04, &status0, &status1);

    cc1101_write_register(cc1101, PKTCTRL1, 0x04, &status0, &status1);

    // Packet format:
    // * whitening off
    // * use FIFOs for Rx and Tx
    // * disable CRC calculation on Tx and CRC check on Rx
    // * fixed packet length specified by PKTLEN register
    cc1101_write_register(cc1101, PKTCTRL0, 0x00, &status0, &status1);

    cc1101_write_register(cc1101, ADDR, 0x00, &status0, &status1);

    cc1101_write_register(cc1101, CHANNR, 0x00, &status0, &status1);

    // Set IF (intermediate frequency).
    // FIXME

    cc1101_write_register(cc1101, FSCTRL1, 0x0c, &status0, &status1);

    cc1101_write_register(cc1101, FSCTRL0, 0x00, &status0, &status1);

    // Set the carrier frequency to 433.920 MHz.
    cc1101_set_base_frequency(cc1101, 433920000);

    // Input bandwidth, ~203 kHz
    cc1101_write_register(cc1101, MDMCFG4, 0x8c, &status0, &status1);

    cc1101_set_baudrate(cc1101, 115200);

    // Set modem to ASK/OOK
    // 0x30: ook, no preamble
    // 0x33: ook, preamble
    cc1101_write_register(cc1101, MDMCFG2, 0x33, &status0, &status1);

    // Disable FEC, 2 preamble bytes
    cc1101_write_register(cc1101, MDMCFG1, 0x22, &status0, &status1);

    cc1101_write_register(cc1101, MDMCFG0, 0xf8, &status0, &status1);

    // Not used for OOK
    cc1101_write_register(cc1101, DEVIATN, 0x62, &status0, &status1);

    // Main Radio Control State Machine Configuration 1:
    // * CCA_MODE=3 ??
    // * RXOFF_MODE=0 (go to IDLE after receiving a packet)
    // * TXOFF_MODE=0 (go to IDLE after transmitting a packet)
    cc1101_write_register(cc1101, MCSM1, 0x30, &status0, &status1);

    // Main Radio Control State Machine Configuration 0:
    // * FS_AUTOCAL=3 (calibrate every 4th time when going from RX or TX to IDLE)
    cc1101_write_register(cc1101, MCSM0, 0x30, &status0, &status1);


    // Configure for OOK per Design Note DN022.
    // * AGCCTRL2 = 0x03 to 0x07
    // * AGCCTRL1 = 0x00
    // * AGCCTRL0 = 0x91 or 0x92

    cc1101_write_register(cc1101, AGCCTRL2, 0x03, &status0, &status1);
    cc1101_write_register(cc1101, AGCCTRL1, 0x00, &status0, &status1);
    cc1101_write_register(cc1101, AGCCTRL0, 0x91, &status0, &status1);

    cc1101_write_register(cc1101, FSCAL3, 0xea, &status0, &status1);
    cc1101_write_register(cc1101, FSCAL2, 0x2a, &status0, &status1);
    cc1101_write_register(cc1101, FSCAL1, 0x00, &status0, &status1);
    cc1101_write_register(cc1101, FSCAL0, 0x1f, &status0, &status1);

    // FREND1:
    //     RX filter bandwidth > 101 kHz, FREND1 = 0xB6
    //     RX filter bandwidth ≤ 101 kHz, FREND1 = 0x56
    // TEST2:
    //     RX filter bandwidth > 325 kHz, TEST2 = 0x88
    //     RX filter bandwidth ≤ 325 kHz, TEST2 = 0x81
    // TEST1:
    //     RX filter bandwidth > 325 kHz, TEST1 = 0x31
    //     RX filter bandwidth ≤ 325 kHz, TEST1 = 0x35

    cc1101_write_register(cc1101, FREND1, 0xb6, &status0, &status1);
    cc1101_write_register(cc1101, FREND0, 0x11, &status0, &status1);

    cc1101_write_register(cc1101, TEST2, 0x88, &status0, &status1);
    cc1101_write_register(cc1101, TEST1, 0x31, &status0, &status1);
    cc1101_write_register(cc1101, TEST0, 0x09, &status0, &status1);

    cc1101_write_register(cc1101, FOCCFG, 0x1d, &status0, &status1);

    cc1101_write_register(cc1101, BSCFG, 0x1c, &status0, &status1);


    values[0] = 0x00;
    values[1] = 0xc6;
    cc1101_write_registers(cc1101, PATABLE, values, statuses, 2);

    // Flush the TX FIFO
    cc1101_command_strobe(cc1101, SFTX, &status0);

    cc1101_read_register(cc1101, TXBYTES, &status0, &value);
    printf("%d bytes in TXFIFO\n", value);

    printf("registers after configuration:\n");
    cc1101_dump_registers(cc1101);

    sleep_ms(2 * 1000);

    // Calibrate the frequency synthesizer.  We're in IDLE Mode so this
    // is allowed.
    cc1101_command_strobe(cc1101, SCAL, &status0);

    // Wait for status to go back to IDLE
    while (true) {
        r = cc1101_read_registers(cc1101, MARCSTATE, statuses, &value, 1);
        if (r && ((value & 0x1f) == 0x1)) {
            break;
        }
    }

    // Ok, now we're ready.
    while(1) {
        read_serial(cc1101);
    }

#if 0
    uint8_t num = 0x00;
    for (;;) {
        for (int i = 0; i < 0x10; ++i) {
            values[i] = num;
            ++ num;
        }
        cc1101_write_registers(cc1101, FIFO, values, statuses, 0x10);

        cc1101_read_registers(cc1101, TXBYTES, statuses, &value, 1);
        printf("%d bytes in TX FIFO\n", value);

        cc1101_read_registers(cc1101, MARCSTATE, &status0, &value, 1);
        printf("MARCSTATE=0x%02x (status=%s)\n", value, cc1101_status_decode(status0));

        printf("sending packet: ");
        for (int i = 0; i < 0x10; ++i) {
            printf("0x%02x ", values[i]);
        }
        printf("\n");

        // Start the TX.
        cc1101_command_strobe(cc1101, STX, &status0);

        // Wait for status to go back to IDLE
        while (true) {
            r = cc1101_read_registers(cc1101, MARCSTATE, statuses, &value, 1);
            if (r && ((value & 0x1f) == 0x1)) {
                break;
            }
        }

        sleep_ms(1 * 1000);
    }
#endif
}
