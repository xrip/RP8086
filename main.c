#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "config.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
extern uint8_t ram[] __attribute__((aligned(4)));
extern uint8_t videoram[] __attribute__((aligned(4)));

[[noreturn]] int main() {
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
    busy_wait_ms(250); // Даем время стабилизироваться напряжению

    stdio_usb_init();
    busy_wait_ms(2500); // Даем время стабилизироваться напряжению

    start_cpu_clock(); // Start i8086 clock generator
    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu(); // Now i8086 can safely start

    printf("\n\n\ni8086 booted @ %d KHz\n\033[2J", I8086_CLOCK_SPEED / KHZ);

    while (true) {
        int c = getchar_timeout_us(0);
        switch (c) {
            case 'R':
            case 'r': {
                printf("\033[2JReseting cpu\n");
                reset_cpu();
                // watchdog_enable(0, 0);
                break;
            }
            case 'B':
            case 'b': {
                printf("===================== RESET");
                reset_usb_boot(0, 0);
            }
            case 'M':
            case 'm': {
                printf("\nMemory dump (first 400 bytes):\n");
                for (int i = 0; i < 400; i += 16) {
                    printf("%04X: ", i);
                    for (int j = 0; j < 16; j++) {
                        uint8_t value = ram[i + j];
                        printf("%02X ", value);
                    }
                    printf(" | ");
                    for (int j = 0; j < 16; j++) {
                        uint8_t value = ram[i+j];
                        printf("%c", value);
                    }
                    printf("\n");
                }
                break;
            }
            case 'V':
            case 'v': {
                printf("\nV Memory dump (first 400 bytes):\n");
                for (int i = 0; i < 400; i += 16) {
                    printf("%04X: ", i);
                    for (int j = 0; j < 16; j++) {
                        uint8_t value = videoram[i + j];
                        printf("%02X ", value);
                    }
                    printf(" | ");
                    for (int j = 0; j < 16; j++) {
                        uint8_t value = videoram[i+j];
                        printf("%c", value);
                    }
                    printf("\n");
                }
                break;
            }
        }
        __wfi();
        tight_loop_contents();
        stdio_flush();
    }
}
