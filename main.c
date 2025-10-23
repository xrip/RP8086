#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "config.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"


[[noreturn]] int main()
{
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
    busy_wait_ms(250); // Даем время стабилизироваться напряжению

    stdio_usb_init();
    busy_wait_ms(2500); // Даем время стабилизироваться напряжению

    start_cpu_clock();  // Start i8086 clock generator
    cpu_bus_init();     // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu();        // Now i8086 can safely start

    printf("\n\n\ni8086 booted @ %d KHz\n\033[2J", I8086_CLOCK_SPEED / KHZ);

    while (true)
    {
        int c = getchar_timeout_us(0);
        switch (c) {
            case 'R': case 'r': {
                printf("Reseting cpu\n");
                reset_cpu();
                // watchdog_enable(0, 0);
                break;
            }
            case 'B': case 'b': {
                printf("Booting cpu\n");
                reset_usb_boot(0,0);

            }
        }
        __wfi();
        tight_loop_contents();
        stdio_flush();
    }
}
