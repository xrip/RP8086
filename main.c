#include "cpu.h"
#include "cpu_bus.h"
#include "config.h"


[[noreturn]] int main()
{
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);

    sleep_ms(25);

    stdio_usb_init();
    sleep_ms(2500);

    start_cpu_clock();  // Start i8086 clock generator
    cpu_bus_init();     // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu();        // Now i8086 can safely start

    while (true)
    {
        __wfi();
        tight_loop_contents();
    }
}
