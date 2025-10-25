#include <stdio.h>

#include "cpu.h"
#include "cpu_bus.h"
#include "config.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
extern uint8_t ram[] __attribute__((aligned(4)));
extern uint8_t ports[] __attribute__((aligned(4)));
extern uint8_t videoram[] __attribute__((aligned(4)));

// ============================================================================
// Core1: Генератор IRQ0 каждые ~54.925ms (18.2 Hz, как в IBM PC)
// ============================================================================
[[noreturn]] void core1_irq_generator(void) {
    constexpr uint32_t timer_interval = 1500; // 549ms на 500Khz, 54.925ms на 5Mhz
    static bool irq_pending = false;

    absolute_time_t next_irq = get_absolute_time();
    next_irq = delayed_by_ms(next_irq, timer_interval);
    while (true) {
        // Проверяем, пора ли генерировать прерывание
        if (absolute_time_diff_us(next_irq, get_absolute_time()) >= 0) {
            // Пора генерировать прерывание
            if (!irq_pending) {
                printf("INTERRUPT REQUESTED\n");
                gpio_put(INTR_PIN, 1); // Поднять INTR
                // irq_pending = true;
            }

            // Следующее прерывание через 54925µs (~54.925ms, 18.2 Hz). Для теста на 5Khz делаем гораздо реже
            next_irq = delayed_by_ms(next_irq, 549);
        }

        tight_loop_contents();
    }
}

// ============================================================================
// Инициализация контроллера прерываний
// ============================================================================
void pic_init(void) {
    // Настройка INTR как выход
    gpio_init(INTR_PIN);
    gpio_set_dir(INTR_PIN, GPIO_OUT);
    gpio_put(INTR_PIN, 0); // По умолчанию LOW

    multicore_launch_core1(core1_irq_generator);
}
[[noreturn]] int main() {
    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
    busy_wait_ms(250); // Даем время стабилизироваться напряжению

    stdio_usb_init();

    busy_wait_ms(2500); // Даем время стабилизироваться напряжению

    start_cpu_clock(); // Start i8086 clock generator
    printf("\033[2Ji8086 booted @ %d KHz\n", I8086_CLOCK_SPEED / KHZ);

    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu(); // Now i8086 can safely start
    pic_init(); // Initialize interrupt controller and start Core1 IRQ generator

    while (true) {
        int c = getchar_timeout_us(10);
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
                break;
            }
            case 'M':
            case 'm': {
                printf("\nMemory dump (first 400 bytes):\n");
                for (int i = 0; i < 0x400; i += 16) {
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
                printf("\nVideo Memory dump \n");
                for (int i = 0; i < 160*25; i += 16) {
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

            case 'P':
            case 'p': {
                printf("\nPorts dump (first 400 bytes):\n");
                for (int i = 0; i < 0x3FF; i += 16) {
                    printf("%04X: ", i);
                    for (int j = 0; j < 16; j++) {
                        uint8_t value = ports[i + j];
                        printf("%02X ", value);
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
