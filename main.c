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
shared_log_buffer_t log_buffer;

// ============================================================================
// Core1: Генератор IRQ0 каждые ~54.925ms (18.2 Hz, как в IBM PC)
// ============================================================================
[[noreturn]] void core1_irq_generator(void) {
    // busy_wait_ms(5492);
    constexpr uint32_t timer_interval = 5492; // 549ms на 500Khz, 54.925ms на 5Mhz
    static bool irq_pending = false;

    absolute_time_t next_irq = get_absolute_time();
    next_irq = delayed_by_ms(next_irq, timer_interval);
    while (true) {
        // Проверяем, пора ли генерировать прерывание
        if (absolute_time_diff_us(next_irq, get_absolute_time()) >= 0) {
            // Пора генерировать прерывание
            if (!irq_pending) {
                //printf("INTERRUPT REQUESTED\n");
                gpio_put(INTR_PIN, 1); // Поднять INTR
                //irq_pending = true;
            }

            // Следующее прерывание через 54925µs (~54.925ms, 18.2 Hz). Для теста на 5Khz делаем гораздо реже
            next_irq = delayed_by_ms(next_irq, 5492);
        }

        if (multicore_fifo_rvalid()) {
            // Если данные есть, вычитываем индекс
            uint32_t entry_index = multicore_fifo_pop_blocking_inline();
            const log_entry_t *entry = &log_buffer.buffer[entry_index];

            if (entry->type == LOG_INTA) {
                //printf("[%llu] !! INTA\n", entry->timestamp);
            } else
                if (entry->type == LOG_WRITE && entry->address >= 0xB0000 ) {
                    const uint16_t address = entry->address - 0xB0000;
                    printf("\x1b[%d;%dH%c", 1+(address / 2) / 80, 1+(address / 2) % 80, entry->data & 0xFF);
                } else
                if (0)
                    {
                // Теперь спокойно форматируем и выводим
                const char *type_str;
                switch(entry->type) {
                    case LOG_READ:  type_str = entry->mio ? "<<R MEM" : "<<R PORT"; break;
                    case LOG_WRITE: type_str = entry->mio ? "W>> MEM" : "W>> PORT"; break;
                        default: type_str = "?? UNKNOWN: "; break;
                }

                printf("[%llu] %s 0x%05lx : 0x%04x BHE:%d\n",                       entry->timestamp, type_str, entry->address, entry->data, entry->bhe);
            }
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
    // gpio_pull_up(INTR_PIN);
    gpio_put(INTR_PIN, 0); // По умолчанию LOW


    log_buffer.head = 0;
    // Очищаем FIFO на случай, если там что-то осталось от предыдущего запуска
    multicore_fifo_drain();
    multicore_launch_core1(core1_irq_generator);
}


[[noreturn]] int main() {


    // Overclock to 400 MHz for maximum performance
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    set_sys_clock_hz(PICO_CLOCK_SPEED, true);
    busy_wait_ms(250); // Даем время стабилизироваться напряжению

    stdio_usb_init();

    busy_wait_ms(2500); // Даем время стабилизироваться напряжению


    printf("\033[2Ji8086 booted @ %d KHz\n", I8086_CLOCK_SPEED / KHZ);


    start_cpu_clock(); // Start i8086 clock generator
    reset_cpu(); // Now i8086 can safely start
    pic_init(); // Initialize interrupt controller and start Core1 IRQ generator
    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    // reset_cpu(); // Now i8086 can safely start



    while (true) {
        int c = getchar_timeout_us(10);
        switch (c) {
            case 'R':
            case 'r': {
                printf("\033[2JReseting cpu\n");
                gpio_put(INTR_PIN, 0); // По умолчанию LOW
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
                for (int i = 0; i < 160*5; i += 16) {
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
