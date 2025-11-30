// RP2350B Related libs
#include <hardware/pwm.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>

#include "common.h"
#include "graphics.h"

// 8086 related libs
#include "hardware/i8237.h"
#include "hardware/i8259.h"
#include "hardware/i8253.h"
#include "hardware/uart16550.h"

#include "ff.h"
#include "f_util.h"
#include <debug.h>
extern cga_s cga;
extern mc6845_s mc6845;
uint8_t videomode = 0;
repeating_timer_t irq0_timer;

uint8_t current_scancode = 0; // 0 = нет данных
pwm_config pwm;
FATFS fs;

static inline void pic_init(void) {
    // Настройка INTR как выход
    gpio_init(INTR_PIN);
    gpio_set_dir(INTR_PIN, GPIO_OUT);
    gpio_put(INTR_PIN, 0); // По умолчанию LOW
}

// ============================================================================
// Core1: Обработка i8086_bus
// ============================================================================

[[noreturn]] void bus_handler_core(void) {
    start_cpu_clock(); // Start i8086 clock generator
    pic_init(); // Initialize interrupt controller and start Core1 IRQ generator
    cpu_bus_init(); // Initialize bus BEFORE releasing i8086 from reset
    reset_cpu(); // Now i8086 can safely start

    while (true) {
        // Управление сигналом INTR
        gpio_put(INTR_PIN, i8259_get_pending_irqs());

        __wfe();
        tight_loop_contents();
    }
}


// ============================================================================
// Set scancode and trigger IRQ1 (keyboard interrupt)
// ============================================================================
bool handleScancode(const uint8_t ps2scancode) {
    if (ps2scancode == 0x00) return false; // Ignore unknown keys

    current_scancode = ps2scancode;
    i8259_interrupt(1); // IRQ1 - Keyboard interrupt через i8259
    return true;
}

[[noreturn]] int main() {
    // IMPORTANT! Dont remove, hack to create .flashdata section for linker
    extern uint32_t PICO_CLOCK_SPEED_MHZ;
    assert(PICO_CLOCK_SPEED_MHZ == PICO_CLOCK_SPEED);
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_65);
    busy_wait_at_least_cycles((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t) XOSC_HZ) / 1000000);

    qmi_hw->m[0].timing = 0x60007305; // 5x FLASH divisor

    set_sys_clock_hz(PICO_CLOCK_SPEED_MHZ, true);

    psram_init(47);


    tusb_init();
    keyboard_init();
    mouse_init();  // Инициализация поддержки Microsoft Serial Mouse
    debug_init();
    FIL file;
    // Mount SD card filesystem
    if (FR_OK != f_mount(&fs, "", 1)) {
        // while (!stdio_usb_connected()) { tight_loop_contents(); }
        printf("SD Card not inserted or SD Card error!");
        reset_usb_boot(0, 0);
    }


    if (FR_OK != f_open(&file, "\\XT\\fdd.img", FA_READ | FA_WRITE)) {
        // while (!stdio_usb_connected()) { tight_loop_contents(); }
        printf("Floppy image not found!");
        reset_usb_boot(0, 0);
    }
    pwm = pwm_get_default_config();
    gpio_set_function(BEEPER_PIN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&pwm, 127);
    pwm_init(pwm_gpio_to_slice_num(BEEPER_PIN), &pwm, true);

    multicore_launch_core1(bus_handler_core);

    absolute_time_t next_frame = get_absolute_time();
    next_frame = delayed_by_us(next_frame, 16666);


    graphics_init();
    graphics_set_buffer((uint8_t *) VIDEORAM, 320, 200);
    graphics_set_mode(TEXTMODE_80x25_BW);
    for (int i = 0; i < 16; i++) {
        graphics_set_palette(i, cga_palette[i]);
    }

    uint32_t frame_counter = 0;
    uint8_t old_videomode = 0;
    while (true) {
        // Обработка DMA
        for (dma_channel_s *channel = dma_channels; channel < dma_channels + DMA_CHANNELS; channel++) {
            if (channel->dreq && !channel->masked) {
                // Вычисляем физический адрес назначения
                const uint32_t dest_addr = channel->page + channel->address;
                const size_t size = (uint32_t) channel->count + 1;

                size_t br;
                switch (channel->data_source_type) {
                    case DMA_SOURCE_MEM_READ: memcpy(&RAM[dest_addr], channel->data_source + channel->data_offset, size);
                        break;
                    case DMA_SOURCE_MEM_WRITE: memcpy((void *) (channel->data_source + channel->data_offset), &RAM[dest_addr], size);
                        break;
                    case DMA_SOURCE_FILE_READ:
                        f_lseek(&file, channel->data_offset);
                        f_read(&file, &RAM[dest_addr], size, &br);
                        // printf("Read %x size %x offset %x \n", br, size, channel->data_offset);
                        break;
                    case DMA_SOURCE_FILE_WRITE:
                        f_lseek(&file, channel->data_offset);
                        f_write(&file, &RAM[dest_addr], size, &br);
                        break;
                }

                // Обновляем счётчики
                update_count(channel, size);

                // Генерируем IRQ если назначен (после завершения передачи!)
                if (channel->finished) {
                    channel->dreq = 0;

                    if (channel->irq)
                        i8259_interrupt(channel->irq);
                    // printf("DMA CH%i transfer compete from %x to %x size %x, irq %d\n", dma_channels-channel, channel->data_source, dest_addr, size, channel->irq);
                }
            }
        }

        // Проверка состояния видеоадаптера и обработка клавиатуры
        if (absolute_time_diff_us(next_frame, get_absolute_time()) >= 0) {
            keyboard_tick();

            next_frame = delayed_by_us(next_frame, 16666);
            mc6845.cursor_blink_state = frame_counter++ >> 4 & 1;

            debug_console(videomode);
        }

        if (cga.updated) {
                if (unlikely(cga.port3D8 & 0b10)) {
                    // Bit 1: Graphics/Text Select
                    if (unlikely(cga.port3D8 & 0b10000)) {
                        {
                            videomode = CGA_640x200x2;
                            graphics_set_bgcolor(0);
                            graphics_set_palette(0, 0);
                            graphics_set_palette(1, cga_palette[cga.port3D9 & 0b1111]);
                        }
                    } else {
                        videomode = CGA_320x200x4;
                        // If colorburst set -- 3rd palette, else from palette register
                        const uint8_t palette = (cga.port3D8 & 4) ? 2 : ((cga.port3D9 >> 5) & 1);
                        const uint8_t intensity = (cga.port3D9 >> 4) & 1;

                        graphics_set_palette(0, cga_palette[cga.port3D9 & 0xF]);
                        graphics_set_bgcolor(cga.port3D9 & 0xF);
                        for (int i = 1; i < 4; i++) {
                            graphics_set_palette(i, cga_palette[cga_gfxpal[palette][intensity][i]]);
                        }
                    }
                } else {
                    videomode = cga.port3D8 & 1 ? TEXTMODE_80x25_COLOR : TEXTMODE_40x25_COLOR;
                    graphics_set_bgcolor(cga.port3D9 & 0xF);
                    for (int i = 0; i < 16; i++) {
                        graphics_set_palette(i, cga_palette[i]);
                    }
                }

                if (unlikely(cga.port3DA_tandy /* == 0x20 */)) {
                    // printf("Tandy hack detected: %i\n", videomode);
                    // videomode = (cga.port3D8 & 0b10000) ? TGA_320x200x16 : TGA_160x200x16;
                    videomode = videomode == CGA_640x200x2 ? TGA_320x200x16 : TGA_160x200x16;
                    graphics_set_bgcolor(cga.port3D9 & 0xF);
                    for (int i = 0; i < 16; i++) {
                        graphics_set_palette(i, cga_palette[i]);
                    }

                }

                if (videomode != old_videomode) {
                    printf("Videomode %i\n", videomode);
                    graphics_set_mode(videomode);
                    old_videomode = videomode;
                }


                // printf("Port 3D8 %x, port 3D9 %x\n", cga.port3D8, cga.port3D9);
                cga.updated = false;
            }
        //__wfi();
        tight_loop_contents();
    }
}
