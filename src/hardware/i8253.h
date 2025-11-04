#pragma once
#include "common.h"
#define PIT_MODE_LATCHCOUNT  0
#define PIT_MODE_LOBYTE 1
#define PIT_MODE_HIBYTE 2
#define PIT_MODE_TOGGLE 3

extern i8253_s i8253;


extern bool speakerenabled;
extern uint32_t timer_interval;

__force_inline static uint8_t i8253_read(const uint16_t port_number) {
    const uint8_t channel = port_number & 3;
    if (channel <= 2) {
        // Если есть защелкнутое значение (LATCHCOUNT), читаем его
        if (i8253.channel_latch_mode[channel]) {
            const uint8_t latch_mode = i8253.channel_latch_mode[channel];
            const uint8_t byte_toggle = i8253.channel_byte_toggle[channel];

            // Определяем какой байт читать
            const bool read_low_byte = (latch_mode == PIT_MODE_LOBYTE) ||
                                       (latch_mode == PIT_MODE_TOGGLE && byte_toggle == 0);

            uint8_t result;
            if (read_low_byte) {
                result = (uint8_t)i8253.channel_latched_value[channel];
            } else {
                result = (uint8_t)(i8253.channel_latched_value[channel] >> 8);
            }

            // Обновляем toggle только для TOGGLE режима
            if (latch_mode == PIT_MODE_TOGGLE) {
                i8253.channel_byte_toggle[channel] = ~byte_toggle & 1;
                // Если прочитали оба байта, сбрасываем latch
                if (byte_toggle == 1) {
                    i8253.channel_latch_mode[channel] = 0;
                }
            } else {
                // Для LOBYTE/HIBYTE сбрасываем latch после одного чтения
                i8253.channel_latch_mode[channel] = 0;
            }

            return result;
        }

        // Обычное чтение (без latch) - читаем текущий счетчик
        const uint8_t access_mode = i8253.channel_access_mode[channel];
        const uint8_t byte_toggle = i8253.channel_byte_toggle[channel];

        const bool read_low_byte = (access_mode == PIT_MODE_LOBYTE) ||
                                   (access_mode == PIT_MODE_TOGGLE && byte_toggle == 0);

        // Декрементируем счетчик (эмуляция тиков таймера)
        // В реальном 8253 счетчик декрементируется на каждом тике 1.193 MHz
        // Для упрощения декрементируем на небольшое значение при каждом чтении
        if (i8253.channel_active[channel]) {
            if (i8253.channel_current_count[channel] < 50) {
                // Reload counter
                i8253.channel_current_count[channel] = i8253.channel_reload_value[channel];
            } else {
                i8253.channel_current_count[channel] -= 50;
            }
        }

        uint8_t result;
        if (read_low_byte) {
            result = (uint8_t)i8253.channel_current_count[channel];
        } else {
            result = (uint8_t)(i8253.channel_current_count[channel] >> 8);
        }

        // Обновляем toggle для TOGGLE режима
        if (access_mode == PIT_MODE_TOGGLE) {
            i8253.channel_byte_toggle[channel] = ~byte_toggle & 1;
        }

        return result;
    }

    return 0xFF;
}

__force_inline static void i8253_write(const uint16_t port_number, const uint8_t data) {
    const uint8_t channel = port_number & 3;
    if (channel <= 2) {
        // channel data
        const uint8_t access_mode = i8253.channel_access_mode[channel];
        const uint8_t byte_toggle = i8253.channel_byte_toggle[channel];

        // Determine which byte to write (low or high)
        const uint8_t current_byte_selector =
                (access_mode == PIT_MODE_LOBYTE || (access_mode == PIT_MODE_TOGGLE && byte_toggle == 0)) ? 0 : 1;

        // Update reload value
        if (current_byte_selector == 0) {
            i8253.channel_reload_value[channel] = (i8253.channel_reload_value[channel] & 0xFF00) | data;
        } else {
            i8253.channel_reload_value[channel] = (i8253.channel_reload_value[channel] & 0x00FF) | (uint16_t) data << 8;
        }

        const uint16_t reload_value = i8253.channel_reload_value[channel];

        if (reload_value == 0) {
            i8253.channel_effective_count[channel] = 65536;
            // speakerenabled = 0;
        } else {
            i8253.channel_effective_count[channel] = reload_value;

            // FIXME!!!
            //speakerenabled = (port61 & 2) ? 1 : 0;
        }

        i8253.channel_active[channel] = 1;

        if (access_mode == PIT_MODE_TOGGLE) {
            i8253.channel_byte_toggle[channel] = ~byte_toggle & 1;
            // Инициализируем счетчик только после записи обоих байт
            if (byte_toggle == 1) {
                i8253.channel_current_count[channel] = i8253.channel_reload_value[channel];
            }
        } else {
            // Для LOBYTE/HIBYTE инициализируем сразу
            i8253.channel_current_count[channel] = i8253.channel_reload_value[channel];
        }

        // Calculate frequency
        i8253.channel_frequency[channel] = 1193182 / i8253.channel_effective_count[channel];

        // Update timer period for channel 0
        if (channel == 0) {
            timer_interval = 1000000 / i8253.channel_frequency[channel];
        }
    } else {
        // portnum == 3: mode/command (control word)
        const uint8_t channel = data >> 6;
        const uint8_t access_mode = (data >> 4) & 3;

        // LATCHCOUNT команда (access_mode == 0)
        if (access_mode == PIT_MODE_LATCHCOUNT) {
            // Защелкиваем текущий счетчик
            i8253.channel_latched_value[channel] = i8253.channel_current_count[channel];
            // Используем текущий access_mode для определения latch_mode
            i8253.channel_latch_mode[channel] = i8253.channel_access_mode[channel];
            // Сбрасываем toggle для корректного чтения
            i8253.channel_byte_toggle[channel] = 0;
        } else {
            // Установка нового access mode
            i8253.channel_access_mode[channel] = access_mode;

            if (access_mode == PIT_MODE_TOGGLE) {
                i8253.channel_byte_toggle[channel] = 0;
            }
        }
    }
}