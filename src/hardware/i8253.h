#pragma once
#include "common.h"

#define PIT_MODE_LATCHCOUNT  0
#define PIT_MODE_LOBYTE      1
#define PIT_MODE_HIBYTE      2
#define PIT_MODE_TOGGLE      3
#define PIT_FREQUENCY        1193182

extern i8253_s i8253;
extern bool speakerenabled;
extern uint32_t timer_interval;

__force_inline static uint16_t i8253_get_current_count(uint8_t channel) {
    i8253_channel_s *ch = &i8253.channels[channel];

    if (!ch->active) {
        return ch->reload_value;
    }

    const uint32_t reload = ch->reload_value ? : 65536;
    const uint64_t now = get_absolute_time();
    if (!ch->start_timestamp_us) {
        ch->start_timestamp_us = now;
        return reload - 1;
    }
    const uint64_t ticks = ((now - ch->start_timestamp_us) * PIT_FREQUENCY) / 1000000ULL;
    return (uint16_t) (reload - 1 - ticks % reload);
}

__force_inline static uint8_t i8253_read(const uint16_t port_number) {
    const uint8_t channel = port_number & 3;
    i8253_channel_s *ch = &i8253.channels[channel];

    const uint8_t latch = ch->latch_mode;
    const uint8_t access_mode = latch ? latch : ch->access_mode;
    const uint16_t value = latch ? ch->latched_value : i8253_get_current_count(channel);
    const uint8_t result = (ch->byte_toggle == 0 || access_mode == PIT_MODE_LOBYTE) ? (uint8_t) value : (uint8_t) (value >> 8);

    if (access_mode == PIT_MODE_TOGGLE) {
        if (latch && ch->byte_toggle) ch->latch_mode = 0;
        ch->byte_toggle ^= 1;
    } else if (latch) {
        ch->latch_mode = 0;
    }

    return result;
}

__force_inline static void i8253_write(const uint16_t port_number, const uint8_t data) {
    if (port_number <= 0x42) {
        // Запись данных в канал (порты 0x40-0x42)
        const uint8_t channel = port_number & 3;
        i8253_channel_s *ch = &i8253.channels[channel];

        if (ch->access_mode == PIT_MODE_LOBYTE) {
            ch->reload_value = (ch->reload_value & 0xFF00) | data;
        } else if (ch->access_mode == PIT_MODE_HIBYTE) {
            ch->reload_value = (ch->reload_value & 0x00FF) | ((uint16_t) data << 8);
        } else {
            // PIT_MODE_TOGGLE
            if (!(ch->byte_toggle ^= 1)) {
                ch->reload_value = (ch->reload_value & 0xFF00) | data;
                return;
            }
            ch->reload_value = (ch->reload_value & 0x00FF) | ((uint16_t) data << 8);
        }

        ch->active = true;

        // Канал 0 управляет системным таймером
        if (!channel) {
            const uint32_t reload_value = ch->reload_value ? : 65536;
            timer_interval = (uint32_t) (((uint64_t) reload_value * 1000000ULL) / PIT_FREQUENCY);
        }
    } else {
        // Запись Control Word (порт 0x43)
        const uint8_t channel = data >> 6;
        const uint8_t access_mode = (data >> 4) & 3;
        i8253_channel_s *ch = &i8253.channels[channel];

        if (access_mode == PIT_MODE_LATCHCOUNT) {
            ch->latched_value = i8253_get_current_count(channel);
            ch->latch_mode = ch->access_mode;
            ch->byte_toggle = 0;
        } else {
            ch->access_mode = access_mode;
            ch->reload_value = 0;
            ch->active = false;
            ch->latch_mode = 0;
            ch->byte_toggle = 0;
            ch->start_timestamp_us = 0;
        }
    }
}
