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

__force_inline static uint16_t i8253_get_current_count( i8253_channel_s *ch) {
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
    i8253_channel_s *channel = &i8253.channels[port_number & 3];

    const uint8_t latch = channel->latch_mode;
    const uint8_t access_mode = latch ? latch : channel->access_mode;
    const uint16_t value = latch ? channel->latched_value : i8253_get_current_count(channel);
    const uint8_t result = (channel->byte_toggle == 0 || access_mode == PIT_MODE_LOBYTE) ? (uint8_t) value : (uint8_t) (value >> 8);

    if (access_mode == PIT_MODE_TOGGLE) {
        if (latch && channel->byte_toggle) channel->latch_mode = 0;
        channel->byte_toggle ^= 1;
    } else if (latch) {
        channel->latch_mode = 0;
    }

    return result;
}

__force_inline static void i8253_write(const uint16_t port_number, const uint8_t data) {
    if (port_number <= 0x42) {
        const uint8_t channel_index = port_number & 3;
        // Запись данных в канал (порты 0x40-0x42)
        i8253_channel_s *channel = &i8253.channels[channel_index];

        if (channel->access_mode == PIT_MODE_LOBYTE) {
            channel->reload_value = (channel->reload_value & 0xFF00) | data;
        } else if (channel->access_mode == PIT_MODE_HIBYTE) {
            channel->reload_value = (channel->reload_value & 0x00FF) | ((uint16_t) data << 8);
        } else {
            // PIT_MODE_TOGGLE
            if (!(channel->byte_toggle ^= 1)) {
                channel->reload_value = (channel->reload_value & 0xFF00) | data;
                return;
            }
            channel->reload_value = (channel->reload_value & 0x00FF) | ((uint16_t) data << 8);
        }

        channel->active = true;

        // Канал 0 управляет системным таймером
        if (!channel_index) {
            const uint32_t reload_value = channel->reload_value ? : 65536;
            timer_interval = (uint32_t) (((uint64_t) reload_value * 1000000ULL) / PIT_FREQUENCY);
        }
    } else {
        // Запись Control Word (порт 0x43)
        const uint8_t access_mode = (data >> 4) & 3;
        i8253_channel_s *channel = &i8253.channels[data >> 6];

        if (access_mode == PIT_MODE_LATCHCOUNT) {
            channel->latched_value = i8253_get_current_count(channel);
            channel->latch_mode = channel->access_mode;
            channel->byte_toggle = 0;
        } else {
            channel->access_mode = access_mode;
            channel->reload_value = 0;
            channel->active = false;
            channel->latch_mode = 0;
            channel->byte_toggle = 0;
            channel->start_timestamp_us = 0;
        }
    }
}
