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
        // channel data
        const uint8_t access_mode = i8253.channel_access_mode[channel];
        const uint8_t byte_toggle = i8253.channel_byte_toggle[channel];

        // Determine which byte to read (low or high)
        const uint8_t current_byte_selector =
        access_mode == 0 || access_mode == PIT_MODE_LOBYTE ||
        (access_mode == PIT_MODE_TOGGLE && byte_toggle == 0) ? 0             : 1;

        // Update toggle state for relevant modes
        if (access_mode == 0 || access_mode == PIT_MODE_TOGGLE) {
            i8253.channel_byte_toggle[channel] = ~byte_toggle & 1;
        }

        if (current_byte_selector == 0) {
            // Low byte - update counter if needed
            if (i8253.channel_current_count[channel] < 10) {
                i8253.channel_current_count[channel] = i8253.channel_reload_value[channel];
            }
            i8253.channel_current_count[channel] -= 10;
            return (uint8_t) i8253.channel_current_count[channel];
        } else {
            // High byte
            return (uint8_t) (i8253.channel_current_count[channel] >> 8);
        }
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
            speakerenabled = 0;
        } else {
            i8253.channel_effective_count[channel] = reload_value;

            // FIXME!!!
            //speakerenabled = (port61 & 2) ? 1 : 0;
        }

        i8253.channel_active[channel] = 1;

        if (access_mode == PIT_MODE_TOGGLE) {
            i8253.channel_byte_toggle[channel] = ~byte_toggle & 1;
        }

        // Calculate frequency
        i8253.channel_frequency[channel] = 1193182 / i8253.channel_effective_count[channel];

        // Update timer period for channel 0
        if (channel == 0) {
            timer_interval = 1000000 / i8253.channel_frequency[channel];
        }
    } else {
        // portnum == 3: mode/command
        const uint8_t channel = data >> 6;
        const uint8_t access_mode = (data >> 4) & 3;

        i8253.channel_access_mode[channel] = access_mode;

        if (access_mode == PIT_MODE_TOGGLE) {
            i8253.channel_byte_toggle[channel] = 0;
        }
    }
}