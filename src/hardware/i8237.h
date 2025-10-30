/* 	Intel 8237 DMA controller */
#pragma once

#define DMA_COMMAND_REGISTER 0x08
#define DMA_REQUEST_REGISTER 0x09
#define DMA_CHANNEL_MASK_REGISTER 0x0A
#define DMA_MODE_REGISTER 0x0B
#define DMA_CLEAR_FF 0x0C          // Clear flip-flop port
#define DMA_STATUS_REGISTER 0x0D
#define DMA_TEMP_REGISTER 0x0E
#define DMA_MASK_REGISTER 0x0F

#define DMA_CHANNELS 4
#include "common.h"
extern dma_channel_s dma_channels[DMA_CHANNELS];

static bool byte_pointer_flipflop, memory_to_memory_enabled;

__force_inline static void i8237_reset() {
    memset(dma_channels, 0x00, sizeof(dma_channel_s) * DMA_CHANNELS);
    memory_to_memory_enabled = false;
    dma_channels[0].masked = 1;
    dma_channels[1].masked = 1;
    dma_channels[2].masked = 1;
    dma_channels[3].masked = 1;
}

__force_inline static void i8237_writeport(const uint16_t port_number, const uint8_t data) {
    switch (port_number & 0xF) {
        case 0 ... 7: {
            const uint8_t channel = (port_number >> 1) & 3;

            if (port_number & 0x01) {
                if (byte_pointer_flipflop) {
                    dma_channels[channel].count = (dma_channels[channel].count & 0x00FF) | (uint16_t) data << 8;
                } else {
                    dma_channels[channel].count = (dma_channels[channel].count & 0xFF00) | (uint16_t) data;
                }
                dma_channels[channel].reload_count = dma_channels[channel].count;
            } else {
                if (byte_pointer_flipflop) {
                    dma_channels[channel].address = (dma_channels[channel].address & 0x00FF) | (uint16_t) data << 8;
                } else {
                    dma_channels[channel].address = (dma_channels[channel].address & 0xFF00) | (uint16_t) data;
                }

                dma_channels[channel].reload_address = dma_channels[channel].address;
            }


            byte_pointer_flipflop ^= 1;
            break;
        }
        case DMA_COMMAND_REGISTER: //DMA channel 0-3 command register
            memory_to_memory_enabled = data & 1;
            break;
        case DMA_REQUEST_REGISTER: //DMA request register
            dma_channels[data & 3].dreq = (data >> 2) & 1;
            break;
        case DMA_CHANNEL_MASK_REGISTER: //DMA channel 0-3 mask register
            dma_channels[data & 3].masked = (data >> 2) & 1;
            // printf("channel %i masked %i\n", value & 3, (value >> 2) & 1);
            break;
        case DMA_MODE_REGISTER: //DMA channel 0-3 mode register
            dma_channels[data & 3].transfer_type = (data >> 2) & 3;
            dma_channels[data & 3].auto_init = (data >> 4) & 1;
            dma_channels[data & 3].address_increase = (data & 0x20) ? 0xFFFFFFFF : 0x1; // bit 5
            dma_channels[data & 3].mode = (data >> 6) & 3;
            break;
        case DMA_STATUS_REGISTER: // DMA master clear
            i8237_reset();
        case DMA_TEMP_REGISTER: // Mask Reset
        case DMA_CLEAR_FF: //clear byte pointer flipflop
            byte_pointer_flipflop = 0;
            break;
        case DMA_MASK_REGISTER: //DMA write mask register
            dma_channels[0].masked = (data >> 0) & 1;
            dma_channels[1].masked = (data >> 1) & 1;
            dma_channels[2].masked = (data >> 2) & 1;
            dma_channels[3].masked = (data >> 3) & 1;
            break;
    }
}

__force_inline static void i8237_writepage(const uint16_t port_number, const uint8_t data) {
    uint8_t channel;
    switch (port_number & 0xF) {
        case 0x07:
            channel = 0;
            break;
        case 0x03:
            channel = 1;
            break;
        case 0x01:
            channel = 2;
            break;
        case 0x02:
            channel = 3;
            break;
        default:
            return;
    }
    dma_channels[channel].page = (uint32_t) data << 16;
}

__force_inline static uint8_t i8237_readport(const uint16_t port_number) {
    uint8_t register_value = 0xFF;

    switch (port_number & 0xf) {
        case 0 ... 7: {
            const uint8_t channel = (port_number >> 1) & 3;

            if (port_number & 1) {
                //count
                if (byte_pointer_flipflop) {
                    register_value = (uint8_t) (dma_channels[channel].count >> 8); //TODO: or give back the reload??
                } else {
                    register_value = (uint8_t) dma_channels[channel].count; //TODO: or give back the reload??
                }
            } else {
                //address
                if (byte_pointer_flipflop) {
                    register_value = (uint8_t) (dma_channels[channel].address >> 8);
                } else {
                    register_value = (uint8_t) dma_channels[channel].address;
                }
            }
            byte_pointer_flipflop ^= 1;
            break;
        }
        case 0x08: //status register
            register_value = 0x0F;
    }
    return register_value;
}

__force_inline static uint8_t i8237_readpage(const uint16_t port_number) {
    uint8_t channel;
    switch (port_number & 0xF) {
        case 0x07:
            channel = 0;
            break;
        case 0x03:
            channel = 1;
            break;
        case 0x01:
            channel = 2;
            break;
        case 0x02:
            channel = 3;
            break;
        default:
            return 0xFF;
    }
    return (uint8_t) (dma_channels[channel].page >> 16);
}

__force_inline void update_count(const uint8_t channel) {
    dma_channels[channel].address += dma_channels[channel].address_increase;
    dma_channels[channel].count--;

    if (dma_channels[channel].count == 0xFFFF) {
        if (dma_channels[channel].auto_init) {
            dma_channels[channel].count = dma_channels[channel].reload_count;
            dma_channels[channel].address = dma_channels[channel].reload_address & 0xFFFF;
        } else {
            dma_channels[channel].masked = 1;
        }
    }
}

// Stub function to read from 8086 ram/rom
__force_inline static uint8_t i8237_read(const uint8_t channel) {
    if (dma_channels[channel].masked) return 0;

    const uint32_t memory_address = dma_channels[channel].page + dma_channels[channel].address;
    //const uint8_t read_data = memory_read(memory_address);
    constexpr uint8_t read_data = 0;
    update_count(channel);

    return read_data;
}

// Stub function to write to 8086 ram/rom
__force_inline static void i8237_write(const uint8_t channel, const uint8_t value) {
    const uint32_t memory_address = dma_channels[channel].page + dma_channels[channel].address;
    update_count(channel);
    //memory_write(memory_address, value, 0);
}
