/* 	Intel 8237 DMA controller */
#pragma once

//#define DEBUG_I8237
#if defined(DEBUG_I8237)
#define debug_log(...) printf(__VA_ARGS__)
#else
#define debug_log(...)
#endif

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
                // Count register (порты 0x01, 0x03, 0x05, 0x07)
                if (byte_pointer_flipflop) {
                    // HIGH byte: сохраняем LOW, записываем HIGH
                    dma_channels[channel].count = (dma_channels[channel].count & 0x00FF) | ((uint16_t) data << 8);
                } else {
                    // LOW byte: обнуляем HIGH, записываем только LOW
                    dma_channels[channel].count = (uint16_t) data;
                }
                dma_channels[channel].reload_count = dma_channels[channel].count;
                if (channel == 2) {
                    debug_log("[%llu] DMA COUNT: CH2 port=0x%02X, flipflop=%d, data=0x%02X, count=0x%04X\n",
                           to_us_since_boot(get_absolute_time()), port_number, byte_pointer_flipflop, data, dma_channels[channel].count);
                }
            } else {
                // Программируем ТОЛЬКО reload_address (base register)
                if (byte_pointer_flipflop) {
                    // HIGH byte (второй вызов): сохраняем LOW byte, записываем HIGH
                    dma_channels[channel].reload_address = (dma_channels[channel].reload_address & 0x00FF) | ((uint16_t) data << 8);

                    // CRITICAL FIX: Если канал активен (не masked), обновляем current address тоже!
                    if (!dma_channels[channel].masked) {
                        dma_channels[channel].address = dma_channels[channel].reload_address;
                    }
                    if (channel == 2) {
                        debug_log("[%llu] DMA ADDR HIGH: CH2 port=0x%02X, data=0x%02X, reload_address=0x%04X\n",
                               to_us_since_boot(get_absolute_time()), port_number, data, dma_channels[channel].reload_address);
                    }
                } else {
                    // LOW byte (первый вызов): обнуляем HIGH byte, записываем только LOW
                    dma_channels[channel].reload_address = (uint16_t) data;
                    if (channel == 2) {
                        debug_log("[%llu] DMA ADDR LOW: CH2 port=0x%02X, data=0x%02X, reload_address=0x%04X\n",
                               to_us_since_boot(get_absolute_time()), port_number, data, dma_channels[channel].reload_address);
                    }
                }
            }


            byte_pointer_flipflop ^= 1;
            break;
        }
        case DMA_COMMAND_REGISTER: //DMA channel 0-3 command register
            memory_to_memory_enabled = data & 1;
            // Bit 5: Auto-initialize enable (affects ALL channels!)
            for (int i = 0; i < DMA_CHANNELS; i++) {
                dma_channels[i].auto_init = (data >> 5) & 1;
            }
            debug_log("DMA COMMAND: mem2mem=%d, auto_init=%d\n", memory_to_memory_enabled, (data >> 5) & 1);
            break;
        case DMA_REQUEST_REGISTER: //DMA request register
            dma_channels[data & 3].dreq = (data >> 2) & 1;
            break;
        case DMA_CHANNEL_MASK_REGISTER: //DMA channel 0-3 mask register
            const uint8_t ch = data & 3;
            const bool new_mask = (data >> 2) & 1;

            // При unmask (0→1 transition) копируем reload → current
            if (dma_channels[ch].masked && !new_mask) {
                dma_channels[ch].address = dma_channels[ch].reload_address;
                dma_channels[ch].count = dma_channels[ch].reload_count;
                if (ch == 2) {
                    debug_log("[%llu] DMA UNMASK: CH2 address=0x%04X, count=0x%04X (reload→current)\n",
                           to_us_since_boot(get_absolute_time()), dma_channels[ch].address, dma_channels[ch].count);
                }
            }

            dma_channels[ch].masked = new_mask;
            break;
        case DMA_MODE_REGISTER: //DMA channel 0-3 mode register
            const uint8_t channel = data & 3;
            dma_channels[channel].transfer_type = (data >> 2) & 3;
            dma_channels[channel].auto_init = (data >> 4) & 1;
            dma_channels[channel].address_increase = (data & 0x20) ? 0xFFFFFFFF : 0x1; // bit 5
            dma_channels[channel].mode = (data >> 6) & 3;
            debug_log("DMA MODE: CH%d raw=0x%02X, auto_init=%d, transfer_type=%d (0=verify,1=write,2=read), mode=%d\n",
                   channel, data, dma_channels[channel].auto_init, dma_channels[channel].transfer_type, dma_channels[channel].mode);
            break;
        case DMA_STATUS_REGISTER: // DMA master clear
            i8237_reset();
            break;
        case DMA_TEMP_REGISTER: // Mask Reset
            byte_pointer_flipflop = 0;
            break;
        case DMA_CLEAR_FF: //clear byte pointer flipflop
            byte_pointer_flipflop = 0;
            debug_log("DMA CLEAR_FF: byte_pointer_flipflop reset to 0\n");
            break;
        case DMA_MASK_REGISTER: //DMA write mask register
            for (int i = 0; i < 4; i++) {
                const bool new_mask = (data >> i) & 1;

                // При unmask копируем reload → current
                if (dma_channels[i].masked && !new_mask) {
                    dma_channels[i].address = dma_channels[i].reload_address;
                    dma_channels[i].count = dma_channels[i].reload_count;
                    if (i == 2) {
                        debug_log("DMA UNMASK (0x0F): CH2 address=0x%04X, count=0x%04X (reload→current)\n",
                               dma_channels[i].address, dma_channels[i].count);
                    }
                }

                dma_channels[i].masked = new_mask;
            }
            break;
    }
}

__force_inline static void i8237_writepage(const uint16_t port_number, const uint8_t data) {
    uint8_t channel;
    switch (port_number) {
        case 0x81:
            channel = 2;  // Channel 2 page register
            break;
        case 0x82:
            channel = 3;  // Channel 3 page register
            break;
        case 0x83:
            channel = 0;  // Channel 0 page register
            break;
        case 0x87:
            channel = 1;  // Channel 1 page register
            break;
        default:
            return;
    }
    dma_channels[channel].page = (uint32_t) data << 16;
    if (channel == 2) {
        debug_log("[%llu] DMA PAGE WRITE: port=0x%02X, channel=%d, page=0x%02X, final_addr=0x%05X\n",
               to_us_since_boot(get_absolute_time()), port_number, channel, data, dma_channels[channel].page | dma_channels[channel].reload_address);
    }
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
                debug_log("[%llu] DMA READ COUNT: CH%d port=0x%02X, flipflop=%d, value=0x%02X, full_count=0x%04X\n",
                       to_us_since_boot(get_absolute_time()), channel, port_number, byte_pointer_flipflop, register_value, dma_channels[channel].count);
            } else {
                //address
                if (byte_pointer_flipflop) {
                    register_value = (uint8_t) (dma_channels[channel].address >> 8);
                } else {
                    register_value = (uint8_t) dma_channels[channel].address;
                }
                debug_log("[%llu] DMA READ ADDRESS: CH%d port=0x%02X, flipflop=%d, value=0x%02X, full_addr=0x%04X\n",
                       to_us_since_boot(get_absolute_time()), channel, port_number, byte_pointer_flipflop, register_value, dma_channels[channel].address);
            }
            byte_pointer_flipflop ^= 1;
            break;
        }
        case 0x08: //status register
            // Bits 0-3: DMA request status for channels 0-3
            // Bits 4-7: Terminal Count reached for channels 0-3
            register_value = 0;
            for (int i = 0; i < 4; i++) {
                if (dma_channels[i].dreq) {
                    register_value |= (1 << i);  // DREQ status
                }
                if (dma_channels[i].finished) {
                    register_value |= (1 << (i + 4));  // TC status
                    dma_channels[i].finished = 0;  // Clear TC flag on read (per Intel 8237A spec)
                }
            }
            debug_log("[%llu] DMA STATUS READ: 0x%02X (DREQ=0x%01X, TC=0x%01X)\n",
                   to_us_since_boot(get_absolute_time()), register_value, register_value & 0x0F, (register_value >> 4) & 0x0F);
    }
    return register_value;
}

__force_inline static uint8_t i8237_readpage(const uint16_t port_number) {
    uint8_t channel;
    switch (port_number) {
        case 0x81:
            channel = 2;  // Channel 2 page register
            break;
        case 0x82:
            channel = 3;  // Channel 3 page register
            break;
        case 0x83:
            channel = 0;  // Channel 0 page register
            break;
        case 0x87:
            channel = 1;  // Channel 1 page register
            break;
        default:
            return 0xFF;
    }
    return (uint8_t) (dma_channels[channel].page >> 16);
}

__force_inline void update_count(const uint8_t channel, const uint16_t count) {
    // Сохраняем старое значение для проверки underflow (terminal count)
    const uint16_t old_count = dma_channels[channel].count;

    // Обновляем адрес (16-битная арифметика с оборачиванием!) и count
    dma_channels[channel].address = (dma_channels[channel].address + dma_channels[channel].address_increase * count) & 0xFFFF;
    dma_channels[channel].count -= count;

    // Terminal count: произошел underflow (count обернулся через 0x0000 → 0xFFFF)
    // Проверка: если новое значение больше старого в беззнаковой арифметике
    if (dma_channels[channel].count > old_count) {
        // КРИТИЧНО: Устанавливаем флаг Terminal Count для Status Register (bit 4-7)
        dma_channels[channel].finished = 1;

        if (dma_channels[channel].auto_init) {
            // Auto-init: перезагрузка из reload registers
            dma_channels[channel].count = dma_channels[channel].reload_count;
            dma_channels[channel].address = dma_channels[channel].reload_address;
            if (channel == 2) {
                debug_log("[%llu] DMA AUTO-INIT: CH2 address=0x%04X, count=0x%04X (reload→current), TC flag set\n",
                       to_us_since_boot(get_absolute_time()), dma_channels[channel].address, dma_channels[channel].count);
            }
        } else {
            // Terminal count без auto-init: маскируем канал
            dma_channels[channel].masked = 1;
            if (channel == 2) {
                debug_log("[%llu] DMA TERMINAL COUNT: CH2 masked, TC flag set\n", to_us_since_boot(get_absolute_time()));
            }
        }
    }
}

// Stub function to read from 8086 ram/rom
__force_inline static uint8_t i8237_read(const uint8_t channel) {
    if (dma_channels[channel].masked) return 0;

    const uint32_t memory_address = dma_channels[channel].page + dma_channels[channel].address;
    debug_log("DMA read from 0x%08X\n", memory_address);
    //const uint8_t read_data = memory_read(memory_address);
    constexpr uint8_t read_data = 0;
    update_count(channel, 1);

    return read_data;
}

// DMA write to 8086 memory
// Возвращает количество ФАКТИЧЕСКИ записанных байт (может быть меньше size, если достигнут Terminal Count)
__force_inline static uint32_t i8237_write(const uint8_t channel, const uint8_t * src, size_t size) {
    const uint32_t page = dma_channels[channel].page;
    const uint32_t base_address = page + dma_channels[channel].address;

    debug_log("[%llu] DMA WRITE START: CH%d, src=0x%08X, dst=0x%05X, size=%d, DMA_count=%d\n",
           to_us_since_boot(get_absolute_time()), channel, (uint32_t)src, base_address, size, dma_channels[channel].count);

    extern uint8_t RAM[];

    // КРИТИЧЕСКИ ВАЖНО: Intel 8237A останавливает передачу при достижении Terminal Count!
    // Terminal Count происходит когда count переходит с 0 на 0xFFFF (underflow)
    // Мы должны остановиться, если запрошенный размер превышает оставшийся count

    // Сколько байт осталось до Terminal Count (count+1, потому что 0 означает "еще 1 байт")
    const uint32_t bytes_until_tc = (uint32_t)dma_channels[channel].count + 1;
    const uint32_t actual_size = (size > bytes_until_tc) ? bytes_until_tc : size;

    if (actual_size < size) {
        debug_log("[%llu] DMA: LIMITING transfer from %zu to %u bytes (TC would occur)\n",
               to_us_since_boot(get_absolute_time()), size, actual_size);
    }

    // Intel 8237A DMA wrapping: адрес оборачивается только внутри 64KB сегмента
    // (только младшие 16 бит инкрементируются, page register остается неизменным)
    size_t bytes_written = 0;
    while (bytes_written < actual_size) {
        // Текущий 16-битный offset в сегменте (с оборачиванием)
        const uint16_t current_offset = (dma_channels[channel].address + bytes_written) & 0xFFFF;
        const uint32_t physical_address = page + current_offset;

        // Сколько байт до конца 64KB сегмента
        const size_t bytes_to_segment_end = 0x10000 - current_offset;
        const size_t bytes_to_write = (actual_size - bytes_written < bytes_to_segment_end)
                                      ? (actual_size - bytes_written)
                                      : bytes_to_segment_end;

        // Проверка границ RAM для КАЖДОЙ части записи (до и после оборачивания)
        if (physical_address < RAM_SIZE) {
            // Адрес внутри RAM - определяем сколько можем записать
            const size_t safe_size = (physical_address + bytes_to_write > RAM_SIZE)
                                     ? (RAM_SIZE - physical_address)
                                     : bytes_to_write;

            if (safe_size > 0) {
                memcpy(RAM + physical_address, src + bytes_written, safe_size);
            }

            if (safe_size < bytes_to_write) {
                debug_log("[%llu] DMA: write at 0x%05X truncated (%zu/%zu bytes) - beyond RAM (size=0x%05X)\n",
                       to_us_since_boot(get_absolute_time()), physical_address, safe_size, bytes_to_write, RAM_SIZE);
            }
        } else {
            // Адрес полностью за пределами RAM - ничего не пишем
            debug_log("[%llu] DMA: write at 0x%05X (%zu bytes) - beyond RAM (size=0x%05X), discarded\n",
                   to_us_since_boot(get_absolute_time()), physical_address, bytes_to_write, RAM_SIZE);
        }

        bytes_written += bytes_to_write;

        // Если достигли конца сегмента, логируем оборачивание
        if (current_offset + bytes_to_write >= 0x10000) {
            debug_log("[%llu] DMA: 64KB segment wrap - continuing at page 0x%02X, offset 0x0000\n",
                   to_us_since_boot(get_absolute_time()), (page >> 16) & 0xFF);
        }
    }

    update_count(channel, actual_size);

    debug_log("[%llu] DMA WRITE END: CH%d, wrote %zu/%zu bytes, new_count=%d, TC=%d\n",
           to_us_since_boot(get_absolute_time()), channel, bytes_written, size,
           dma_channels[channel].count, dma_channels[channel].finished);

    return bytes_written;
}
#if defined(DEBUG_I8237)
#undef debug_log
#endif