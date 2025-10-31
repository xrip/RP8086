#pragma once
#include <stdio.h>

#include "common.h"
#include "hardware/i8237.h"
#include "hardware/i8253.h"
#include "hardware/i8259.h"
#include "hardware/i8272.h"
// ============================================================================
// External Port Arrays and Variables
// ============================================================================
extern volatile uint8_t current_scancode; // Keyboard scancode (defined in main.c)

// ============================================================================
// Port State Variables
// ============================================================================
static uint8_t port3DA = 0; // VGA status port state
static uint8_t port61 = 0;  // System Control Port B (8255 PPI Port B)

// Keyboard controller (8042) state
static uint8_t keyboard_command_byte = 0x45; // Default: interrupts enabled, system flag set
static uint8_t keyboard_last_command = 0;
static uint8_t keyboard_response_buffer = 0;
static bool keyboard_has_response = false;

__force_inline static uint8_t port_read8(const uint32_t address) {
    switch (address) {
        case 0 ... 0x0F: {
            return i8237_readport(address);
        }
        case 0x3F0 ... 0x3F7: {
            return i8272_readport(address);
        }
        case 0x3BA: {
            // MDA status port
            return port3DA ^= 9;
        }
        case 0x20 ... 0x21: {
            return i8259_read(address);
        }
        case 0x40 ... 0x43: {
            return i8253_read(address);
        }
        case 0x60: {
            // Keyboard Data Port - читаем ответ от контроллера или скан-код
            if (keyboard_has_response) {
                keyboard_has_response = false;
                return keyboard_response_buffer;
            }
            // Иначе читаем скан-код и сбрасываем
            const uint8_t scancode = current_scancode;
            current_scancode = 0;
            return scancode;
        }
        case 0x61: {
            // System Control Port B (8255 PPI Port B)
            // Bit 0: Timer 2 gate to speaker
            // Bit 1: Speaker data enable
            // Bit 4: RAM parity check (1=no error)
            // Bit 5: I/O channel check (1=no error)
            // Возвращаем сохраненное значение с установленными битами 4-5 (нет ошибок)
            return (port61 & 0x0F) | 0x30;
        }
        case 0x64: {
            // Keyboard Controller Status Register (Intel 8042)
            // Bit 0: Output buffer full (1 = данные доступны в 0x60)
            // Bit 1: Input buffer full (0 = готов принять команду)
            // Bit 2: System flag (1 = POST passed) - КРИТИЧНО для DOS!
            // Bit 3: Command/Data (0 = data, 1 = command)
            // Bit 4: Keyboard enabled
            // Bit 5-7: таймауты и ошибки (0)

            uint8_t status = 0x14; // Биты 2 (System) и 4 (Keyboard enabled) установлены

            // Bit 0: есть данные для чтения
            if (current_scancode != 0 || keyboard_has_response) {
                status |= 0x01;
            }

            return status;
        }
        case 0x81:
        case 0x82:
        case 0x83:
        case 0x87: {
            return i8237_readpage(address);
        }

        default:
            return 0xFF;
    }
}

// ============================================================================
// Port Read (16-bit)
// ============================================================================
__force_inline static uint16_t port_read(const uint32_t address, const bool bhe) {
    // Оптимизация: проверяем A0 и BHE для выбора 8/16-битного пути
    const bool a0 = address & 1;

    // BHE=0, A0=0 -> 16-битная операция word (оба байта)
    if (likely(!bhe && !a0)) {
        return port_read8(address) | (port_read8(address + 1) << 8);
    }

    // BHE=1, A0=0 -> 8-битная операция low byte (старший байт выключен)
    if (unlikely(bhe && !a0)) {
        return port_read8(address);
    }

    // BHE=0, A0=1 -> 8-битная операция high byte (нечетный адрес)
    if (unlikely(!bhe && a0)) {
        return port_read8(address) << 8;
    }

    // BHE=1, A0=1 -> невалидная комбинация (не используется в i8086)
    return 0xFFFF;
}

__force_inline static void port_write8(const uint32_t address, const uint8_t data, const bool bhe) {
    switch (address) {
        case 0 ... 0x0F: {
            return i8237_writeport(address, data);
        }
        case 0x3F0 ... 0x3F7: {
            return i8272_writeport(address, data);
        }
        case 0x20 ... 0x21: {
            return i8259_write(address, data);
        }
        case 0x40 ... 0x43: {
            return i8253_write(address, data);
        }
        case 0x60: {
            // Keyboard Data Port - запись данных (response to command)
            // Обработка зависит от последней команды
            if (keyboard_last_command == 0x60) {
                // Write command byte
                keyboard_command_byte = data;
                keyboard_last_command = 0;
            }
            return;
        }
        case 0x61: {
            // System Control Port B (8255 PPI Port B)
            // Сохраняем младшие биты (speaker, timer gate, keyboard control)
            port61 = data;
            return;
        }
        case 0x64: {
            // Keyboard Controller Command Register (Intel 8042)
            keyboard_last_command = data;

            switch (data) {
                case 0x20: // Read command byte
                    keyboard_response_buffer = keyboard_command_byte;
                    keyboard_has_response = true;
                    break;
                case 0x60: // Write command byte (next byte to 0x60)
                    // Ожидаем данные в порт 0x60
                    break;
                case 0xAA: // Self test
                    keyboard_response_buffer = 0x55; // Self test passed
                    keyboard_has_response = true;
                    break;
                case 0xAD: // Disable keyboard
                    keyboard_command_byte &= ~0x10; // Clear bit 4
                    break;
                case 0xAE: // Enable keyboard
                    keyboard_command_byte |= 0x10; // Set bit 4
                    break;
                default:
                    // Неизвестная команда - игнорируем
                    break;
            }
            return;
        }
        case 0x81:
        case 0x82:
        case 0x83:
        case 0x87: {
            return i8237_writepage(address, data);
        }
    }
}

// ============================================================================
// Port Write (16-bit with BHE support)
// ============================================================================
__force_inline static void port_write(const uint32_t address, const uint16_t data, const bool bhe) {
    // Оптимизация: проверяем A0 и BHE для выбора 8/16-битного пути
    const bool a0 = address & 1;

    // BHE=0, A0=0 -> 16-битная операция word (оба байта)
    if (likely(!bhe && !a0)) {
        port_write8(address, data, bhe);
        port_write8(address + 1, data >> 8, bhe);
        return;
    }

    // BHE=1, A0=0 -> 8-битная операция low byte (старший байт выключен)
    if (unlikely(bhe && !a0)) {
        port_write8(address, data, bhe);
        return;
    }

    // BHE=0, A0=1 -> 8-битная операция high byte (нечетный адрес)
    if (unlikely(!bhe && a0)) {
        port_write8(address, data >> 8, bhe);
    }

    // BHE=1, A0=1 -> невалидная комбинация (игнорируем)
}
