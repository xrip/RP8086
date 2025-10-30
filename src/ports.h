#pragma once
#include <stdio.h>

#include "common.h"
#include "hardware/i8253.h"
#include "hardware/i8259.h"
// ============================================================================
// External Port Arrays and Variables
// ============================================================================
extern volatile uint8_t current_scancode; // Keyboard scancode (defined in main.c)

// ============================================================================
// Port State Variables
// ============================================================================
static uint8_t port3DA = 0; // VGA status port state

__force_inline static uint8_t port_read8(const uint32_t address) {
    switch (address) {
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
            // Keyboard Data Port - читаем скан-код и сбрасываем
            const uint8_t scancode = current_scancode;
            current_scancode = 0;
            return scancode;
        }
        case 0x64: {
            // Keyboard Status Port - bit 0 = данные доступны
            return current_scancode != 0; // Упрощено: компилятор генерирует оптимальный код
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
        case 0x20 ... 0x21: {
            return i8259_write(address, data);
        }
        case 0x40 ... 0x43: {
            return i8253_write(address, data);
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
