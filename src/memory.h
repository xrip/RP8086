#pragma once
#include "common.h"
#include "bios.h"
#include "basic.h"

// ============================================================================
// External Memory Arrays
// ============================================================================
extern uint8_t RAM[];        // Main RAM (defined in main.c)
extern uint8_t VIDEORAM[];   // Video RAM (defined in main.c)

// ============================================================================
// Memory Read (16-bit)
// ============================================================================
__force_inline static uint16_t memory_read(const uint32_t address) {
    // RAM: 0x00000-0x2FFFF (192KB)
    if (address < RAM_SIZE) {
        return *(uint16_t *)&RAM[address];
    }

    // Video RAM: 0xB0000-0xB7FFF (4KB, MDA text mode)
    // Оптимизация: одна проверка через вычитание вместо двух сравнений
    if ((address - 0xB0000) < 0x8000) {
        return *(uint16_t *)&VIDEORAM[address & 4094];
    }

    // BASIC ROM: 0xF6000-0xFDFFF
    if ((address - 0xF6000) < (BIOS_ROM_BASE - 0xF6000)) {
        return *(uint16_t *)&BASIC[address - 0xF6000];
    }

    // BIOS ROM: 0xFE000-0xFFFFF (8KB)
    if (address >= BIOS_ROM_BASE) {
        return *(uint16_t *)&BIOS[address - BIOS_ROM_BASE];
    }

    // Unmapped memory
    return 0xFFFF;
}

// ============================================================================
// Memory Write (16-bit with BHE support)
// ============================================================================
__force_inline static void memory_write(const uint32_t address, const uint16_t data, const bool bhe) {
    // RAM: 0x00000-0x2FFFF (192KB)
    if (address < RAM_SIZE) {
        write_to(RAM, address, data, bhe);
        return;
    }

    // Video RAM: 0xB0000-0xB7FFF (4KB, MDA text mode)
    // Оптимизация: одна проверка вместо двух для видеопамяти
    if ((address - 0xB0000) < 0x8000) {
        write_to(VIDEORAM, address & 0xFFF, data, bhe);
    }

    // ROM areas are read-only, ignore writes
}
