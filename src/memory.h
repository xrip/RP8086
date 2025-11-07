#pragma once
#include "common.h"
#include "rom/bios.h"
#include "rom/basic.h"
#include <stdio.h>

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

    // Video RAM: MDA 0xB0000-0xB0FFF (4KB)
    if ((address - 0xB0000) < 0x7FFF) {
        return *(uint16_t *)&VIDEORAM[address & 0x3FFF];
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

    // Video RAM: MDA 0xB0000-0xB0FFF (4KB)
    if ((address - 0xB8000) < 0x7FFF) {
        write_to(VIDEORAM, address & 0x3FFF, data, bhe);
        return;
    }

    // ROM areas are read-only, ignore writes
}
