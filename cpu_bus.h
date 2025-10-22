#pragma once
#include <stdint.h>

// Bus transaction structure
typedef struct {
    uint32_t address; // 20-bit address
    uint8_t m_io; // 1=memory, 0=I/O
    uint8_t bhe; // Bus High Enable
    uint8_t rd; // Read signal
    uint8_t wr; // Write signal
} bus_info_t;

void cpu_bus_init();
