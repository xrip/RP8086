#include "i8086_bus.pio.h"
#include "cpu_bus.h"

#include <stdio.h>

#include "config.h"
#include "bios.h"

// ROM configuration
#define BIOS_ROM_SIZE          sizeof(GLABIOS_0_4_1_8T_ROM) // 8K BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)   // 0xFE000-0xFFFFF
#define BIOS_ROM_MASK          (BIOS_ROM_SIZE - 1)          // 0x1FFF
#define BIOS_ROM_MASK_16BIT    (BIOS_ROM_SIZE - 2)          // 0x1FFE

// RAM configuration (RP2040 has 264KB SRAM, use 224KB for i8086)
#define RAM_SIZE         (224 * 1024)                        // 224KB
#define RAM_MASK         (RAM_SIZE - 1)                     // 0xFFFF
#define RAM_MASK_16BIT   (RAM_SIZE - 2)                     // 0xFFFE

static uint8_t ram[RAM_SIZE] __attribute__((aligned(4)));

// Temporary test variables
static uint16_t prevFibNo = 1;
static uint16_t currFibNo = 1;

// Extract bus transaction info from PIO FIFO data
static inline bus_info_t parse_bus_state(const uint32_t bus_data) {
    bus_info_t info;

    info.address = bus_data & 0xFFFFF;       // Bits [19:0]
    info.rd      = (bus_data >> 21) & 1;     // Bit [21]
    info.wr      = (bus_data >> 22) & 1;     // Bit [22]
    info.m_io    = (bus_data >> 23) & 1;     // Bit [23]
    info.bhe     = (bus_data >> 24) & 1;     // Bit [24]

    return info;
}

uint16_t __time_critical_func(i8086_read)(const uint32_t address, const bool is_memory_access) {
    if (is_memory_access) {
        // Memory access
        if (address < RAM_SIZE) {
            // RAM area: 0x00000-0x0FFFF (64K)
            const uint32_t aligned_offset = address & RAM_MASK_16BIT;
            return *(uint16_t *) &ram[aligned_offset];
        } else if (address >= BIOS_ROM_BASE && address < 0x100000) {
            // ROM area: 0xFE000-0xFFFFF (8K)
            const uint32_t aligned_offset = address & BIOS_ROM_MASK_16BIT;
            return *(uint16_t *) &GLABIOS_0_4_1_8T_ROM[aligned_offset];
        }
        // Unmapped memory
        return 0xFFFF;
    } else {
        // I/O access
        const uint16_t port = address & 0xFFFF;

        if (port == 0x00) {
            const uint16_t tmp = prevFibNo;
            prevFibNo = currFibNo;
            currFibNo += tmp;
            return currFibNo;
        }
        return 0xFFFF;
    }
}

void __time_critical_func(i8086_write)(const uint32_t address, const uint16_t data, const bool is_memory_access) {
    if (is_memory_access) {
        // Memory write
        if (address < RAM_SIZE) {
            // RAM area: 0x00000-0x0FFFF (64K)
            const uint32_t aligned_offset = address & RAM_MASK;
            *(uint16_t *) &ram[aligned_offset] = data;
        }
    } else {
        // I/O write
        const uint16_t port = address & 0xFFFF;

        if (port == 0x00) {
            prevFibNo = 1;
            currFibNo = 1;
        }
    }
}

void __time_critical_func(bus_write_handler)() {
    while (pio_sm_get_rx_fifo_level(BUS_CTRL_PIO, BUS_CTRL_SM) >= 2) {
        // First FIFO entry: address + control signals
        const uint32_t bus_state = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);
        const bus_info_t bus = parse_bus_state(bus_state);

        // Second FIFO entry: data from AD0-AD15
        const uint16_t data = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM) & 0xFFFF;

        printf("Write %05x : %x m_io %d\n", bus.address, data, bus.m_io);


        // i8086_write(bus.address, data, bus.m_io);
    }

    pio_interrupt_clear(BUS_CTRL_PIO, 0);
}

void __time_critical_func(bus_read_handler)() {
    while (pio_sm_get_rx_fifo_level(BUS_CTRL_PIO, BUS_CTRL_SM) > 0) {
        // FIFO entry: address + control signals
        const uint32_t bus_state = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);
        const bus_info_t bus = parse_bus_state(bus_state);

        // Read data and send back to PIO
        // const uint16_t data = bus.address & 2 ? 0xF4F4 : 0xFEEB;
        const uint16_t data = i8086_read(0xF0000 | bus.address, bus.m_io);
        pio_sm_put_blocking(BUS_CTRL_PIO, BUS_CTRL_SM, data);

        printf("Read %x : %x mio %x\n", bus.address, data, bus.m_io);
    }

    pio_interrupt_clear(BUS_CTRL_PIO, 1);
}

void cpu_bus_init() {
    const uint pio_offset = pio_add_program(BUS_CTRL_PIO, &i8086_bus_program);
    i8086_bus_program_init(BUS_CTRL_PIO, BUS_CTRL_SM, pio_offset);

    pio_set_irq0_source_enabled(BUS_CTRL_PIO, pis_interrupt0, true);
    irq_set_exclusive_handler(WRITE_IRQ, bus_write_handler);
    irq_set_priority(WRITE_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(WRITE_IRQ, true);

    pio_set_irq1_source_enabled(BUS_CTRL_PIO, pis_interrupt1, true);
    irq_set_exclusive_handler(READ_IRQ, bus_read_handler);
    irq_set_priority(READ_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(READ_IRQ, true);

    pio_sm_set_enabled(BUS_CTRL_PIO, BUS_CTRL_SM, true);
}
