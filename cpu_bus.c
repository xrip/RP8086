#include "i8086_bus.pio.h"
#include "cpu_bus.h"

#include <stdio.h>

#include "config.h"
#include "bios.h"
#include "pico/multicore.h"

// ROM configuration
#define BIOS_ROM_SIZE          sizeof(GLABIOS_0_4_1_8T_ROM) // 8K BIOS
#define BIOS_ROM_BASE          (0x100000 - BIOS_ROM_SIZE)   // 0xFE000-0xFFFFF
#define BIOS_ROM_MASK          (BIOS_ROM_SIZE - 1)          // 0x1FFF
#define BIOS_ROM_MASK_16BIT    (BIOS_ROM_SIZE - 2)          // 0x1FFE

// RAM configuration (RP2040 has 264KB SRAM, use 224KB for i8086)
#define RAM_SIZE         (128 * 1024)                        // 224KB
#define RAM_MASK         (RAM_SIZE - 1)                     // 0xFFFF
#define RAM_MASK_16BIT   (RAM_SIZE - 2)                     // 0xFFFE

uint8_t ram[RAM_SIZE] __attribute__((aligned(4)));
uint8_t videoram[4096] __attribute__((aligned(4)));
// Temporary test variables
static uint16_t prevFibNo = 1;
static uint16_t currFibNo = 1;
static uint64_t log_event_counter = 0;

static inline void __time_critical_func(log_event)(log_type_t type, uint32_t address, uint16_t data, bool bhe, bool m_io) {
    if (address >= 0xB0000 && address < 0xB8000) {
        // Получаем индекс для новой записи
        uint32_t current_head = log_buffer.head;

        // Заполняем данные
        log_buffer.buffer[current_head].type = type;
        log_buffer.buffer[current_head].address = address;
        log_buffer.buffer[current_head].data = data;
        log_buffer.buffer[current_head].bhe = bhe;
        log_buffer.buffer[current_head].mio = m_io;
        log_buffer.buffer[current_head].timestamp = log_event_counter++;

        // Продвигаем head для следующей записи
        log_buffer.head = (current_head + 1) & (LOG_BUFFER_SIZE - 1);

        // Проверяем, есть ли место в аппаратном FIFO.
        // Если его нет, мы пропустим этот лог, но ISR не заблокируется!
        if (multicore_fifo_wready()) {
            // Отправляем УКАЗАТЕЛЬ на нашу запись во второе ядро.
            // Мы можем отправить либо полный указатель, либо только индекс. Индекс эффективнее.
            multicore_fifo_push_blocking_inline((uint32_t)current_head);
        }
    }
}

// Extract bus transaction info from PIO FIFO data
static inline bus_info_t parse_bus_state(const uint32_t bus_data) {
    bus_info_t info;

    info.address = bus_data & 0xFFFFF;       // Bits [19:0]
    info.ale     = (bus_data >> 20) & 1;     // Bit [21]
    info.rd      = (bus_data >> 21) & 1;     // Bit [21]
    info.wr      = (bus_data >> 22) & 1;     // Bit [22]
    info.inta    = (bus_data >> 23) & 1;     // Bit [23]
    info.m_io    = (bus_data >> 24) & 1;     // Bit [24]
    info.bhe     = (bus_data >> 25) & 1;     // Bit [25]

    return info;
}

uint8_t ports[0xfff] __attribute__((aligned(4))) = { 0xFF };
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
        } else if (address >= 0xB0000 && address < 0xB8000) {
            return *(uint16_t *) &videoram[address & 4094];
        }
        // Unmapped memory
        return 0xFFFF;
    } else {
        // I/O access
        const uint16_t port = address & 0xFFE;
        static uint8_t port3DA = 0;
        if (port == 0x3BA) {
            port3DA ^= 1;
            if (!(port3DA & 1)) port3DA ^= 8;
             return port3DA;

        }

        // if (port == 0x40 || port == 0x61 || port == 0x41 || port == 0x21) {
            // return 0x0000;
        // }

        // return *(uint16_t *) &ports[port];
        // return 0;
        return 0xFFFF;
    }
}


void __time_critical_func(i8086_write)(uint32_t address, const uint16_t data, const bool is_memory_access, const bool bhe) {
    if (is_memory_access) {
        // Memory write
        if (address < RAM_SIZE) {
            const bool a0 = address & 1;

            // Определяем тип операции по BHE и A0
            if (!bhe && !a0) {
                // 16-bit transfer: BHE=0, A0=0
                *(uint16_t *) &ram[address] = data;
            } else if (bhe && !a0) {
                // Byte transfer на четном адресе: BHE=1, A0=0 (только младший байт D7-D0)
                ram[address] = data & 0xFF;
            } else if (!bhe && a0) {
                // Byte transfer на нечетном адресе: BHE=0, A0=1 (только старший байт D15-D8)
                ram[address] = (data >> 8) & 0xFF;
            }
            // BHE=1, A0=1 - недопустимая комбинация, игнорируем
        } else if (address >= 0xB0000 && address < 0xB8000) {

            // printf ("VIDEO RAM write at %05x %x %d\n", address, data, bhe);
            const bool a0 = address & 1;
            address &= 4095;
            // Определяем тип операции по BHE и A0
            if (!bhe && !a0) {
                // 16-bit transfer: BHE=0, A0=0
                const uint16_t tmp_addr = address & 4094;
                //printf("\x1b[%d;%dH%c", 1+(address / 2) / 80, 1+(address / 2) % 80, data & 0xFF);
                *(uint16_t *) &videoram[address] = data;
            } else if (bhe && !a0) {
                // Byte transfer на четном адресе: BHE=1, A0=0 (только младший байт D7-D0)
                videoram[address] = data & 0xFF;
                // printf("\x1b[%d;%dH%c", address / 80, address % 80, data & 0xFF);
            } else if (!bhe && a0) {
                // Byte transfer на нечетном адресе: BHE=0, A0=1 (только старший байт D15-D8)
                videoram[address] = (data >> 8) & 0xFF;
                // printf("\x1b[%d;%dH%c", 1+(address / 2) / 80, 1+(address / 2) % 80, (data >> 8) & 0xFF);
            }

        }
    } else {
        // I/O write

        const bool a0 = address & 1;
        // Определяем тип операции по BHE и A0
        if (!bhe && !a0) {
            *(uint16_t *) &ports[address & 0xFFE] = data;
        } else if (bhe && !a0) {
            ports[address & 0xFFF] = data & 0xFF;
        } else if (!bhe && a0) {
            ports[address & 0xFFF] = (data >> 8) & 0xFF;
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
        i8086_write(bus.address, data, bus.m_io, bus.bhe);

        log_event(LOG_WRITE, bus.address, data, bus.bhe, bus.m_io);
        // if (!bus.m_io)
        {
            // printf(">> %s WRITE %05x : %04x BHE:%d\n", !bus.m_io ? "PORT" : "", bus.address, data, bus.bhe);
        }
    }
    pio_interrupt_clear(BUS_CTRL_PIO, 0);

}
bool irq_pending1 = false;
void __time_critical_func(bus_read_handler)() {
    static bool tmp = false;
    if (pio_interrupt_get(BUS_CTRL_PIO, 1)) {
        while (pio_sm_get_rx_fifo_level(BUS_CTRL_PIO, BUS_CTRL_SM) > 0) {
            // FIFO entry: address + control signals
            const uint32_t bus_state = pio_sm_get_blocking(BUS_CTRL_PIO, BUS_CTRL_SM);
            const bus_info_t bus = parse_bus_state(bus_state);

            // Read data and send back to PIO
            // const uint16_t data = bus.address & 2 ? 0xF4F4 : 0xFEEB;

            const uint16_t data = irq_pending1 ? 0xFF08 : i8086_read(bus.address, bus.m_io);
            pio_sm_put_blocking(BUS_CTRL_PIO, BUS_CTRL_SM, data);

            if (irq_pending1) {
                // printf("irq written\n");
                irq_pending1 = false;
            }

            log_event(LOG_READ, bus.address, data, bus.bhe, bus.m_io);
            // if (!bus.m_io)
            {
                // printf("<< %s READ %05x : %04x BHE:%d\n", !bus.m_io ? "PORT" : "", bus.address, data, bus.bhe);
            }
        }

        pio_interrupt_clear(BUS_CTRL_PIO, 1);
    } else if (pio_interrupt_get(BUS_CTRL_PIO, 3)){ // Обработка прерывания
            log_event(LOG_INTA, 0, 0, 0, 0);
            gpio_put(INTR_PIN, 0);
            irq_pending1 = true;

        pio_interrupt_clear(BUS_CTRL_PIO, 3);
    }
}

void cpu_bus_init() {
    const uint pio_offset = pio_add_program(BUS_CTRL_PIO, &i8086_bus_program);
    i8086_bus_program_init(BUS_CTRL_PIO, BUS_CTRL_SM, pio_offset);

    pio_set_irq0_source_enabled(BUS_CTRL_PIO, pis_interrupt0, true);
    irq_set_exclusive_handler(WRITE_IRQ, bus_write_handler);
    irq_set_priority(WRITE_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(WRITE_IRQ, true);

    pio_set_irq1_source_enabled(BUS_CTRL_PIO, pis_interrupt1, true);
    pio_set_irq1_source_enabled(BUS_CTRL_PIO, pis_interrupt3, true);
    irq_set_exclusive_handler(READ_IRQ, bus_read_handler);
    irq_set_priority(READ_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(READ_IRQ, true);

    pio_sm_set_enabled(BUS_CTRL_PIO, BUS_CTRL_SM, true);
}
