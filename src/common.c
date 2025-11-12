#include "common.h"
#include "../rom/fdd.h"
#include "hardware/i8237.h"
#include "hardware/i8272.h"

uint8_t RAM[RAM_SIZE] __attribute__((aligned(4), section(".psram")));
uint8_t VIDEORAM[VIDEORAM_SIZE] __attribute__((aligned(4)));

i8259_s i8259 __attribute__((aligned(4))) = {
    .interrupt_mask_register = 0xFF, // Все IRQ замаскированы
    .interrupt_vector_offset = 0x08, // Стандартный offset для IBM PC
};
i8253_s i8253 __attribute__((aligned(4))) = {
    {
        {.latched_value = 0xFFFF},
        {.latched_value = 0xFFFF},
        {.latched_value = 0xFFFF},
    }
};
i8272_s i8272 __attribute__((aligned(4)));
dma_channel_s dma_channels[DMA_CHANNELS];

uart_16550_s uart __attribute__((aligned(4))) = {
    .data_ready = false,
    .lcr = 0x03,  // 8 data bits, 1 stop bit, no parity (стандарт)
    .mcr = 0x00,
    .msr = 0xB0,  // CTS, DSR, DCD активны
};

mc6845_s mc6845 __attribute__((aligned(4)));
cga_s cga __attribute__((aligned(4)));