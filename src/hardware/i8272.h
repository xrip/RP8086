/* Intel 8272A Floppy Disk Controller (упрощенная эмуляция для BIOS) */
#pragma once

#include "common.h"
#include "i8259.h"
#include <stdio.h>

// #define DEBUG_I8272
#if defined(DEBUG_I8272)
#define debug_log(...) printf(__VA_ARGS__)
#else
#define debug_log(...)
#endif

#define FDC_PORT_DOR  0x3F2
#define FDC_PORT_MSR  0x3F4
#define FDC_PORT_DATA 0x3F5

#define FDD_SECTOR_SIZE 512

// Floppy geometry - auto-detected from FDD360[] size
// Объявляем extern - определение в main.c
extern uint16_t FDD_CYLINDERS;
extern uint16_t FDD_HEADS;
extern uint16_t FDD_SECTORS_PER_TRACK;

extern i8272_s i8272;
extern uint8_t FDD360[];
extern uint8_t RAM[];
extern dma_channel_s dma_channels[];

// Auto-detect floppy geometry from array size (вызывается один раз при инициализации)
__force_inline static void fdd_detect_geometry(const size_t fdd_size) {

    // Default: 360KB (40 cyls, 2 heads, 9 sectors)
    FDD_CYLINDERS = 40;
    FDD_HEADS = 2;
    FDD_SECTORS_PER_TRACK = 9;

    // Detect by size (based on your insertdisk() logic)
    if (fdd_size <= 163840) {          // 160KB: single-sided, 8 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 1;
        FDD_SECTORS_PER_TRACK = 8;
    } else if (fdd_size <= 184320) {   // 180KB: single-sided, 9 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 1;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 327680) {   // 320KB: double-sided, 8 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 8;
    } else if (fdd_size <= 368640) {   // 360KB: double-sided, 9 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 737280) {   // 720KB: 80 cyls, 2 heads, 9 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 1228800) {  // 1.2MB: 80 cyls, 2 heads, 15 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 15;
    } else if (fdd_size <= 1474560) {  // 1.44MB: 80 cyls, 2 heads, 18 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 18;
    } else if (fdd_size <= 2949120) {  // 2.88MB: 80 cyls, 2 heads, 36 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 36;
    }

    debug_log("FDD: Auto-detected geometry from %lu bytes: C=%d, H=%d, S=%d\n",
           (unsigned long)fdd_size, FDD_CYLINDERS, FDD_HEADS, FDD_SECTORS_PER_TRACK);
}

__force_inline static void i8272_reset(void) {
    memset(&i8272, 0, sizeof(i8272_s));
    i8272.msr_rqm = 1;
    i8272.reset_pending = 1;
    i8272.current_cylinder = 0;

    debug_log("FDC: Controller reset completed, pending status read\n");
}

__force_inline static uint8_t i8272_get_msr(void) {
    return (i8272.msr_rqm ? 0x80 : 0) |
           (i8272.msr_dio ? 0x40 : 0) |
           (i8272.msr_ndma ? 0x20 : 0) |
           (i8272.msr_busy ? 0x10 : 0) |
           (i8272.msr_acta ? 0x01 : 0);
}

__force_inline static void i8272_irq(void) {
    i8259_interrupt(6);

    debug_log("FDC: IRQ6 generated (BIOS will set flag 0x43E)\n");
}

__force_inline static uint8_t i8272_read_result(void) {
    if (i8272.result_index < i8272.result_count) {
        uint8_t data = i8272.result_buffer[i8272.result_index++];

        if (i8272.result_index >= i8272.result_count) {
            i8272.msr_rqm = 1;
            i8272.msr_dio = 0;
            i8272.msr_busy = 0;
            i8272.msr_acta = 0;
        } else {
            i8272.msr_rqm = 1;
            i8272.msr_dio = 1;
            i8272.msr_busy = 1;
        }

        return data;
    }
    return 0xFF;
}

__force_inline static void i8272_cmd_specify(void) {
    debug_log("FDC: SPECIFY - SRT/HUT=0x%02X, HLT/NDMA=0x%02X, NDMA=%d\n",
           i8272.command_buffer[1], i8272.command_buffer[2], i8272.command_buffer[2] & 0x01);

    i8272.msr_ndma = i8272.command_buffer[2] & 0x01;
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

__force_inline static void i8272_cmd_recalibrate(void) {
    i8272.current_cylinder = 0;
    i8272.current_drive = i8272.command_buffer[1] & 0x03;

    debug_log("FDC: RECALIBRATE - Drive=%d, target cylinder=0, IRQ6 sent\n", i8272.current_drive);

    i8272.interrupt_pending = 1;
    i8272_irq();
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

__force_inline static void i8272_cmd_sense_interrupt(void) {
    uint8_t st0 = 0x20 | i8272.current_drive;
    i8272.result_buffer[0] = st0;
    i8272.result_buffer[1] = i8272.current_cylinder;
    i8272.result_count = 2;
    i8272.result_index = 0;

    debug_log("FDC: SENSE_INTERRUPT - ST0=0x%02X, PCN=%d (BIOS will clear flag 0x43E)\n", st0, i8272.current_cylinder);

    i8272.interrupt_pending = 0;
    i8272.command_index = 0;
    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
}

__force_inline static void i8272_cmd_sense_status(void) {
    if (i8272.reset_pending) {
        static const uint8_t reset_status[] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02};
        memcpy(i8272.result_buffer, reset_status, sizeof(reset_status));
        i8272.reset_pending = 0;

        debug_log("FDC: SENSE_STATUS after reset - 7 bytes ready, ST0=0xC0\n");
    } else {
        // ST0: IC=00 (Normal), SE=0, EC=0, NR=0, HD=0, US(drive)
        i8272.result_buffer[0] = i8272.current_drive & 0x03;
        i8272.result_buffer[1] = 0x00;  // ST1
        i8272.result_buffer[2] = 0x00;  // ST2
        i8272.result_buffer[3] = i8272.current_cylinder;  // C
        i8272.result_buffer[4] = 0x00;  // H
        i8272.result_buffer[5] = 0x01;  // R
        i8272.result_buffer[6] = 0x02;  // N

        debug_log("FDC: SENSE_STATUS normal - 7 bytes ready, ST0=0x%02X\n", i8272.result_buffer[0]);
    }

    i8272.result_count = 7;
    i8272.result_index = 0;
    i8272.command_index = 0;
    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
    i8272.msr_acta = 1;
}

__force_inline static void i8272_cmd_seek(void) {
    i8272.current_drive = (i8272.command_buffer[1] >> 2) & 1;
    i8272.current_cylinder = i8272.command_buffer[2];

    debug_log("FDC: SEEK - Drive=%d, target cylinder=%d, IRQ6 sent\n",
           i8272.current_drive, i8272.current_cylinder);

    i8272.interrupt_pending = 1;
    i8272_irq();
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

// -----------------------------------------------------------------------------
// Общие вспомогательные функции
// -----------------------------------------------------------------------------

static void i8272_set_result(uint8_t st0, uint8_t st1, uint8_t st2,
                             uint8_t c, uint8_t h, uint8_t r, uint8_t n) {
    i8272.result_buffer[0] = st0;
    i8272.result_buffer[1] = st1;
    i8272.result_buffer[2] = st2;
    i8272.result_buffer[3] = c;
    i8272.result_buffer[4] = h;
    i8272.result_buffer[5] = r;
    i8272.result_buffer[6] = n;
    i8272.result_count = 7;
    i8272.result_index = 0;

    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
    i8272.msr_acta = 1;

    i8272_irq();
    i8272.command_index = 0;
}

static bool i8272_validate_chrn(uint8_t c, uint8_t h, uint8_t r) {
    return !(c >= FDD_CYLINDERS || h >= FDD_HEADS ||
             r < 1 || r > FDD_SECTORS_PER_TRACK);
}

// -----------------------------------------------------------------------------
// READ DATA (0x06)
// -----------------------------------------------------------------------------

__force_inline static void i8272_cmd_read_data(void) {
    uint8_t drive  = i8272.command_buffer[1] & 3;
    uint8_t head   = (i8272.command_buffer[1] >> 2) & 1;
    uint8_t cyl    = i8272.command_buffer[2];
    uint8_t sector = i8272.command_buffer[4];
    uint8_t eot    = i8272.command_buffer[6];
    uint8_t ncode  = i8272.command_buffer[5];

    if (!i8272_validate_chrn(cyl, head, sector)) {
        i8272_set_result(0x40, 0x80, 0x00, cyl, head, sector, ncode);
        return;
    }

    uint8_t count = MIN(eot - sector + 1, FDD_SECTORS_PER_TRACK - sector + 1);
    uint32_t offset = ((cyl * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (sector - 1)) * FDD_SECTOR_SIZE;
    uint32_t size = count * FDD_SECTOR_SIZE;

    uint32_t transferred = i8237_write(2, &FDD360[offset], size);
    uint8_t last_sector = sector + (transferred / FDD_SECTOR_SIZE) - 1;

    i8272_set_result((head << 2) | drive, 0x00, 0x00, cyl, head, last_sector, ncode);
}

// -----------------------------------------------------------------------------
// WRITE DATA (0x05)
// -----------------------------------------------------------------------------

__force_inline static void i8272_cmd_write_data(void) {
    uint8_t drive  = i8272.command_buffer[1] & 3;
    uint8_t head   = (i8272.command_buffer[1] >> 2) & 1;
    uint8_t cyl    = i8272.command_buffer[2];
    uint8_t sector = i8272.command_buffer[4];
    uint8_t eot    = i8272.command_buffer[6];
    uint8_t ncode  = i8272.command_buffer[5];

    if (!i8272_validate_chrn(cyl, head, sector)) {
        i8272_set_result(0x40, 0x80, 0x00, cyl, head, sector, ncode);
        return;
    }

    uint8_t count = MIN(eot - sector + 1, FDD_SECTORS_PER_TRACK - sector + 1);
    uint32_t offset = ((cyl * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (sector - 1)) * FDD_SECTOR_SIZE;
    uint32_t size = count * FDD_SECTOR_SIZE;

    uint32_t received = i8237_read(2, &FDD360[offset], size);
    uint8_t last_sector = sector + (received / FDD_SECTOR_SIZE) - 1;

    i8272_set_result((head << 2) | drive, 0x00, 0x00, cyl, head, last_sector, ncode);
}



__force_inline static void i8272_write_command(const uint8_t data) {
    if (!i8272.msr_rqm || i8272.msr_dio || i8272.msr_busy) {
        return;
    }

    if (i8272.command_index == 0) {
        i8272.command_buffer[0] = data;
        i8272.command_index = 1;

        // Логируем начало команды READ
        if ((data & 0x1F) == 0x06) {
            debug_log("[%llu] FDC: Received READ command byte 0x%02X\n",
                   to_us_since_boot(get_absolute_time()), data);
        }

        if ((data & 0x1F) == 0x08) {
            if (i8272.reset_pending) {
                i8272_cmd_sense_status();
            } else {
                i8272_cmd_sense_interrupt();
            }
        }
        return;
    }

    i8272.command_buffer[i8272.command_index++] = data;
    const uint8_t cmd = i8272.command_buffer[0] & 0x1F;

    switch (cmd) {
        case 0x03: if (i8272.command_index >= 3) i8272_cmd_specify(); break;
        case 0x07: if (i8272.command_index >= 2) i8272_cmd_recalibrate(); break;
        case 0x0F: if (i8272.command_index >= 3) i8272_cmd_seek(); break;
        case 0x06: case 0x46:
            if (i8272.command_index >= 9) i8272_cmd_read_data();
            break;
    }
}

__force_inline static uint8_t i8272_readport(const uint16_t port) {
    switch (port) {
        case FDC_PORT_MSR: {
            uint8_t msr = i8272_get_msr();
            debug_log("FDC: Read MSR=0x%02X (RQM=%d,DIO=%d,BUSY=%d,ACTA=%d)\n", msr,
                   (msr & 0x80) ? 1 : 0, (msr & 0x40) ? 1 : 0,
                   (msr & 0x10) ? 1 : 0, (msr & 0x01) ? 1 : 0);
            return msr;
        }
        case FDC_PORT_DATA: {
            uint8_t data = i8272_read_result();
            debug_log("FDC: Read DATA=0x%02X\n", data);
            return data;
        }
        default: return 0xFF;
    }
}

__force_inline static void i8272_writeport(const uint16_t port, const uint8_t data) {
    switch (port) {
        case FDC_PORT_DOR: {
            static uint8_t last_dor = 0;
            i8272.msr_actc = (data >> 2) & 1;

            debug_log("FDC: Write DOR=0x%02X\n", data);

            if ((last_dor & 0x04) == 0 && (data & 0x04) == 4) {
                debug_log("FDC: Reset detected, performing reset\n");
                i8272_reset();
                i8272_irq();
            }
            last_dor = data;
            break;
        }
        case FDC_PORT_DATA:
            debug_log("FDC: Write DATA=0x%02X\n", data);
            i8272_write_command(data);
            break;
    }
}
#if defined(DEBUG_I8272)
#undef debug_log
#endif