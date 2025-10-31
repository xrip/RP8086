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
    if (fdd_size <= 163840) {
        // 160KB: single-sided, 8 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 1;
        FDD_SECTORS_PER_TRACK = 8;
    } else if (fdd_size <= 184320) {
        // 180KB: single-sided, 9 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 1;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 327680) {
        // 320KB: double-sided, 8 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 8;
    } else if (fdd_size <= 368640) {
        // 360KB: double-sided, 9 sectors
        FDD_CYLINDERS = 40;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 737280) {
        // 720KB: 80 cyls, 2 heads, 9 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 9;
    } else if (fdd_size <= 1228800) {
        // 1.2MB: 80 cyls, 2 heads, 15 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 15;
    } else if (fdd_size <= 1474560) {
        // 1.44MB: 80 cyls, 2 heads, 18 sectors
        FDD_CYLINDERS = 80;
        FDD_HEADS = 2;
        FDD_SECTORS_PER_TRACK = 18;
    } else if (fdd_size <= 2949120) {
        // 2.88MB: 80 cyls, 2 heads, 36 sectors
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
    i8272.msr_ndma = i8272.command_buffer[2] & 0x01;
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

__force_inline static void i8272_cmd_recalibrate(void) {
    i8272.current_cylinder = 0;
    i8272.current_drive = i8272.command_buffer[1] & 0x03;
    i8272.interrupt_pending = 1;
    i8272_irq();
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

__force_inline static void i8272_cmd_sense_interrupt(void) {
    i8272.result_buffer[0] = 0x20 | i8272.current_drive;
    i8272.result_buffer[1] = i8272.current_cylinder;
    i8272.result_count = 2;
    i8272.result_index = 0;
    i8272.interrupt_pending = 0;
    i8272.command_index = 0;
    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
}

__force_inline static void i8272_cmd_sense_status(void) {
    if (i8272.reset_pending) {
        // Reset status: ST0=0xC0 (interrupted), все остальные параметры 0 или default
        static const uint8_t reset_status[] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02};
        memcpy(i8272.result_buffer, reset_status, sizeof(reset_status));
        i8272.reset_pending = 0;
    } else {
        // Normal status: ST0=drive, ST1/ST2=0, C/H/R/N
        i8272.result_buffer[0] = i8272.current_drive & 0x03;
        i8272.result_buffer[1] = 0x00; // ST1
        i8272.result_buffer[2] = 0x00; // ST2
        i8272.result_buffer[3] = i8272.current_cylinder;
        i8272.result_buffer[4] = 0x00; // H
        i8272.result_buffer[5] = 0x01; // R
        i8272.result_buffer[6] = 0x02; // N
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
    i8272.interrupt_pending = 1;
    i8272_irq();
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

// -----------------------------------------------------------------------------
// Общие вспомогательные функции
// -----------------------------------------------------------------------------

// Быстрая установка результата операции с inline оптимизацией
__force_inline static void i8272_set_result(const uint8_t status_register_0, const uint8_t status_register_1, const uint8_t status_register_2,
                                            const uint8_t cylinder, const uint8_t head, const uint8_t sector, const uint8_t ncode) {
    // Прямое заполнение буфера результата (7 байт)
    i8272.result_buffer[0] = status_register_0;
    i8272.result_buffer[1] = status_register_1;
    i8272.result_buffer[2] = status_register_2;
    i8272.result_buffer[3] = cylinder;
    i8272.result_buffer[4] = head;
    i8272.result_buffer[5] = sector;
    i8272.result_buffer[6] = ncode;

    // Установка счетчиков и флагов MSR одной операцией
    i8272.result_count = 7;
    i8272.result_index = 0;
    i8272.command_index = 0;

    // MSR: RQM=1, DIO=1, BUSY=1, ACTA=1
    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
    i8272.msr_acta = 1;

    i8272_irq();
}

// Быстрая валидация параметров CHS
__force_inline static bool i8272_validate_chs(const uint8_t cylinder, const uint8_t head, const uint8_t sector) {
    return (cylinder < FDD_CYLINDERS) & (head < FDD_HEADS) & (sector > 0) & (sector <= FDD_SECTORS_PER_TRACK);
}

// -----------------------------------------------------------------------------
// Общая функция для READ/WRITE DATA
// -----------------------------------------------------------------------------

__force_inline static void i8272_cmd_rw_data(const bool is_write) {
    const uint8_t drive = i8272.command_buffer[1] & 3;
    const uint8_t head = (i8272.command_buffer[1] >> 2) & 1;
    const uint8_t cylinder = i8272.command_buffer[2];
    const uint8_t sector = i8272.command_buffer[4];
    const uint8_t eot = i8272.command_buffer[6];
    const uint8_t ncode = i8272.command_buffer[5];

    // Валидация параметров
    if (!i8272_validate_chs(cylinder, head, sector)) {
        i8272_set_result(0x40, 0x80, 0x00, cylinder, head, sector, ncode);
        return;
    }

    // Расчет параметров передачи
    const uint8_t count = MIN(eot - sector + 1, FDD_SECTORS_PER_TRACK - sector + 1);
    const uint32_t offset = ((cylinder * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (sector - 1)) * FDD_SECTOR_SIZE;
    const uint32_t size = count * FDD_SECTOR_SIZE;

    // DMA операция (read или write)
    const uint32_t transferred = is_write
                                     ? i8237_read(2, &FDD360[offset], size) // WRITE: RAM → FDD
                                     : i8237_write(2, &FDD360[offset], size); // READ: FDD → RAM

    const uint8_t last_sector = sector + (transferred / FDD_SECTOR_SIZE) - 1;
    i8272_set_result((head << 2) | drive, 0x00, 0x00, cylinder, head, last_sector, ncode);
}


__force_inline static void i8272_write_command(const uint8_t data) {
    // Проверка готовности контроллера (RQM=1, DIO=0, BUSY=0)
    if (!i8272.msr_rqm || i8272.msr_dio || i8272.msr_busy) return;

    // Первый байт команды - определяет тип операции
    if (i8272.command_index == 0) {
        i8272.command_buffer[0] = data;
        i8272.command_index = 1;

        // SENSE INTERRUPT (0x08) - немедленное выполнение
        const uint8_t cmd = data & 0x1F;
        if (cmd == 0x08) {
            i8272.reset_pending ? i8272_cmd_sense_status() : i8272_cmd_sense_interrupt();
        }
        return;
    }

    // Сбор параметров команды
    i8272.command_buffer[i8272.command_index++] = data;
    const uint8_t cmd = i8272.command_buffer[0] & 0x1F;

    // Диспетчеризация команд по длине
    switch (cmd) {
        case 0x07: if (i8272.command_index >= 2) i8272_cmd_recalibrate();
            break; // RECALIBRATE (2 bytes)
        case 0x03: if (i8272.command_index >= 3) i8272_cmd_specify();
            break; // SPECIFY (3 bytes)
        case 0x0F: if (i8272.command_index >= 3) i8272_cmd_seek();
            break; // SEEK (3 bytes)
        case 0x05:
        case 0x45: if (i8272.command_index >= 9) i8272_cmd_rw_data(true);
            break; // WRITE (9 bytes)
        case 0x06:
        case 0x46: if (i8272.command_index >= 9) i8272_cmd_rw_data(false);
            break; // READ (9 bytes)
    }
}

__force_inline static uint8_t i8272_readport(const uint16_t port_number) {
    switch (port_number) {
        case FDC_PORT_MSR:
            return i8272_get_msr();
        case FDC_PORT_DATA:
            return i8272_read_result();
    }
}

__force_inline static void i8272_writeport(const uint16_t port_number, const uint8_t data) {
    switch (port_number) {
        case FDC_PORT_DOR: {
            static uint8_t last_dor = 0;
            i8272.msr_actc = (data >> 2) & 1;

            // Обнаружение reset-последовательности (0x00 → 0x04)
            if ((last_dor & 0x04) == 0 && (data & 0x04) == 4) {
                i8272_reset();
                i8272_irq();
            }
            last_dor = data;
            break;
        }
        case FDC_PORT_DATA:
            i8272_write_command(data);
            break;
    }
}
#if defined(DEBUG_I8272)
#undef debug_log
#endif
