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

__force_inline static void i8272_cmd_read_data(void) {
    const uint8_t head = (i8272.command_buffer[1] >> 2) & 1;
    i8272.current_drive = i8272.command_buffer[1] & 0x03;
    const uint8_t cylinder = i8272.command_buffer[2];
    const uint8_t start_sector = i8272.command_buffer[4];
    const uint8_t eot = i8272.command_buffer[6];  // End of Track - last sector number on track
    const uint8_t sector_size_code = i8272.command_buffer[5];  // N: 0=128, 1=256, 2=512, 3=1024

    // Всегда логируем READ_DATA - это критически важная команда
    debug_log("[%llu] FDC: READ_DATA - C=%d, H=%d, R=%d (start), EOT=%d, N=%d, Drive=%d\n",
           to_us_since_boot(get_absolute_time()), cylinder, head, start_sector, eot, sector_size_code, i8272.current_drive);

    if (cylinder >= FDD_CYLINDERS || head >= FDD_HEADS ||
        start_sector < 1 || start_sector > FDD_SECTORS_PER_TRACK) {
        debug_log("[%llu] FDC: READ_DATA ERROR - Invalid parameters (C=%d/%d, H=%d/%d, R=%d)\n",
               to_us_since_boot(get_absolute_time()), cylinder, FDD_CYLINDERS, head, FDD_HEADS, start_sector);

        i8272.result_buffer[0] = 0x40;  // Abnormal termination
        i8272.result_buffer[1] = 0x80;  // No data
        i8272.result_buffer[2] = 0x00;
        i8272.result_buffer[3] = cylinder;
        i8272.result_buffer[4] = head;
        i8272.result_buffer[5] = start_sector;
        i8272.result_buffer[6] = sector_size_code;
    } else {
        // КРИТИЧЕСКИ ВАЖНО: Контроллер 8272A читает данные до тех пор, пока:
        // 1. Не достигнут сектор EOT, ИЛИ
        // 2. DMA контроллер не установит сигнал Terminal Count (TC)
        //
        // BIOS программирует DMA на ТОЧНОЕ количество байт (например, 512 для 1 сектора)
        // и передает EOT как максимальный номер сектора на дорожке.
        // Когда DMA досчитывает до 0, он устанавливает TC, и 8272A останавливается.
        //
        // Мы эмулируем это поведение: читаем данные порциями по 512 байт,
        // проверяя TC после каждого сектора. DMA контроллер сам управляет счетчиком.

        const uint32_t start_lba = (cylinder * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (start_sector - 1);
        uint32_t offset = start_lba * FDD_SECTOR_SIZE;

        uint8_t current_sector = start_sector;
        uint32_t total_bytes_transferred = 0;

        // Читаем сектора последовательно, пока DMA не установит TC
        while (current_sector <= eot && current_sector <= FDD_SECTORS_PER_TRACK) {
            // Передаем один сектор через DMA
            const uint32_t bytes_written = i8237_write(2, &FDD360[offset], FDD_SECTOR_SIZE);
            total_bytes_transferred += bytes_written;
            offset += FDD_SECTOR_SIZE;
            current_sector++;

            // КРИТИЧЕСКИ ВАЖНО: Проверяем флаг TC в DMA контроллере!
            // Флаг finished устанавливается когда count переходит через 0 → 0xFFFF
            if (dma_channels[2].finished) {
                // TC установлен - DMA досчитал до конца, останавливаем передачу
                debug_log("[%llu] FDC: READ_DATA stopped by DMA TC after %d bytes (%d sectors), last_sector=R%d\n",
                       to_us_since_boot(get_absolute_time()), total_bytes_transferred,
                       total_bytes_transferred / FDD_SECTOR_SIZE, current_sector - 1);

                // КРИТИЧНО: Сбрасываем флаг TC, чтобы следующая команда READ_DATA не остановилась сразу!
                // Этот флаг читается только FDC, поэтому мы можем безопасно его сбросить здесь.
                dma_channels[2].finished = 0;
                break;
            }
        }

        debug_log("[%llu] FDC: READ_DATA OK - LBA=%d, total=%d bytes (%d sectors), R=%d to R=%d\n",
               to_us_since_boot(get_absolute_time()), start_lba, total_bytes_transferred,
               (total_bytes_transferred + FDD_SECTOR_SIZE - 1) / FDD_SECTOR_SIZE, start_sector, current_sector - 1);

        // Intel 8272A Result Phase: 7 bytes (ST0, ST1, ST2, C, H, R, N)
        // ST0: IC(7-6)=00 (Normal), SE(5)=0, EC(4)=0, NR(3)=0, HD(2), US(1-0)
        i8272.result_buffer[0] = (head << 2) | (i8272.current_drive & 0x03);  // ST0
        i8272.result_buffer[1] = 0x00;  // ST1: No errors
        i8272.result_buffer[2] = 0x00;  // ST2: No errors
        i8272.result_buffer[3] = cylinder;  // C
        i8272.result_buffer[4] = head;      // H
        i8272.result_buffer[5] = current_sector;  // R: номер следующего сектора после последнего прочитанного
        i8272.result_buffer[6] = sector_size_code;  // N

        // CRITICAL DEBUG: Log only result phase (minimal overhead)
        debug_log("FDC_RES: C=%d H=%d R_start=%d R_end=%d bytes=%d TC=%d\n",
               cylinder, head, start_sector, current_sector, total_bytes_transferred, dma_channels[2].finished);
    }

    i8272.result_count = 7;
    i8272.result_index = 0;

    // Готовим MSR для чтения результата
    i8272.msr_rqm = 1;
    i8272.msr_dio = 1;
    i8272.msr_busy = 1;
    i8272.msr_acta = 1;

    i8272_irq();
    i8272.command_index = 0;
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