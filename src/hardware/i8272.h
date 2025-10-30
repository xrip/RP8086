/* Intel 8272A Floppy Disk Controller (упрощенная эмуляция для BIOS) */
#pragma once

#include "common.h"
#include "i8259.h"
#include <stdio.h>

#define DEBUG_FDC 0

#define FDC_PORT_DOR  0x3F2
#define FDC_PORT_MSR  0x3F4
#define FDC_PORT_DATA 0x3F5

#define FDD_SECTORS_PER_TRACK 9
#define FDD_HEADS             2
#define FDD_CYLINDERS         40
#define FDD_SECTOR_SIZE       512

extern i8272_s i8272;
extern uint8_t FDD360[];
extern uint8_t RAM[];

__force_inline static void i8272_reset(void) {
    memset(&i8272, 0, sizeof(i8272_s));
    i8272.msr_rqm = 1;
    i8272.reset_pending = 1;
    i8272.current_cylinder = 0;

    #if DEBUG_FDC
    printf("FDC: Controller reset completed, pending status read\n");
    #endif
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

    #if DEBUG_FDC
    printf("FDC: IRQ6 generated (BIOS will set flag 0x43E)\n");
    #endif
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
    #if DEBUG_FDC
    printf("FDC: SPECIFY - SRT/HUT=0x%02X, HLT/NDMA=0x%02X, NDMA=%d\n",
           i8272.command_buffer[1], i8272.command_buffer[2], i8272.command_buffer[2] & 0x01);
    #endif

    i8272.msr_ndma = i8272.command_buffer[2] & 0x01;
    i8272.command_index = 0;
    i8272.msr_busy = 0;
    i8272.msr_rqm = 1;
}

__force_inline static void i8272_cmd_recalibrate(void) {
    i8272.current_cylinder = 0;
    i8272.current_drive = i8272.command_buffer[1] & 0x03;

    #if DEBUG_FDC
    printf("FDC: RECALIBRATE - Drive=%d, target cylinder=0, IRQ6 sent\n", i8272.current_drive);
    #endif

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

    #if DEBUG_FDC
    printf("FDC: SENSE_INTERRUPT - ST0=0x%02X, PCN=%d (BIOS will clear flag 0x43E)\n", st0, i8272.current_cylinder);
    #endif

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

        #if DEBUG_FDC
        printf("FDC: SENSE_STATUS after reset - 7 bytes ready, ST0=0xC0\n");
        #endif
    } else {
        i8272.result_buffer[0] = i8272.current_drive;
        i8272.result_buffer[1] = 0x00;
        i8272.result_buffer[2] = 0x00;
        i8272.result_buffer[3] = i8272.current_cylinder;
        i8272.result_buffer[4] = 0x00;
        i8272.result_buffer[5] = 0x01;
        i8272.result_buffer[6] = 0x02;

        #if DEBUG_FDC
        printf("FDC: SENSE_STATUS normal - 7 bytes ready, ST0=0x%02X\n", i8272.result_buffer[0]);
        #endif
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

    #if DEBUG_FDC
    printf("FDC: SEEK - Drive=%d, target cylinder=%d, IRQ6 sent\n",
           i8272.current_drive, i8272.current_cylinder);
    #endif

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
    const uint8_t sector = i8272.command_buffer[4];

    #if DEBUG_FDC
    printf("FDC: READ_DATA - C=%d, H=%d, R=%d, N=%d, Drive=%d\n",
           cylinder, head, sector, i8272.command_buffer[5], i8272.current_drive);
    #endif

    if (cylinder >= FDD_CYLINDERS || head >= FDD_HEADS ||
        sector < 1 || sector > FDD_SECTORS_PER_TRACK) {
        #if DEBUG_FDC
        printf("FDC: READ_DATA ERROR - Invalid parameters\n");
        #endif

        i8272.result_buffer[0] = 0x40;  // Abnormal termination
        i8272.result_buffer[1] = 0x80;  // No data
        i8272.result_buffer[2] = 0x00;
        i8272.result_buffer[3] = cylinder;
        i8272.result_buffer[4] = head;
        i8272.result_buffer[5] = sector;
        i8272.result_buffer[6] = i8272.command_buffer[5];
    } else {
        const uint32_t lba = (cylinder * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (sector - 1);
        const uint32_t offset = lba * FDD_SECTOR_SIZE;

        #if 1
        printf("FDC: READ_DATA OK - LBA=%d, offset=0x%04X, DMA transfer start\n", lba, offset);
        #endif

        i8237_write(2, &FDD360[offset], FDD_SECTOR_SIZE);
        // memcpy(RAM, FDD360, FDD_SECTOR_SIZE);
        // for (int i = 0; i < FDD_SECTOR_SIZE; i++) {
            // i8237_write(2, FDD360[offset + i]);
        // }

        i8272.result_buffer[0] = i8272.current_drive;
        i8272.result_buffer[1] = 0x00;  // No errors
        i8272.result_buffer[2] = 0x00;  // No errors
        i8272.result_buffer[3] = cylinder;
        i8272.result_buffer[4] = head;
        i8272.result_buffer[5] = sector + 1;
        i8272.result_buffer[6] = i8272.command_buffer[5];
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
            #if DEBUG_FDC
            printf("FDC: Read MSR=0x%02X (RQM=%d,DIO=%d,BUSY=%d,ACTA=%d)\n", msr,
                   (msr & 0x80) ? 1 : 0, (msr & 0x40) ? 1 : 0,
                   (msr & 0x10) ? 1 : 0, (msr & 0x01) ? 1 : 0);
            #endif
            return msr;
        }
        case FDC_PORT_DATA: {
            uint8_t data = i8272_read_result();
            #if DEBUG_FDC
            printf("FDC: Read DATA=0x%02X\n", data);
            #endif
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

            #if DEBUG_FDC
            printf("FDC: Write DOR=0x%02X\n", data);
            #endif

            if ((last_dor & 0x04) == 0 && (data & 0x04) == 4) {
                #if DEBUG_FDC
                printf("FDC: Reset detected, performing reset\n");
                #endif
                i8272_reset();
                i8272_irq();
            }
            last_dor = data;
            break;
        }
        case FDC_PORT_DATA:
            #if DEBUG_FDC
            printf("FDC: Write DATA=0x%02X\n", data);
            #endif
            i8272_write_command(data);
            break;
    }
}