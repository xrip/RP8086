/* Intel 8272A Floppy Disk Controller (упрощенная эмуляция для BIOS) */
#pragma once


#include "i8259.h"

// #define DEBUG_I8272
#if defined(DEBUG_I8272)
#define debug_log(...) printf(__VA_ARGS__)
#else
#define debug_log(...)
#endif
extern uint8_t FDD360[];

// Consolidated controller state for tighter locality; align to cache line for fast access
typedef struct {
    uint8_t DOR;
    uint8_t response[4];
    uint8_t command[9];
    uint8_t result[7];
    uint8_t presentCylinder[4];
    uint8_t command_length;
    uint8_t result_count;
    uint8_t command_index;
    uint8_t result_index;
    uint8_t check_drives_mask;
} I8272State;

// Single instance of the controller state (zero-initialized)
static I8272State i8272_state __attribute__((aligned(4))) = {};

// Keep existing code unchanged by mapping old names to struct fields
#define DOR                (i8272_state.DOR)
#define response           (i8272_state.response)
#define command            (i8272_state.command)
#define result             (i8272_state.result)
#define presentCylinder    (i8272_state.presentCylinder)
#define command_length     (i8272_state.command_length)
#define result_count       (i8272_state.result_count)
#define command_index      (i8272_state.command_index)
#define result_index       (i8272_state.result_index)
#define check_drives_mask  (i8272_state.check_drives_mask)

#define FDD_CYLINDERS 40
#define FDD_HEADS 2
#define FDD_SECTORS_PER_TRACK 9
#define FDD_SECTOR_SIZE 512

#define FDC_PORT_DOR  0x3F2
#define FDC_PORT_MSR  0x3F4
#define FDC_PORT_DATA 0x3F5


#define FDC_CMD_SPECIFY  0x03
#define FDC_CMD_SENSE_DRIVE_STATUS 0x04
#define FDC_CMD_WRITE_DATA  0x05
#define FDC_CMD_READ_DATA  0x06
#define FDC_CMD_RECALIBRATE  0x07
#define FDC_CMD_SENSE_INTERRUPT_STATUS  0x08
#define FDC_CMD_READ_ID  0x0A
#define FDC_CMD_SEEK  0x0F

__force_inline static void i8272_irq(void) {
    // if interrupts enabled
    if (DOR & (1 << 3)) {
        i8259_interrupt(6);
        debug_log("FDC: IRQ6 generated\n");
    }
}

__force_inline static uint8_t i8272_readport(const uint16_t port_number) {
    switch (port_number) {
        case FDC_PORT_MSR: // floppy main status
        {
            // no drives busy, dma-mode
            return 0 |
                   (result_count ? 1 << 4 : 0) | // fdc busy if there's a result to read
                   (result_count ? 1 << 6 : 0) | // fdc->cpu if we have result data else cpu->fdc
                   1 << 7; // ready
        }
        case FDC_PORT_DATA: // floppy command/data
        {
            if (result_count) {
                const auto r = result[result_index++];

                // end of a result
                if (result_index == result_count)
                    result_count = 0;

                return r;
            }

            return 0xFF;
        }
    }
}

__force_inline static void i8272_writeport(const uint16_t port_number, const uint8_t data) {
    switch (port_number) {
        case FDC_PORT_DOR: // floppy digital output
        {
            const auto dor_changed_bits = DOR ^ data;

            if ((dor_changed_bits & (1 << 2)) && (data & (1 << 2))) {
                i8272_irq();

                check_drives_mask = 0xF; // all of them
            }
            DOR = data;
            break;
        }

        case FDC_PORT_DATA: // floppy command/data
        {
            if (command_length == 0) {
                command[0] = data;
                auto const cmd = data & 0x1F;
                // printf("FCD = %02X\n", data);
                if (cmd == FDC_CMD_SPECIFY) // specify
                    command_length = 3;
                else if (cmd == FDC_CMD_SENSE_DRIVE_STATUS) // sense drive status
                    command_length = 2;
                else if (cmd == FDC_CMD_WRITE_DATA) // write
                    command_length = 9;
                else if (cmd == FDC_CMD_READ_DATA) // read
                    command_length = 9;
                else if (cmd == FDC_CMD_RECALIBRATE) // recalibrate
                    command_length = 2;
                else if (cmd == FDC_CMD_SENSE_INTERRUPT_STATUS) // sense interrupt status
                    command_length = 1;
                else if (cmd == FDC_CMD_READ_ID) // read id
                    command_length = 2;
                else if (cmd == FDC_CMD_SEEK)
                    command_length = 3;
                else {
                    // return an error
                    response[0] = 1 << 7; // invalid command
                    result_count = 1;
                    result[0] = response[0];
                    result_index = 0;
                }

                if (command_length)
                    command_index = 1;
            } else
                command[command_index++] = data;

            if (command_length && command_index == command_length) {
                // got full command
                auto const cmd = command[0] & 0x1F;
                if (cmd == FDC_CMD_READ_DATA) // read
                {
                    // multitrack mfm skip
                    [[maybe_unused]] bool multiTrack = command[0] & (1 << 7);
                    [[maybe_unused]] bool mfm = command[0] & (1 << 6);
                    // bool skipDeleted = command[0] & (1 << 5);

                    const int drive = command[1] & 3;
                    const int head = (command[1] >> 2) & 1;

                    const auto cylinder = command[2];
                    [[maybe_unused]] auto headAgain = command[3];
                    const auto sector = command[4];
                    const auto number = command[5];
                    const auto endOfTrack = command[6];
                    //auto gapLength = command[7];
                    //auto dataLength = command[8];

                    assert(head == headAgain);
                    assert(number == 2);
                    assert(multiTrack);
                    assert(mfm);

                    //auto sectorSize = 128 << number;

                    // У нас только одна дискетка
                    const bool failed = drive > 0;

                    response[0] = drive | head << 2;

                    if (failed)
                        response[0] |= 1 << 6;

                    result_count = 7;
                    result[0] = response[0];
                    result[1] = response[1];
                    result[2] = response[2];
                    result[3] = cylinder;
                    result[4] = head;
                    result[5] = sector;
                    result[6] = number;

                    // printf("FDC: read sector %d\n", record); //
                    if (!failed) {
                        // Запускаем асинхронную DMA передачу на канале 2
                        // IRQ6 будет сгенерирован автоматически при завершении передачи
                        // const uint32_t size = endOfTrack * FDD_SECTOR_SIZE;
                        const uint32_t offset = ((cylinder * FDD_HEADS + head) * FDD_SECTORS_PER_TRACK + (sector - 1)) * FDD_SECTOR_SIZE;
                        dma_start_transfer(2, &FDD360[offset], 6);
                    } else {
                        // При ошибке генерируем IRQ сразу
                        i8272_irq();
                    }
                } else if (cmd == FDC_CMD_SENSE_INTERRUPT_STATUS) // sense interrupt status
                {
                    if (check_drives_mask) {
                        const int drive = __builtin_ctz(check_drives_mask);
                        check_drives_mask &= check_drives_mask - 1; // clear bit
                        result[0] = 0xC0 | drive;
                    } else {
                        result[0] = response[0];
                    }

                    result[1] = presentCylinder[response[0] & 3];
                    result_count = 2;
                } else if (cmd == FDC_CMD_SEEK) // seek
                {
                    const int drive = command[1] & 3;
                    // int head = (command[1] >> 2) & 1;
                    const auto cylinder = command[2];

                    response[0] = drive;
                    if (drive > 0)
                        response[0] |= 1 << 6 | 1 << 4; // abnormal termination/equipment check
                    else {
                        presentCylinder[drive] = cylinder;

                        // set seek end
                        response[0] |= 1 << 5;
                    }

                    i8272_irq();
                } else if (cmd == FDC_CMD_WRITE_DATA) // write
                {
                    [[maybe_unused]] bool multiTrack = command[0] & (1 << 7);
                    [[maybe_unused]] bool mfm = command[0] & (1 << 6);
                    // bool skipDeleted = command[0] & (1 << 5);

                    int unit = command[1] & 3;
                    int head = (command[1] >> 2) & 1;

                    auto cylinder = command[2];
                    [[maybe_unused]] auto headAgain = command[3];
                    auto record = command[4];
                    [[maybe_unused]] auto number = command[5];
                    //auto endOfTrack = command[6];
                    //auto gapLength = command[7];
                    //auto dataLength = command[8];

                    assert(head == headAgain);
                    assert(number == 2);
                    assert(multiTrack);
                    assert(mfm);

                    // prepare for write
                    bool failed = false;
                    /* TODO: Проверять доступ к другой дискетке
                    if(!io)
                        failed = true;
*/
                    response[0] = unit | head << 2;

                    if (failed)
                        response[0] |= 1 << 6;

                    result_count = 7;
                    result[0] = response[0];
                    result[1] = response[1];
                    result[2] = response[2];
                    result[3] = cylinder;
                    result[4] = head;
                    result[5] = record;
                    result[6] = number;

                    if (!failed) {
                        // start DMA if we didn't immediately fail
                        // FIXME START DMA
                        //sys.getChipset().dmaRequest(2, true, this);
                    } else {
                        i8272_irq();
                    }
                } else if (cmd == FDC_CMD_SPECIFY) // specify
                {
                    // auto stepRateTime = command[1] >> 4;
                    // auto headUnloadTime = command[1] & 0xF;
                    // auto headLoadTime = command[2] >> 1;
                    // bool nonDMA = command[2] & 1;
                } else if (cmd == FDC_CMD_SENSE_DRIVE_STATUS) // sense drive status
                {
                    const int drive = command[1] & 3;
                    // int head = (command[1] >> 2) & 1;

                    const bool isTrack_0 = presentCylinder[drive] == 0;

                    result_count = 1;
                    result[0] = (isTrack_0 ? 1 << 4 : 0) | 1 << 5 /*ready*/;
                } else if (cmd == FDC_CMD_RECALIBRATE) // recalibrate
                {
                    const int drive = command[1] & 3;

                    response[0] = drive;
                    if (drive > 0)
                        response[0] |= 1 << 6 | 1 << 4; // abnormal termination/equipment check
                    else {
                        presentCylinder[drive] = 0;
                        response[0] |= 1 << 5; // set seek end
                    }

                    i8272_irq();
                } else if (cmd == FDC_CMD_READ_ID) // read id
                {
                    [[maybe_unused]] bool mfm = command[0] & (1 << 6);

                    int drive = command[1] & 3;
                    int head = (command[1] >> 2) & 1;

                    assert(mfm);

                    response[0] = drive | head << 2;

                    result_count = 7;
                    result[0] = response[0];
                    result[1] = response[1];
                    result[2] = response[2];
                    result[3] = presentCylinder[drive];
                    result[4] = head;
                    result[5] = 1;
                    result[6] = 2; // ?

                    i8272_irq();
                }


                command_length = 0;
                result_index = 0;
            }

            break;
        }
    }
}

#if defined(DEBUG_I8272)
#undef debug_log
#endif
