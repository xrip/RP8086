#include <string.h>
#include <stdio.h> // для printf в отладке/ошибках (как и было)

// Отладочный вывод HDD
//#define DEBUG_HDD
#if defined(DEBUG_HDD)
#define debug_log(...) printf(__VA_ARGS__)
#else
#define debug_log(...)
#endif

// Регистры
#define IDE_REG_DATA        0
#define IDE_REG_ERROR       1 // Read
#define IDE_REG_FEATURES    1 // Write
#define IDE_REG_SEC_COUNT   2
#define IDE_REG_LBA_LOW     3
#define IDE_REG_LBA_MID     4
#define IDE_REG_LBA_HIGH    5
#define IDE_REG_DEVICE      6
#define IDE_REG_STATUS      7 // Read
#define IDE_REG_COMMAND     7 // Write

// Флаги Статуса
#define IDE_SR_ERR  0x01 // Error
#define IDE_SR_DRQ  0x08 // Data Request
#define IDE_SR_DSC  0x10 // Drive Seek Complete
#define IDE_SR_RDY  0x40 // Drive Ready
#define IDE_SR_BSY  0x80 // Busy

extern ide_s ide; // структура должна быть в другом модуле как и раньше

// Вспомогательные inline-функции — чтобы убрать дублирование
static inline void set_status(uint8_t flags) {
    ide.regs[IDE_REG_STATUS] = flags;
}
static inline void add_status(uint8_t flags) {
    ide.regs[IDE_REG_STATUS] |= flags;
}
static inline void clear_status(uint8_t flags) {
    ide.regs[IDE_REG_STATUS] &= ~flags;
}
static inline void set_error(uint8_t err) {
    ide.regs[IDE_REG_ERROR] = err;
    add_status(IDE_SR_ERR);
}
static inline void set_ready_no_err(void) {
    ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
}

// Магия PCem для строк: меняет порядок байт и заполняет пробелами
static void ide_padstr(uint8_t *dst, const char *src, int len) {
    for (int i = 0; i < len; i++) {
        char val = (*src) ? *src++ : ' ';
        dst[i ^ 1] = val; // XOR 1 меняет Endianness (0->1, 1->0)
    }
}

static uint32_t ide_get_lba() {
    if (ide.regs[IDE_REG_DEVICE] & 0x40) {
        uint32_t lba = ide.regs[IDE_REG_LBA_LOW];
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_MID] << 8);
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_HIGH] << 16);
        lba |= ((uint32_t) (ide.regs[IDE_REG_DEVICE] & 0x0F) << 24);
        return lba;
    } else {
        uint16_t cylinder = ide.regs[IDE_REG_LBA_MID] | ((uint16_t)ide.regs[IDE_REG_LBA_HIGH] << 8);
        uint8_t head = ide.regs[IDE_REG_DEVICE] & 0x0F;
        uint8_t sector = ide.regs[IDE_REG_LBA_LOW];
        const uint8_t heads = 16;
        const uint8_t sectors = 63;
        uint32_t lba = ((uint32_t)cylinder * heads + head) * sectors + (sector - 1);
        return lba;
    }
}

// Увеличить LBA регистры (low->mid->high), учесть carry
static void increment_lba_regs(void) {
    ide.regs[IDE_REG_LBA_LOW]++;
    if (ide.regs[IDE_REG_LBA_LOW] == 0) {
        ide.regs[IDE_REG_LBA_MID]++;
        if (ide.regs[IDE_REG_LBA_MID] == 0) {
            ide.regs[IDE_REG_LBA_HIGH]++;
            // DEVICE high nibble (LBA28) не трогаем здесь — оно изменяется только при необходимости
        }
    }
}

// Функция чтения сектора с диска в секторный буфер; возвращает 0 при успехе или код ошибки
static int load_sector_from_disk(uint32_t lba) {
    if (!ide.disk_image) return -1;
    if (f_lseek(ide.disk_image, lba * 512) != FR_OK) return -2;
    UINT read_cnt = 0;
    FRESULT res = f_read(ide.disk_image, ide.sector_buffer, 512, &read_cnt);
    if (res != FR_OK) return -3;
    if (read_cnt < 512) memset(ide.sector_buffer + read_cnt, 0, 512 - read_cnt);
    return 0;
}

// Аналогично — записать буфер на диск (используется при завершении записи одного сектора)
static int write_sector_to_disk(uint32_t lba) {
    if (!ide.disk_image) return -1;
    if (f_lseek(ide.disk_image, lba * 512) != FR_OK) return -2;
    UINT bw = 0;
    FRESULT res = f_write(ide.disk_image, ide.sector_buffer, 512, &bw);
    if (res != FR_OK || bw != 512) return -3;
    if (f_sync(ide.disk_image) != FR_OK) return -4;
    return 0;
}

// --- Реализация команд (упрощённо) ---
static void cmd_identify() {
    memset(ide.sector_buffer, 0, 512);
    uint16_t *wbuf = (uint16_t *) ide.sector_buffer;

    uint32_t total_sectors = 0;
    if (ide.disk_image) total_sectors = (uint32_t)(f_size(ide.disk_image) / 512);

    const uint8_t heads = 16;
    const uint8_t sectors = 63;
    uint32_t cylinders = (total_sectors / (heads * sectors));
    if (cylinders < 1) cylinders = 1;
    if (cylinders > 16383) cylinders = 16383;

    total_sectors = (uint32_t)cylinders * heads * sectors;

    wbuf[1] = (uint16_t)cylinders;
    wbuf[3] = heads;
    wbuf[6] = sectors;

    ide_padstr(&ide.sector_buffer[20], "", 10);
    ide_padstr(&ide.sector_buffer[46], "v1.0", 4);
    ide_padstr(&ide.sector_buffer[54], "RP8086 HDD      ", 20);

    wbuf[0] = (1 << 6);
    wbuf[20] = 3;
    wbuf[21] = 512;
    wbuf[47] = 16;
    wbuf[48] = 1;
    wbuf[49] = (1 << 9) | (1 << 8);
    wbuf[50] = 0x4000;
    wbuf[51] = 2 << 8;
    wbuf[52] = 2 << 8;
    wbuf[53] = 0x0002;
    wbuf[59] = 16 | 0x100;
    wbuf[60] = total_sectors & 0xFFFF;
    wbuf[61] = (total_sectors >> 16) & 0xFFFF;
    wbuf[63] = 7;
    wbuf[80] = 0x0E;

    // Данные готовы к чтению
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

static void prepare_read_sector(void) {
    if (!ide.disk_image) {
        set_error(0x04); // ABRT? Оставляем как в оригинале — но возможно 0x04 у тебя был ABRT
        set_ready_no_err();
        return;
    }

    uint32_t lba = ide_get_lba();
    int ret = load_sector_from_disk(lba);
    if (ret != 0) {
        debug_log("IDE: Read error LBA=%lu ret=%d\n", lba, ret);
        set_error(0x40); // UNC / generic read error (как в оригинале)
        set_ready_no_err();
        return;
    }

    uint8_t count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (count == 0) ? 1 : count;

    debug_log("IDE: cmd_read_sector - sectors_remaining=%u LBA=%lu\n", ide.sectors_remaining, lba);
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

static void prepare_write_sector(void) {
    uint8_t count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (count == 0) ? 1 : count;
    debug_log("IDE: cmd_write_sector - sectors_remaining=%u\n", ide.sectors_remaining);
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

// READ VERIFY (без данных)
static void handle_read_verify(void) {
    uint32_t lba = ide_get_lba();
    uint8_t count_reg = ide.regs[IDE_REG_SEC_COUNT];
    uint16_t count = (count_reg == 0) ? 256 : count_reg;

    debug_log("IDE: READ VERIFY LBA=%lu Count=%u\n", lba, count);

    if (ide.disk_image) {
        uint32_t max_lba = (uint32_t)(f_size(ide.disk_image) / 512);
        if (lba + count > max_lba) {
            debug_log("IDE: READ VERIFY out of bounds! LBA=%lu+%u > %lu\n", lba, count, max_lba);
            set_error(0x10); // IDNF
            set_ready_no_err();
            return;
        }
    }
    set_ready_no_err();
}

// --- Главный обработчик команд ---
void handle_command(uint8_t cmd) {
    ide.current_command = cmd;
    ide.buffer_index = 0;
    ide.latch_flag = false;

    // По умолчанию показываем BSY, а потом команда установит нужный статус
    set_status(IDE_SR_BSY);

    switch (cmd) {
        case 0xEC:
            cmd_identify();
            {
                uint16_t *wbuf = (uint16_t *) ide.sector_buffer;
                debug_log("IDE: IDENTIFY - Cyl=%u Head=%u Sec=%u TotalSec=%lu\n",
                          wbuf[1], wbuf[3], wbuf[6],
                          (uint32_t)wbuf[60] | ((uint32_t)wbuf[61] << 16));
            }
            break;

        case 0x20:
            {
                static uint32_t read_count = 0;
                uint32_t lba = ide_get_lba();
                if (lba < 100 || (read_count % 1000) == 0) {
                    if (ide.regs[IDE_REG_DEVICE] & 0x40) {
                        debug_log("IDE: READ #%lu LBA=%lu (LBA mode)\n", read_count, lba);
                    } else {
                        uint16_t cyl = ide.regs[IDE_REG_LBA_MID] | ((uint16_t)ide.regs[IDE_REG_LBA_HIGH] << 8);
                        uint8_t head = ide.regs[IDE_REG_DEVICE] & 0x0F;
                        uint8_t sector = ide.regs[IDE_REG_LBA_LOW];
                        debug_log("IDE: READ #%lu CHS=%u/%u/%u → LBA=%lu\n", read_count, cyl, head, sector, lba);
                    }
                }
                read_count++;
                prepare_read_sector();
            }
            break;

        case 0x30:
            debug_log("IDE: WRITE LBA=%lu Count=%u\n", ide_get_lba(),
                      ide.regs[IDE_REG_SEC_COUNT] ? ide.regs[IDE_REG_SEC_COUNT] : 1);
            prepare_write_sector();
            break;

        case 0x40:
        case 0x41:
            handle_read_verify();
            break;

        case 0x91:
            debug_log("IDE: Init Drive Params (0x91) - Head=%u Sec=%u\n",
                      ide.regs[IDE_REG_DEVICE] & 0x0F, ide.regs[IDE_REG_SEC_COUNT]);
            set_ready_no_err();
            break;

        case 0xEF:
            {
                uint8_t feature = ide.regs[IDE_REG_FEATURES];
                debug_log("IDE: Set Features (0xEF) - Feature=0x%02X (ignored)\n", feature);
                set_ready_no_err();
            }
            break;

        case 0x70:
        case 0x10:
            debug_log("IDE: Seek/Recalibrate (0x%02X)\n", cmd);
            set_ready_no_err();
            break;

        default:
            printf("IDE: Unknown command 0x%02X - ABORTED\n", cmd);
            set_error(0x04);
            set_ready_no_err();
            break;
    }
}

// --- IO HANDLERS (не менять имена) ---

void ide_reset() {
    memset(ide.regs, 0, 8);
    set_ready_no_err();
    ide.regs[IDE_REG_ERROR] = 0x01;
    ide.latch_flag = false;
}

uint8_t ide_read(uint16_t port) {
    port &= 0xf; // Mask 0..7

    if (port == IDE_REG_DATA) {
        const uint16_t word = *(uint16_t *) &ide.sector_buffer[ide.buffer_index];
        ide.buffer_index += 2;
        ide.temp_high_byte = (uint8_t)(word >> 8);

        if (ide.buffer_index >= 512) {
            ide.buffer_index = 0;

            if (ide.current_command == 0x20) { // READ SECTORS
                if (ide.sectors_remaining > 0) ide.sectors_remaining--;
                debug_log("IDE: Multi-sector read - sectors_remaining=%u\n", ide.sectors_remaining);

                if (ide.sectors_remaining > 0) {
                    increment_lba_regs();
                    uint32_t lba = ide_get_lba();
                    if (load_sector_from_disk(lba) == 0) {
                        set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
                    } else {
                        debug_log("IDE: Multi-sector read error at LBA=%lu\n", lba);
                        set_error(0x10);
                        set_ready_no_err();
                    }
                } else {
                    debug_log("IDE: Multi-sector read complete\n");
                    set_ready_no_err();
                }
            } else {
                // Для других команд просто снять DRQ
                set_ready_no_err();
            }
        }
        return (uint8_t)(word & 0xff);
    }

    if (port == 8) {
        return ide.temp_high_byte;
    }

    if (port == IDE_REG_STATUS) {
        static uint32_t status_read_count = 0;
        if (ide.buffer_index == 0 || (status_read_count % 10000) == 0) {
            debug_log("IDE: STATUS read #%lu = 0x%02X (DRQ=%d BSY=%d RDY=%d ERR=%d)\n",
                      status_read_count, ide.regs[port],
                      !!(ide.regs[port] & IDE_SR_DRQ),
                      !!(ide.regs[port] & IDE_SR_BSY),
                      !!(ide.regs[port] & IDE_SR_RDY),
                      !!(ide.regs[port] & IDE_SR_ERR));
        }
        status_read_count++;
    }

    return ide.regs[port];
}

void ide_write(uint16_t port, uint8_t data) {
    port &= 0xf;

    if (port == IDE_REG_DATA) {
        *(uint16_t *)&ide.sector_buffer[ide.buffer_index] = data | (ide.temp_high_byte << 8);
        ide.buffer_index += 2;

        if (ide.buffer_index <= 8) {
            debug_log("IDE: Write DATA[%d] = %02X (high=%02X)\n", ide.buffer_index - 2, data, ide.temp_high_byte);
        }

        if (ide.buffer_index >= 512) {
            uint32_t lba = ide_get_lba();
            debug_log("IDE: Buffer write complete - flushing to disk LBA=%lu\n", lba);
            ide.buffer_index = 0;

            if (ide.current_command == 0x30) {
                int ret = write_sector_to_disk(lba);
                if (ret != 0) {
                    debug_log("IDE: flush_buffer write error ret=%d\n", ret);
                    set_error(0x40);
                    set_ready_no_err();
                    return;
                }

                if (ide.sectors_remaining > 0) ide.sectors_remaining--;
                debug_log("IDE: Multi-sector write - sectors_remaining=%u\n", ide.sectors_remaining);

                if (ide.sectors_remaining > 0) {
                    increment_lba_regs();
                    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
                } else {
                    debug_log("IDE: Multi-sector write complete\n");
                    set_ready_no_err();
                }
            } else {
                clear_status(IDE_SR_DRQ);
                add_status(IDE_SR_RDY | IDE_SR_DSC);
            }
        }
        return;
    }

    if (port == 8) {
        ide.temp_high_byte = data;
        return;
    }

    if (port == IDE_REG_COMMAND) {
        static uint32_t cmd_count = 0;
        debug_log("IDE: Command #%lu = 0x%02X\n", cmd_count++, data);
        handle_command(data);
        return;
    }

    ide.regs[port] = data;

    // Хак для XTIDE: при выборе устройства сбрасываем статус в Ready
    if (port == IDE_REG_DEVICE) set_ready_no_err();
}
