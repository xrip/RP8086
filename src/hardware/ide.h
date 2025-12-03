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
constexpr uint8_t heads = 16;
constexpr uint8_t sectors = 63;

// Вспомогательные inline-функции — чтобы убрать дублирование
static inline void set_status(const uint8_t flags) {
    ide.regs[IDE_REG_STATUS] = flags;
}

static inline void add_status(const uint8_t flags) {
    ide.regs[IDE_REG_STATUS] |= flags;
}

static inline void clear_status(const uint8_t flags) {
    ide.regs[IDE_REG_STATUS] &= ~flags;
}

static inline void set_error(const uint8_t error) {
    ide.regs[IDE_REG_ERROR] = error;
    add_status(IDE_SR_ERR);
}

static inline void set_ready() {
    ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
}

// Магия PCem для строк: меняет порядок байт и заполняет пробелами
static void ide_padstr(uint8_t *dst, const char *src, int len) {
    for (int i = 0; i < len; i++) {
        dst[i ^ 1] = *src ? *src++ : ' '; // XOR 1 меняет Endianness (0->1, 1->0)
    }
}

static uint32_t ide_get_lba() {
    if (ide.regs[IDE_REG_DEVICE] & 0x40) {
        uint32_t lba = ide.regs[IDE_REG_LBA_LOW];
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_MID] << 8);
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_HIGH] << 16);
        lba |= ((uint32_t) (ide.regs[IDE_REG_DEVICE] & 0x0F) << 24);
        return lba;
    }


    const uint8_t head = ide.regs[IDE_REG_DEVICE] & 0x0F;
    const uint8_t sector = ide.regs[IDE_REG_LBA_LOW];

    const uint16_t cylinder = ide.regs[IDE_REG_LBA_MID] | ((uint16_t) ide.regs[IDE_REG_LBA_HIGH] << 8);

    return ((uint32_t) cylinder * heads + head) * sectors + (sector - 1);
}

// Увеличить LBA регистры (low->mid->high), учесть carry
static void increment_lba_regs() {
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
static int load_sector_from_disk(const uint32_t LBA) {
    if (!ide.disk_image) return -1;
    if (f_lseek(ide.disk_image, LBA * 512) != FR_OK) return -2;
    UINT bytes_read = 0;
    if (f_read(ide.disk_image, ide.sector_buffer, 512, &bytes_read) != FR_OK) return -3;
    if (bytes_read < 512) memset(ide.sector_buffer + bytes_read, 0, 512 - bytes_read);
    return 0;
}

// Аналогично — записать буфер на диск (используется при завершении записи одного сектора)
static int write_sector_to_disk(const uint32_t LBA) {
    if (!ide.disk_image) return -1;
    if (f_lseek(ide.disk_image, LBA * 512) != FR_OK) return -2;
    UINT bytes_writen = 0;
    if (f_write(ide.disk_image, ide.sector_buffer, 512, &bytes_writen) != FR_OK || bytes_writen != 512) return -3;
    // А надо ли?
    // if (f_sync(ide.disk_image) != FR_OK) return -4;
    return 0;
}

// --- Реализация команд (упрощённо) ---
static void cmd_identify() {
    memset(ide.sector_buffer, 0, 512);
    // uint16_t *wbuf = (uint16_t *) ide.sector_buffer;

    uint32_t total_sectors = 0;
    if (ide.disk_image) total_sectors = (uint32_t) (f_size(ide.disk_image) / 512);

    uint32_t cylinders = (total_sectors / (heads * sectors));
    if (cylinders < 1) cylinders = 1;
    if (cylinders > 16383) cylinders = 16383;

    total_sectors = cylinders * heads * sectors;
/*
    wbuf[0] = (1 << 6); // 0x40
    wbuf[1] = (uint16_t) cylinders;
    wbuf[3] = heads;
    wbuf[6] = sectors;
*/
    ide.sector_buffer[0] = 0x40;
    ide.sector_buffer[1] = cylinders << 8;
    ide.sector_buffer[2] = cylinders & 0xFF;
    ide.sector_buffer[6] = heads;
    ide.sector_buffer[12] = sectors;

    ide_padstr(&ide.sector_buffer[20], "", 10);
    ide_padstr(&ide.sector_buffer[46], "v1.0", 4);
    ide_padstr(&ide.sector_buffer[54], "SD:\\XT\\HDD.IMG", 20);


    /*wbuf[20] = 3;
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
    wbuf[80] = 0x0E;*/

    // Данные готовы к чтению
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

static void prepare_read_sector(void) {
    if (!ide.disk_image) {
        set_error(0x04); // ABRT? Оставляем как в оригинале — но возможно 0x04 у тебя был ABRT
        set_ready();
        return;
    }

    const uint32_t LBA = ide_get_lba();
    const int res = load_sector_from_disk(LBA);
    if (res != 0) {
        debug_log("IDE: Read error LBA=%lu ret=%d\n", LBA, ret);
        set_error(0x40); // UNC / generic read error (как в оригинале)
        set_ready();
        return;
    }

    const uint8_t sectors_count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (sectors_count == 0) ? 1 : sectors_count;

    debug_log("IDE: cmd_read_sector - sectors_remaining=%u LBA=%lu\n", ide.sectors_remaining, lba);
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

static void prepare_write_sector(void) {
    const uint8_t sectors_count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (sectors_count == 0) ? 1 : sectors_count;
    debug_log("IDE: cmd_write_sector - sectors_remaining=%u\n", ide.sectors_remaining);
    set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
}

// READ VERIFY (без данных)
static void handle_read_verify() {
    const uint32_t LBA = ide_get_lba();
    const uint8_t sectors_count = ide.regs[IDE_REG_SEC_COUNT];
    const uint16_t total_sectors = (sectors_count == 0) ? 256 : sectors_count;

    debug_log("IDE: READ VERIFY LBA=%lu Count=%u\n", lba, count);

    if (ide.disk_image) {
        const uint32_t max_lba = (uint32_t) (f_size(ide.disk_image) / 512);
        if (LBA + total_sectors > max_lba) {
            debug_log("IDE: READ VERIFY out of bounds! LBA=%lu+%u > %lu\n", lba, count, max_lba);
            set_error(0x10); // IDNF
            set_ready();
            return;
        }
    }
    set_ready();
}

// --- Главный обработчик команд ---
__force_inline static void handle_command(const uint8_t cmd) {
    ide.current_command = cmd;
    ide.buffer_index = 0;

    // По умолчанию показываем BSY, а потом команда установит нужный статус
    set_status(IDE_SR_BSY);

    switch (cmd) {
        case 0xEC:
            cmd_identify();
            break;

        case 0x20:
            prepare_read_sector();
            break;

        case 0x30:
            prepare_write_sector();
            break;

        case 0x40:
        case 0x41:
            handle_read_verify();
            break;

        case 0x91:
        case 0xEF:
        case 0x70:
        case 0x10:
            set_ready();
            break;

        default:
            printf("IDE: Unknown command 0x%02X - ABORTED\n", cmd);
            set_error(0x04);
            set_ready();
            break;
    }
}

__force_inline static uint8_t ide_read(uint16_t port_number) {
    port_number &= 0xf;

    switch (port_number) {
        case IDE_REG_DATA: {
            const uint16_t word = *(uint16_t *) &ide.sector_buffer[ide.buffer_index];
            ide.buffer_index += 2;
            ide.temp_high_byte = (uint8_t) (word >> 8);

            if (ide.buffer_index >= 512) {
                ide.buffer_index = 0;

                if (ide.current_command == 0x20) {
                    // READ SECTORS
                    if (ide.sectors_remaining > 0) ide.sectors_remaining--;
                    debug_log("IDE: Multi-sector read - sectors_remaining=%u\n", ide.sectors_remaining);

                    if (ide.sectors_remaining > 0) {
                        increment_lba_regs();
                        const uint32_t LBA = ide_get_lba();
                        if (load_sector_from_disk(LBA) == 0) {
                            set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
                        } else {
                            debug_log("IDE: Multi-sector read error at LBA=%lu\n", lba);
                            set_error(0x10);
                            set_ready();
                        }
                    } else {
                        debug_log("IDE: Multi-sector read complete\n");
                        set_ready();
                    }
                } else {
                    // Для других команд просто снять DRQ
                    set_ready();
                }
            }
            return (uint8_t) (word & 0xff);
        }
        case 0x8: return ide.temp_high_byte;
        default:
            return ide.regs[port_number];
    }
}

__force_inline static void ide_write(uint16_t port_number, const uint8_t data) {
    port_number &= 0xf;

    switch (port_number) {
        case IDE_REG_DATA: {
            *(uint16_t *) &ide.sector_buffer[ide.buffer_index] = data | (ide.temp_high_byte << 8);
            ide.buffer_index += 2;

            if (ide.buffer_index >= 512) {
                const uint32_t LBA = ide_get_lba();
                debug_log("IDE: Buffer write complete - flushing to disk LBA=%lu\n", LBA);
                ide.buffer_index = 0;

                if (ide.current_command == 0x30) {
                    int ret = write_sector_to_disk(LBA);
                    if (ret != 0) {
                        debug_log("IDE: flush_buffer write error ret=%d\n", ret);
                        set_error(0x40);
                        set_ready();
                        return;
                    }

                    if (ide.sectors_remaining > 0) ide.sectors_remaining--;
                    debug_log("IDE: Multi-sector write - sectors_remaining=%u\n", ide.sectors_remaining);

                    if (ide.sectors_remaining > 0) {
                        increment_lba_regs();
                        set_status(IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC);
                    } else {
                        debug_log("IDE: Multi-sector write complete\n");
                        set_ready();
                    }
                } else {
                    clear_status(IDE_SR_DRQ);
                    add_status(IDE_SR_RDY | IDE_SR_DSC);
                }
            }
            return;
        }
        case 0x8: {
            ide.temp_high_byte = data;
            return;
        }
        case IDE_REG_COMMAND:
            handle_command(data);
            return;
        case IDE_REG_DEVICE:
            if (ide.disk_image)
                set_ready();
        default:
            ide.regs[port_number] = data;
    }
}