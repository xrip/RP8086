#include <string.h>

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

extern ide_s ide;
uint8_t ata_swap[20];

// Магия PCem для строк: меняет порядок байт и заполняет пробелами
static void ide_padstr(uint8_t *dst, const char *src, int len) {
    for (int i = 0; i < len; i++) {
        char val = (*src) ? *src++ : ' ';
        dst[i ^ 1] = val; // XOR 1 меняет Endianness (0->1, 1->0)
    }
}

static uint32_t ide_get_lba() {
    // Проверяем режим: LBA (bit 6 = 1) или CHS (bit 6 = 0)
    if (ide.regs[IDE_REG_DEVICE] & 0x40) {
        // LBA режим
        uint32_t lba = ide.regs[IDE_REG_LBA_LOW];
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_MID] << 8);
        lba |= ((uint32_t) ide.regs[IDE_REG_LBA_HIGH] << 16);
        lba |= ((uint32_t) (ide.regs[IDE_REG_DEVICE] & 0x0F) << 24);
        return lba;
    } else {
        // CHS режим: конвертируем в LBA
        // В CHS: LBA_LOW=Sector (1-based), LBA_MID/HIGH=Cylinder, DEVICE[0:3]=Head
        uint16_t cylinder = ide.regs[IDE_REG_LBA_MID] | ((uint16_t)ide.regs[IDE_REG_LBA_HIGH] << 8);
        uint8_t head = ide.regs[IDE_REG_DEVICE] & 0x0F;
        uint8_t sector = ide.regs[IDE_REG_LBA_LOW];

        // Наша геометрия: 16 heads, 63 sectors (стандартная IDE геометрия)
        const uint8_t heads = 16;
        const uint8_t sectors = 63;

        // CHS to LBA: LBA = (C × heads + H) × sectors + (S - 1)
        uint32_t lba = ((uint32_t)cylinder * heads + head) * sectors + (sector - 1);
        return lba;
    }
}

// --- Реализация команд ---

static void cmd_identify() {
    memset(ide.sector_buffer, 0, 512);
    uint16_t *wbuf = (uint16_t *) ide.sector_buffer;
    uint8_t *bbuf = (uint8_t *) ide.sector_buffer;

    // 1. Расчет размера диска
    uint32_t total_sectors = 0;
    if (ide.disk_image) {
        // Получаем размер файла в секторах
        uint32_t file_size = (uint32_t) f_size(ide.disk_image);
        total_sectors = file_size / 512;
        debug_log("IDE: Disk image size = %lu bytes (%lu sectors)\n", file_size, total_sectors);
    }
    // ЗАЩИТА ОТ ERROR 1h: Диск не может быть нулем или слишком маленьким
    // Используем фиксированную геометрию: 1024 × 16 × 17 = 278528 sectors
    if (total_sectors < 278528) {
        debug_log("IDE: File too small, using fixed geometry: 278528 sectors\n");
        total_sectors = 278528;
    }

    // 2. Геометрия CHS (стандартная IDE геометрия)
    const uint8_t heads = 16;          // Стандарт
    const uint8_t sectors = 63;        // IDE стандарт (НЕ 17 как в MFM!)

    // Вычисляем cylinders из total_sectors
    uint32_t cylinders = total_sectors / (heads * sectors);
    if (cylinders > 16383) cylinders = 16383; // Максимум для CHS
    if (cylinders < 1) cylinders = 1;

    // ВАЖНО: Пересчитываем total_sectors чтобы соответствовать CHS геометрии
    // FDISK проверяет что Cyl × Head × Sec = Total Sectors!
    total_sectors = (uint32_t)cylinders * heads * sectors;

    wbuf[1] = cylinders;
    wbuf[3] = heads;
    wbuf[6] = sectors;

    // 3. Строки (Serial, Firmware, Model)
    ide_padstr(&ide.sector_buffer[20], "", 10);          // Word 10-19: Serial (20 bytes)
    ide_padstr(&ide.sector_buffer[46], "v1.0", 4);       // Word 23-26: Firmware (8 bytes)
    ide_padstr(&ide.sector_buffer[54], "RP8086 HDD      ", 20); // Word 27-46: Model (40 bytes)

    // 4. Флаги и параметры (как в PCem)
    wbuf[0] = (1 << 6);                // Fixed Device
    wbuf[20] = 3;                      // Buffer type
    wbuf[21] = 512;                    // Buffer size (512 bytes)
    wbuf[47] = 16;                     // Max sectors on multiple transfer
    wbuf[48] = 1;                      // Dword transfers supported
    wbuf[49] = (1 << 9) | (1 << 8);    // LBA and DMA supported
    wbuf[50] = 0x4000;                 // Capabilities
    wbuf[51] = 2 << 8;                 // PIO timing mode
    wbuf[52] = 2 << 8;                 // DMA timing mode
    wbuf[53] = 0x0002;                 // Validity
    wbuf[59] = 16 | 0x100;             // Multiple sectors valid (16 sectors)
    wbuf[60] = total_sectors & 0xFFFF; // Total addressable sectors (LBA) low
    wbuf[61] = (total_sectors >> 16) & 0xFFFF; // Total addressable sectors (LBA) high
    wbuf[63] = 7;                      // Multiword DMA modes
    wbuf[80] = 0x0E;                   // ATA-1 to ATA-3 supported

    // Статус: Готов передавать данные
    ide.regs[IDE_REG_STATUS] = IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC;
}

static void cmd_read_sector() {
    // Сразу ставим BSY, чтобы показать работу
    ide.regs[IDE_REG_STATUS] = IDE_SR_BSY;

    if (!ide.disk_image) {
        debug_log("IDE: No disk image!\n");
        ide.regs[IDE_REG_ERROR] = 0x04; // ABRT
        ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
        return;
    }

    uint32_t lba = ide_get_lba();

    if (f_lseek(ide.disk_image, lba * 512) != FR_OK) {
        debug_log("IDE: Seek error LBA=%lu\n", lba);
        ide.regs[IDE_REG_ERROR] = 0x10; // IDNF
        ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
        return;
    }

    UINT read_cnt = 0; // ВАЖНО: Тип UINT для FatFs
    FRESULT res = f_read(ide.disk_image, ide.sector_buffer, 512, &read_cnt);

    if (res != FR_OK) {
        // Ошибка чтения
        debug_log("IDE: Read error res=%d cnt=%u\n", res, read_cnt);
        ide.regs[IDE_REG_ERROR] = 0x40; // UNC
        ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
        return;
    }

    // Если прочитали хвост, добить нулями
    if (read_cnt < 512) {
        memset(ide.sector_buffer + read_cnt, 0, 512 - read_cnt);
    }

    // Логируем только важные секторы (MBR и boot sector)
    if (lba == 0 || lba == 1 || lba == 17) {
        debug_log("IDE: Read OK, first 16 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
               ide.sector_buffer[0], ide.sector_buffer[1], ide.sector_buffer[2], ide.sector_buffer[3],
               ide.sector_buffer[4], ide.sector_buffer[5], ide.sector_buffer[6], ide.sector_buffer[7],
               ide.sector_buffer[8], ide.sector_buffer[9], ide.sector_buffer[10], ide.sector_buffer[11],
               ide.sector_buffer[12], ide.sector_buffer[13], ide.sector_buffer[14], ide.sector_buffer[15]);
    }

    // Инициализируем счетчик секторов для multi-sector read
    uint8_t count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (count == 0) ? 1 : count;

    debug_log("IDE: cmd_read_sector - sectors_remaining=%u\n", ide.sectors_remaining);

    // УСПЕХ: Данные готовы
    ide.regs[IDE_REG_STATUS] = IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC;
}

static void cmd_write_sector() {
    // Для записи мы просто ждем, пока CPU наполнит буфер
    // DRQ = готов принимать данные
    ide.regs[IDE_REG_STATUS] = IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC;

    // Инициализируем счетчик секторов для multi-sector write
    uint8_t count = ide.regs[IDE_REG_SEC_COUNT];
    ide.sectors_remaining = (count == 0) ? 1 : count; // 0 обычно означает 1 сектор

    debug_log("IDE: cmd_write_sector - sectors_remaining=%u\n", ide.sectors_remaining);
}

static void flush_buffer() {
    if (!ide.disk_image) {
        printf("IDE: flush_buffer - no disk image!\n"); // Критическая ошибка
        return;
    }

    uint32_t lba = ide_get_lba();
    debug_log("IDE: flush_buffer LBA=%lu\n", lba);

    FRESULT res = f_lseek(ide.disk_image, lba * 512);
    if (res != FR_OK) {
        printf("IDE: flush_buffer seek error res=%d\n", res); // Критическая ошибка
        return;
    }

    UINT bw = 0;
    res = f_write(ide.disk_image, ide.sector_buffer, 512, &bw);
    if (res != FR_OK || bw != 512) {
        printf("IDE: flush_buffer write error res=%d bw=%u\n", res, bw); // Критическая ошибка
        return;
    }

    res = f_sync(ide.disk_image);
    if (res != FR_OK) {
        printf("IDE: flush_buffer sync error res=%d\n", res); // Критическая ошибка
        return;
    }

    debug_log("IDE: flush_buffer OK - wrote %u bytes\n", bw);
}

// --- Главный обработчик команд ---

void handle_command(uint8_t cmd) {
    ide.current_command = cmd;
    ide.buffer_index = 0;
    ide.latch_flag = false;

    // Ставим BSY по умолчанию
    ide.regs[IDE_REG_STATUS] = IDE_SR_BSY;

    switch (cmd) {
        case 0xEC: {
            cmd_identify();
            // Логируем важные поля IDENTIFY
            uint16_t *wbuf = (uint16_t *) ide.sector_buffer;
            debug_log("IDE: IDENTIFY - Cyl=%u Head=%u Sec=%u TotalSec=%lu\n",
                   wbuf[1], wbuf[3], wbuf[6],
                   (uint32_t)wbuf[60] | ((uint32_t)wbuf[61] << 16));
            break;
        }
        case 0x20: {
            static uint32_t read_count = 0;
            uint32_t lba = ide_get_lba();

            // Логируем только каждую 1000-ю операцию или важные секторы
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
            cmd_read_sector();
            break;
        }
        case 0x30: {
            uint32_t lba = ide_get_lba();
            uint8_t count = ide.regs[IDE_REG_SEC_COUNT];
            if (count == 0) count = 1; // 0 обычно означает 1 сектор для WRITE
            debug_log("IDE: WRITE LBA=%lu Count=%u\n", lba, count);
            cmd_write_sector();
            break;
        }

        case 0x40: // READ VERIFY SECTORS (без передачи данных)
        case 0x41: { // READ VERIFY SECTORS (с retry)
            uint32_t lba = ide_get_lba();
            uint8_t count_reg = ide.regs[IDE_REG_SEC_COUNT];
            uint16_t count = (count_reg == 0) ? 256 : count_reg; // 0 означает 256 секторов

            debug_log("IDE: READ VERIFY LBA=%lu Count=%u\n", lba, count);

            // Проверяем что сектор в пределах диска
            // Получаем размер из disk_image
            uint32_t max_lba = 0;
            if (ide.disk_image) {
                max_lba = (uint32_t)(f_size(ide.disk_image) / 512);
            }
            if (max_lba == 0) max_lba = 277776; // Fallback на геометрию

            if (lba + count > max_lba) {
                debug_log("IDE: READ VERIFY out of bounds! LBA=%lu+%u > %lu\n", lba, count, max_lba);
                ide.regs[IDE_REG_ERROR] = 0x10; // IDNF (ID Not Found)
                ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
            } else {
                // Сектор существует - возвращаем успех (без чтения данных!)
                ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
            }
            break;
        }

        case 0x91: // Init Params
            debug_log("IDE: Init Drive Params (0x91) - Head=%u Sec=%u\n",
                   ide.regs[IDE_REG_DEVICE] & 0x0F, ide.regs[IDE_REG_SEC_COUNT]);
            ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
            break;

        case 0xEF: { // Set Features
            uint8_t feature = ide.regs[IDE_REG_FEATURES];
            debug_log("IDE: Set Features (0xEF) - Feature=0x%02X\n", feature);

            // XT-IDE не поддерживает современные фичи, но притворяемся что поддерживаем
            // чтобы FORMAT не переключился в interrupt mode
            // Просто игнорируем все фичи и возвращаем SUCCESS
            debug_log("IDE: Set Features 0x%02X - ignored (fake success)\n", feature);
            ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
            break;
        }

        case 0x70: // Seek
        case 0x10: // Recalibrate
            debug_log("IDE: Seek/Recalibrate (0x%02X)\n", cmd);
            ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
            break;

        default:
            // Неизвестная команда -> ABRT
            printf("IDE: Unknown command 0x%02X - ABORTED\n", cmd); // Критическая ошибка - оставляем printf
            ide.regs[IDE_REG_ERROR] = 0x04;
            ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
            break;
    }
}

// --- IO HANDLERS ---

void ide_reset() {
    memset(ide.regs, 0, 8);
    // Начальное состояние: Готов и Головка на месте
    ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
    ide.regs[IDE_REG_ERROR] = 0x01;
    ide.latch_flag = false;
}

uint8_t ide_read(uint16_t port) {
    port &= 0xf; // Mask 0..7

    if (port == IDE_REG_DATA) {
        const uint16_t word = *(uint16_t *) &ide.sector_buffer[ide.buffer_index];
        ide.buffer_index += 2;
        ide.temp_high_byte = word >> 8;

        if (ide.buffer_index >= 512) {
            ide.buffer_index = 0;

            if (ide.current_command == 0x20) { // READ SECTORS
                // Декрементируем счетчик секторов
                if (ide.sectors_remaining > 0) {
                    ide.sectors_remaining--;
                    debug_log("IDE: Multi-sector read - sectors_remaining=%u\n", ide.sectors_remaining);
                }

                // Если еще есть секторы - загружаем следующий
                if (ide.sectors_remaining > 0) {
                    debug_log("IDE: Loading next sector for multi-sector read\n");

                    // Увеличиваем LBA для следующего сектора
                    ide.regs[IDE_REG_LBA_LOW]++;
                    if (ide.regs[IDE_REG_LBA_LOW] == 0) {
                        ide.regs[IDE_REG_LBA_MID]++;
                        if (ide.regs[IDE_REG_LBA_MID] == 0) {
                            ide.regs[IDE_REG_LBA_HIGH]++;
                        }
                    }

                    // Читаем следующий сектор (копируем логику из cmd_read_sector)
                    uint32_t lba = ide_get_lba();
                    if (ide.disk_image && f_lseek(ide.disk_image, lba * 512) == FR_OK) {
                        UINT read_cnt = 0;
                        f_read(ide.disk_image, ide.sector_buffer, 512, &read_cnt);
                        if (read_cnt < 512) memset(ide.sector_buffer + read_cnt, 0, 512 - read_cnt);
                        // Оставляем DRQ=1 для чтения следующего сектора
                        ide.regs[IDE_REG_STATUS] = IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC;
                    } else {
                        // Ошибка чтения - сбрасываем DRQ
                        debug_log("IDE: Multi-sector read error at LBA=%lu\n", lba);
                        ide.regs[IDE_REG_STATUS] = IDE_SR_ERR | IDE_SR_RDY | IDE_SR_DSC;
                    }
                } else {
                    // Все секторы прочитаны - сбрасываем DRQ
                    debug_log("IDE: Multi-sector read complete\n");
                    ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
                }
            } else {
                // Не READ команда - просто сбрасываем DRQ
                ide.regs[IDE_REG_STATUS] &= ~IDE_SR_DRQ;
                ide.regs[IDE_REG_STATUS] |= (IDE_SR_RDY | IDE_SR_DSC);
            }
        }
        return word & 0xff;
    }

    if (port == 8) {
        return ide.temp_high_byte;
    }

    // Логируем чтение STATUS для отладки
    if (port == IDE_REG_STATUS) {
        static uint32_t status_read_count = 0;

        // Логируем первые 10 чтений после каждой команды + каждое 10000-е
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

    // Чтение остальных регистров
    return ide.regs[port];
}

void ide_write(uint16_t port, uint8_t data) {
    port &= 0xf;

    if (port == IDE_REG_DATA) {
            *(uint16_t *)&ide.sector_buffer[ide.buffer_index] = data | (ide.temp_high_byte << 8);
            ide.buffer_index += 2;

            // Debug: первые 4 записи
            if (ide.buffer_index <= 8) {
                debug_log("IDE: Write DATA[%d] = %02X (high=%02X)\n", ide.buffer_index - 2, data, ide.temp_high_byte);
            }

            if (ide.buffer_index >= 512) {
                debug_log("IDE: Buffer write complete - flushing to disk LBA=%lu\n", ide_get_lba());
                ide.buffer_index = 0;

                if (ide.current_command == 0x30) {
                    flush_buffer();

                    // Декрементируем счетчик секторов
                    if (ide.sectors_remaining > 0) {
                        ide.sectors_remaining--;
                        debug_log("IDE: Multi-sector write - sectors_remaining=%u\n", ide.sectors_remaining);
                    }

                    // Если еще есть секторы - выставляем DRQ снова
                    if (ide.sectors_remaining > 0) {
                        debug_log("IDE: Setting DRQ for next sector\n");
                        ide.regs[IDE_REG_STATUS] = IDE_SR_DRQ | IDE_SR_RDY | IDE_SR_DSC;
                        // Увеличиваем LBA для следующего сектора
                        ide.regs[IDE_REG_LBA_LOW]++;
                        if (ide.regs[IDE_REG_LBA_LOW] == 0) {
                            ide.regs[IDE_REG_LBA_MID]++;
                            if (ide.regs[IDE_REG_LBA_MID] == 0) {
                                ide.regs[IDE_REG_LBA_HIGH]++;
                            }
                        }
                    } else {
                        // Все секторы записаны - сбрасываем DRQ
                        debug_log("IDE: Multi-sector write complete\n");
                        ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
                    }
                } else {
                    // Не WRITE команда - просто сбрасываем DRQ
                    ide.regs[IDE_REG_STATUS] &= ~IDE_SR_DRQ;
                    ide.regs[IDE_REG_STATUS] |= (IDE_SR_RDY | IDE_SR_DSC);
                }
            }
        return; // ВЫХОДИМ, чтобы не писать в regs[]
    }

    if (port == 8) {
        ide.temp_high_byte = data;
        return;
    }

    if (port == IDE_REG_COMMAND) {
        // Запись команды
        static uint32_t cmd_count = 0;
        debug_log("IDE: Command #%lu = 0x%02X\n", cmd_count++, data);
        handle_command(data);
        return; // ВАЖНО: ВЫХОДИМ! Нельзя писать data в regs[7] (Status)
    }

    // Запись параметров (кроме Data и Command)
    // Все регистры кроме Command записываются напрямую
    ide.regs[port] = data;

    // Хак для XTIDE: при выборе устройства сбрасываем статус в Ready
    if (port == IDE_REG_DEVICE) {
        ide.regs[IDE_REG_STATUS] = IDE_SR_RDY | IDE_SR_DSC;
    }
}
