#include "setup.h"
#include "graphics.h"
#include "common.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>  // Для malloc/free
#include <pico/time.h>  // Для sleep_us()
#include "../drivers/fatfs/ff.h"  // FatFS для работы с SD картой

#include "pico/stdio.h"
#ifndef DEBUG
#include "../drivers/usbhid/hid_app.h"  // Для keyboard_tick() в RELEASE режиме
#endif

// Глобальная структура настроек (инициализация по умолчанию)
settings_s settings = {
    .tandy_enabled = 0,  // NO
    .turbo = 1,          // ENABLED
    .fda = "/XT/fdd.img",
    .fdb = "",           // По умолчанию отключен
    .hdd = "/XT/hdd.img",
};

// Определение пунктов меню
static const MenuItem menu_items[] = {
    { "Features:", NONE, nullptr, nullptr, 0, .colors = {14, 1} },  // Желтый на синем
    { "  Turbo 6MHz:              %s", ARRAY, &settings.turbo, nullptr, 1, { "DISABLED", "ENABLED" }},
    { "  IBM PCjr/Tandy mode:     %s", ARRAY, &settings.tandy_enabled, nullptr, 1, { "NO", "YES" }},
    { "", NONE, nullptr, nullptr, 0, .colors = {7, 1} },  // Серый разделитель
    { "Storage devices:", NONE, nullptr, nullptr, 0, .colors = {14, 1} },  // Желтый на синем
    { "  Floppy #1:               %s", STRING, &settings.fda, nullptr, 255, {} },
    { "  Floppy #2:               %s", STRING, &settings.fdb, nullptr, 255, {} },
    { "  Hard drive:              %s", STRING, &settings.hdd, nullptr, 255, {} },
    { "", NONE, nullptr, nullptr, 0, .colors = {7, 1} },  // Серый разделитель
    { "Save Settings And Exit", EXIT, nullptr, nullptr, 0, .colors = {10, 1} }  // Зеленый
};

#define MENU_ITEMS_COUNT (sizeof(menu_items) / sizeof(MenuItem))

// Внешние переменные из main.c
extern uint8_t current_scancode;

// Структура для управления навигацией по списку
typedef struct {
    uint8_t current_item;   // Текущий выбранный элемент
    uint8_t scroll_offset;  // Смещение прокрутки
    uint8_t item_count;     // Общее количество элементов
    uint8_t max_visible;    // Максимум видимых элементов
} ListNavigation;

// Навигация вверх по списку
static void list_nav_up(ListNavigation* nav) {
    if (nav->current_item > 0) {
        nav->current_item--;
        // Прокрутка вверх
        if (nav->current_item < nav->scroll_offset) {
            nav->scroll_offset = nav->current_item;
        }
    }
}

// Навигация вниз по списку
static void list_nav_down(ListNavigation* nav) {
    if (nav->current_item < nav->item_count - 1) {
        nav->current_item++;
        // Прокрутка вниз
        if (nav->current_item >= nav->scroll_offset + nav->max_visible) {
            nav->scroll_offset = nav->current_item - nav->max_visible + 1;
        }
    }
}

// Навигация вверх по меню (с пропуском NONE элементов)
static void menu_nav_up(uint8_t* current_item, const MenuItem* items, uint8_t item_count) {
    do {
        if (*current_item == 0) {
            *current_item = item_count - 1;
        } else {
            (*current_item)--;
        }
    } while (items[*current_item].type == NONE);
}

// Навигация вниз по меню (с пропуском NONE элементов)
static void menu_nav_down(uint8_t* current_item, const MenuItem* items, uint8_t item_count) {
    do {
        *current_item = (*current_item + 1) % item_count;
    } while (items[*current_item].type == NONE);
}

// Очистка экрана
static void clear_screen(void) {
    memset(VIDEORAM, 0, TEXTMODE_COLS * 2 * TEXTMODE_ROWS);
}

// Ожидание и получение скан-кода
static uint8_t wait_for_scancode(void) {
#ifdef DEBUG
    // В DEBUG режиме нужно обрабатывать ANSI escape последовательности
    static enum {
        ANSI_STATE_NORMAL,
        ANSI_STATE_ESC,
        ANSI_STATE_SS3,
        ANSI_STATE_CSI
    } ansi_state = ANSI_STATE_NORMAL;

    while (true) {
        const int c = getchar_timeout_us(0);

        if (c == PICO_ERROR_TIMEOUT) {
            busy_wait_ms(16);
            continue;
        }

        switch (ansi_state) {
            case ANSI_STATE_NORMAL:
                if (c == 0x1B) {  // ESC
                    ansi_state = ANSI_STATE_ESC;
                    // Ждем следующий символ с таймаутом 50ms
                    const int next = getchar_timeout_us(50000);
                    if (next == PICO_ERROR_TIMEOUT) {
                        // Таймаут - это одиночный ESC
                        ansi_state = ANSI_STATE_NORMAL;
                        return 0x01;  // ESC scancode
                    }
                    // Обрабатываем следующий символ сразу
                    if (next == 'O') {
                        ansi_state = ANSI_STATE_SS3;
                    } else if (next == '[') {
                        ansi_state = ANSI_STATE_CSI;  // CSI последовательность (ESC [)
                    } else {
                        ansi_state = ANSI_STATE_NORMAL;
                        return 0x01;  // ESC scancode
                    }
                } else if (c == 0x0D || c == '\n') {  // Enter
                    return 0x1C;
                } else if (c == 0x20) {  // Space
                    return 0x39;
                } else if (c == 0x7F || c == 0x08) {  // Backspace (DEL или BS)
                    return 0x0E;
                } else {
                    // Возвращаем символ как есть для отладки
                    return c;
                }
                break;

            case ANSI_STATE_ESC:
                // Этот случай теперь обрабатывается выше
                if (c == 'O') {  // SS3 последовательность (ESC O)
                    ansi_state = ANSI_STATE_SS3;
                } else if (c == '[') {  // CSI последовательность (ESC [) - игнорируем
                    ansi_state = ANSI_STATE_NORMAL;
                } else {
                    // Одиночный ESC без последовательности
                    ansi_state = ANSI_STATE_NORMAL;
                    return 0x01;  // ESC scancode
                }
                break;

            case ANSI_STATE_SS3:
                ansi_state = ANSI_STATE_NORMAL;
                switch (c) {
                    case 'A': return 0x48;  // UP Arrow (правильный XT scancode)
                    case 'B': return 0x50;  // DOWN Arrow
                    case 'C': return 0x4D;  // RIGHT Arrow
                    case 'D': return 0x4B;  // LEFT Arrow
                    default: break;
                }
                break;

            case ANSI_STATE_CSI:
                // Проверяем, это финальный символ или промежуточный
                if (c >= '0' && c <= '9') {
                    // Цифры - часть параметров, продолжаем ждать
                    break;
                } else if (c == ';') {
                    // Разделитель параметров, продолжаем ждать
                    break;
                } else {
                    // Финальный символ - обрабатываем и сбрасываем состояние
                    ansi_state = ANSI_STATE_NORMAL;
                    switch (c) {
                        case 'A': return 0x48;  // UP Arrow
                        case 'B': return 0x50;  // DOWN Arrow
                        case 'C': return 0x4D;  // RIGHT Arrow
                        case 'D': return 0x4B;  // LEFT Arrow
                        case '~': break;  // Расширенные последовательности игнорируем
                        default: break;
                    }
                }
                break;
        }
    }
#else
    // В RELEASE режиме используем USB HID
    while (true) {
        keyboard_tick();  // Обработка USB HID клавиатуры

        if (current_scancode != 0) {
            uint8_t scancode = current_scancode;
            current_scancode = 0;  // Сбрасываем
            return scancode;
        }
        busy_wait_ms(16);  // Небольшая задержка 16ms
    }
#endif
}

// Структура элемента файлового списка
typedef struct {
    char name[64];      // Имя файла/директории
    bool is_dir;        // true если директория
} FileEntry;

// Сохранение настроек на SD карту
bool save_settings(void) {
    FIL file;
    UINT bytes_written;
    char line[300];

    // Открываем файл для записи
    FRESULT res = f_open(&file, "/XT/config.sys", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return false;
    }

    // Записываем настройки в текстовом формате
    snprintf(line, sizeof(line), "TANDY=%d\n", settings.tandy_enabled);
    f_write(&file, line, strlen(line), &bytes_written);

    snprintf(line, sizeof(line), "TURBO=%d\n", settings.turbo);
    f_write(&file, line, strlen(line), &bytes_written);

    snprintf(line, sizeof(line), "FDA=%s\n", settings.fda);
    f_write(&file, line, strlen(line), &bytes_written);

    snprintf(line, sizeof(line), "FDB=%s\n", settings.fdb);
    f_write(&file, line, strlen(line), &bytes_written);

    snprintf(line, sizeof(line), "HDD=%s\n", settings.hdd);
    f_write(&file, line, strlen(line), &bytes_written);

    f_close(&file);
    return true;
}

// Загрузка настроек с SD карты
bool load_settings(void) {
    FIL file;
    char line[300];
    UINT bytes_read;

    // Открываем файл для чтения
    FRESULT res = f_open(&file, "/XT/config.sys", FA_READ);
    if (res != FR_OK) {
        return false;  // Файл не найден - используем значения по умолчанию
    }

    // Читаем файл построчно
    while (f_gets(line, sizeof(line), &file)) {
        // Убираем перевод строки
        char* newline = strchr(line, '\n');
        if (newline) *newline = '\0';

        // Парсим строки
        if (strncmp(line, "TANDY=", 6) == 0) {
            settings.tandy_enabled = atoi(line + 6);
        } else if (strncmp(line, "TURBO=", 6) == 0) {
            settings.turbo = atoi(line + 6);
        } else if (strncmp(line, "FDA=", 4) == 0) {
            strncpy(settings.fda, line + 4, sizeof(settings.fda) - 1);
            settings.fda[sizeof(settings.fda) - 1] = '\0';
        } else if (strncmp(line, "FDB=", 4) == 0) {
            strncpy(settings.fdb, line + 4, sizeof(settings.fdb) - 1);
            settings.fdb[sizeof(settings.fdb) - 1] = '\0';
        } else if (strncmp(line, "HDD=", 4) == 0) {
            strncpy(settings.hdd, line + 4, sizeof(settings.hdd) - 1);
            settings.hdd[sizeof(settings.hdd) - 1] = '\0';
        }
    }

    f_close(&file);
    return true;
}

// Файловый браузер для выбора образов дисков
bool file_browser(char* selected_path, uint8_t max_path_len, const char* filter) {
    // Выделяем память динамически
    DIR *dir = (DIR *)malloc(sizeof(DIR));
    FILINFO *fno = (FILINFO *)malloc(sizeof(FILINFO));
    FileEntry *files = (FileEntry *)malloc(50 * sizeof(FileEntry));
    char *current_path = (char *)malloc(256);

    // Проверяем успешность выделения памяти
    if (!dir || !fno || !files || !current_path) {
        // Освобождаем уже выделенную память
        if (dir) free(dir);
        if (fno) free(fno);
        if (files) free(files);
        if (current_path) free(current_path);
        return false;  // Не удалось выделить память
    }

    const uint8_t BROWSER_WIDTH = 60;
    const uint8_t BROWSER_HEIGHT = 20;
    const uint8_t BROWSER_X = (TEXTMODE_COLS - BROWSER_WIDTH) / 2;
    const uint8_t BROWSER_Y = (TEXTMODE_ROWS - BROWSER_HEIGHT) / 2;
    const uint8_t MAX_VISIBLE = 14;  // Максимум видимых элементов (20 - рамка - заголовок - футер)

    // Структура для навигации
    ListNavigation nav = {
        .current_item = 0,
        .scroll_offset = 0,
        .item_count = 0,
        .max_visible = MAX_VISIBLE
    };

    bool exit_browser = false;
    bool file_selected = false;

    // Инициализируем начальный путь
    strcpy(current_path, "/XT");

    while (!exit_browser) {
        // Загрузка списка файлов из текущей директории
        nav.item_count = 0;

        // Добавляем ".." для возврата назад (если не в корне)
        if (strcmp(current_path, "/") != 0) {
            strcpy(files[nav.item_count].name, "..");
            files[nav.item_count].is_dir = true;
            nav.item_count++;
        }

        // Открываем директорию
        FRESULT res = f_opendir(dir, current_path);
        if (res != FR_OK) {
            // Если не удалось открыть, возвращаемся в корень
            strcpy(current_path, "/");
            f_opendir(dir, current_path);
        }

        // Читаем файлы
        while (nav.item_count < 50) {
            res = f_readdir(dir, fno);
            if (res != FR_OK || fno->fname[0] == 0) break;  // Конец списка

            // Пропускаем скрытые файлы
            if (fno->fname[0] == '.') continue;

            // Проверяем фильтр для файлов
            if (!(fno->fattrib & AM_DIR)) {
                // Это файл - проверяем расширение
                const char* ext = strrchr(fno->fname, '.');
                if (!ext || strcmp(ext, filter) != 0) continue;
            }

            // Добавляем в список
            strncpy(files[nav.item_count].name, fno->fname, sizeof(files[nav.item_count].name) - 1);
            files[nav.item_count].name[sizeof(files[nav.item_count].name) - 1] = '\0';
            files[nav.item_count].is_dir = (fno->fattrib & AM_DIR) != 0;
            nav.item_count++;
        }
        f_closedir(dir);

        // Если список пуст, добавляем хотя бы ".."
        if (nav.item_count == 0) {
            strcpy(files[0].name, "..");
            files[0].is_dir = true;
            nav.item_count = 1;
        }

        // Сбрасываем позицию если вышли за пределы
        if (nav.current_item >= nav.item_count) {
            nav.current_item = nav.item_count - 1;
        }

        // Отрисовка браузера
        char title[BROWSER_WIDTH + 1];
        char footer[BROWSER_WIDTH + 1];

        // Заголовок с текущим путем (обрезаем если длинный)
        if (strlen(current_path) > BROWSER_WIDTH - 4) {
            snprintf(title, sizeof(title), "...%s", current_path + strlen(current_path) - (BROWSER_WIDTH - 7));
        } else {
            snprintf(title, sizeof(title), "%s", current_path);
        }

        snprintf(footer, sizeof(footer), "ENTER: Select  ESC: Disable disk");

        // Рисуем дочернее окно
        draw_window(title, footer, BROWSER_X, BROWSER_Y, BROWSER_WIDTH, BROWSER_HEIGHT);

        // Очищаем внутреннее пространство окна (заполняем пустыми строками)
        char empty_line[BROWSER_WIDTH + 1];
        memset(empty_line, ' ', BROWSER_WIDTH - 2);
        empty_line[BROWSER_WIDTH - 2] = '\0';
        for (uint8_t i = 0; i < MAX_VISIBLE; i++) {
            draw_text(empty_line, BROWSER_X + 2, BROWSER_Y + 3 + i, 15, 1);
        }

        // Рисуем файлы
        for (uint8_t i = 0; i < MAX_VISIBLE && (nav.scroll_offset + i) < nav.item_count; i++) {
            uint8_t file_idx = nav.scroll_offset + i;
            char line[BROWSER_WIDTH + 1];

            // Формируем строку с именем файла
            if (files[file_idx].is_dir) {
                snprintf(line, sizeof(line), "  [%s]", files[file_idx].name);
            } else {
                snprintf(line, sizeof(line), "  %s", files[file_idx].name);
            }

            // Ограничиваем длину строки и дополняем пробелами
            size_t len = strlen(line);
            const size_t max_len = BROWSER_WIDTH - 4;  // Минус рамки и отступы
            if (len > max_len) {
                line[max_len] = '\0';  // Обрезаем длинную строку
                len = max_len;
            }
            // Дополняем пробелами до фиксированной длины
            while (len < max_len) {
                line[len++] = ' ';
            }
            line[len] = '\0';

            // Цвета: выбранный - инверсия, директория - голубой, файл - белый
            uint8_t color, bgcolor;
            if (file_idx == nav.current_item) {
                color = 0;
                bgcolor = 15;
            } else if (files[file_idx].is_dir) {
                color = 11;  // Голубой
                bgcolor = 1;
            } else {
                color = 15;  // Белый
                bgcolor = 1;
            }

            draw_text(line, BROWSER_X + 2, BROWSER_Y + 3 + i, color, bgcolor);
        }

        // Ожидаем нажатие клавиши
        uint8_t scancode = wait_for_scancode();

        switch (scancode) {
            case 0x48: // UP Arrow
                list_nav_up(&nav);
                break;

            case 0x50: // DOWN Arrow
                list_nav_down(&nav);
                break;

            case 0x1C: // ENTER
                if (files[nav.current_item].is_dir) {
                    // Вход в директорию
                    if (strcmp(files[nav.current_item].name, "..") == 0) {
                        // Возврат назад
                        char* last_slash = strrchr(current_path, '/');
                        if (last_slash && last_slash != current_path) {
                            *last_slash = '\0';  // Обрезаем последнюю директорию
                        } else {
                            strcpy(current_path, "/");  // Корень
                        }
                    } else {
                        // Вход в поддиректорию
                        if (strcmp(current_path, "/") != 0) {
                            strcat(current_path, "/");
                        }
                        strcat(current_path, files[nav.current_item].name);
                    }
                    nav.current_item = 0;
                    nav.scroll_offset = 0;
                } else {
                    // Выбран файл
                    if (strcmp(current_path, "/") != 0) {
                        snprintf(selected_path, max_path_len, "%s/%s", current_path, files[nav.current_item].name);
                    } else {
                        snprintf(selected_path, max_path_len, "/%s", files[nav.current_item].name);
                    }
                    file_selected = true;
                    exit_browser = true;
                }
                break;

            case 0x01: // ESC - отключить диск
                selected_path[0] = '\0';  // Пустая строка = диск отключен
                file_selected = true;      // Считаем что "выбрано" пустое значение
                exit_browser = true;
                break;

            default:
                break;
        }
    }

    // Освобождаем выделенную память
    free(dir);
    free(fno);
    free(files);
    free(current_path);

    return file_selected;
}

// Отрисовка одного пункта меню
static void draw_menu_item(const MenuItem* item, uint8_t y, bool selected) {
    char line[TEXTMODE_COLS + 1];
    uint8_t color, bgcolor;

    // Определяем цвета в зависимости от типа и состояния
    if (item->type == NONE) {
        // Для NONE используем заданные цвета (всегда)
        color = item->colors.fg_color;
        bgcolor = item->colors.bg_color;
    } else if (selected) {
        // Выбранный пункт - инверсия (включая EXIT)
        color = 0;
        bgcolor = 15;
    } else if (item->type == EXIT) {
        // EXIT не выбран - используем заданные цвета
        color = item->colors.fg_color;
        bgcolor = item->colors.bg_color;
    } else {
        // Обычный пункт
        color = 15;
        bgcolor = 1;
    }

    // Формируем и рисуем текст пункта
    if (item->type == ARRAY || item->type == STRING) {
        // Для ARRAY и STRING рисуем название и значение разными цветами
        const char* format_pos = strstr(item->text, "%s");
        if (format_pos) {
            // Находим позицию %s в строке
            size_t label_len = format_pos - item->text;
            char label[TEXTMODE_COLS + 1];
            char value_str[TEXTMODE_COLS + 1];

            // Копируем название (до %s)
            strncpy(label, item->text, label_len);
            label[label_len] = '\0';

            // Формируем значение
            if (item->type == ARRAY) {
                snprintf(value_str, sizeof(value_str), "%s", item->value_list[*(uint8_t*)item->value]);
            } else {  // STRING
                const char* value = (char*)item->value;
                if (value[0] == '\0') {
                    snprintf(value_str, sizeof(value_str), "<no image selected>");
                } else {
                    snprintf(value_str, sizeof(value_str), "%s", value);
                }
            }

            // Рисуем название (серым цветом для не выбранных, инверсией для выбранных)
            uint8_t label_color = selected ? 0 : 7;   // Серый или черный
            draw_text(label, 2, y, label_color, bgcolor);

            // Рисуем значение (ярким цветом)
            uint8_t value_color = selected ? 0 : 15;  // Белый или черный
            draw_text(value_str, 2 + strlen(label), y, value_color, bgcolor);
        } else {
            // Если нет %s, просто рисуем как есть
            snprintf(line, TEXTMODE_COLS, "%s", item->text);
            draw_text(line, 2, y, color, bgcolor);
        }
    } else {
        // Для NONE и EXIT рисуем как обычно
        snprintf(line, TEXTMODE_COLS, "%s", item->text);
        draw_text(line, 2, y, color, bgcolor);
    }
}

// Главная функция SETUP меню
void setup_menu(void) {
#ifdef DEBUG
    printf("=== SETUP MENU START ===\n");
    printf("TEXTMODE_COLS=%d, TEXTMODE_ROWS=%d\n", TEXTMODE_COLS, TEXTMODE_ROWS);
    printf("VIDEORAM address: %p\n", VIDEORAM);
#endif

    // Сохраняем копию текущих настроек для возможности отмены
    settings_s settings_backup = settings;

    // Очищаем экран
    clear_screen();

    // Буферы для текста (правильный размер для устранения предупреждений)
    char title[TEXTMODE_COLS + 1];
    char footer[TEXTMODE_COLS + 1];

    snprintf(title, sizeof(title), "RP8086 SETUP v1.0");

    uint8_t current_item = 1;  // Начинаем с первого редактируемого пункта (Turbo)
    bool exit_menu = false;
    bool needs_redraw = true;  // Флаг для полной перерисовки

    while (!exit_menu) {
        // Полная перерисовка экрана при необходимости
        if (needs_redraw) {
            // Формируем footer в зависимости от типа выбранного пункта
            switch (menu_items[current_item].type) {
                case ARRAY:
                    snprintf(footer, sizeof(footer), "ENTER/LEFT/RIGHT: Change  UP/DOWN: Navigate  ESC: Exit without saving");
                    break;
                case STRING:
                    snprintf(footer, sizeof(footer), "ENTER: Browse  UP/DOWN: Navigate  ESC: Exit without saving");
                    break;
                case EXIT:
                    snprintf(footer, sizeof(footer), "ENTER: Save and Exit  ESC: Exit without saving");
                    break;
                default:
                    snprintf(footer, sizeof(footer), "UP/DOWN: Navigate  ESC: Exit without saving");
                    break;
            }

            // Рисуем окно с footer
            draw_window(title, footer, 0, 0, TEXTMODE_COLS, TEXTMODE_ROWS);

            // Рисуем все пункты меню
            uint8_t y = 3;
            for (size_t i = 0; i < MENU_ITEMS_COUNT; i++) {
                draw_menu_item(&menu_items[i], y, (i == current_item));
                y++;
            }

            needs_redraw = false;
        }

        // Ожидаем нажатие клавиши
        uint8_t scancode = wait_for_scancode();

#ifdef DEBUG
        printf("Scancode received: 0x%02X\n", scancode);
#endif

        switch (scancode) {
            case 0x48: // UP Arrow
                menu_nav_up(&current_item, menu_items, MENU_ITEMS_COUNT);
                needs_redraw = true;
                break;

            case 0x50: // DOWN Arrow
                menu_nav_down(&current_item, menu_items, MENU_ITEMS_COUNT);
                needs_redraw = true;
                break;

            case 0x4B: // LEFT Arrow
                if (menu_items[current_item].type == ARRAY) {
                    uint8_t* value = (uint8_t*)menu_items[current_item].value;
                    // Циклическое переключение
                    if (*value == 0) {
                        *value = menu_items[current_item].max_value;
                    } else {
                        (*value)--;
                    }
                    needs_redraw = true;
                }
                break;

            case 0x4D: // RIGHT Arrow
                if (menu_items[current_item].type == ARRAY) {
                    uint8_t* value = (uint8_t*)menu_items[current_item].value;
                    // Циклическое переключение
                    if (*value >= menu_items[current_item].max_value) {
                        *value = 0;
                    } else {
                        (*value)++;
                    }
                    needs_redraw = true;
                }
                break;

            case 0x1C: // ENTER (scancode)
                if (menu_items[current_item].type == EXIT) {
                    // Сохраняем настройки и выходим
                    save_settings();
                    exit_menu = true;
                } else if (menu_items[current_item].type == ARRAY) {
                    // Для ARRAY - циклическое переключение (как RIGHT)
                    uint8_t* value = (uint8_t*)menu_items[current_item].value;
                    if (*value >= menu_items[current_item].max_value) {
                        *value = 0;
                    } else {
                        (*value)++;
                    }
                    needs_redraw = true;
                } else if (menu_items[current_item].type == STRING) {
                    // Открываем файловый браузер для выбора образа диска
                    if (file_browser(menu_items[current_item].value, 255, ".img")) {
                        // Файл выбран - обновляем значение
                        // char* str = (char*)menu_items[current_item].value;
                        // strncpy(str, selected_path, menu_items[current_item].max_value);
                        // str[menu_items[current_item].max_value] = '\0';
#ifdef DEBUG
                        printf("File selected: %s\n", menu_items[current_item].value);
#endif
                    }
                    // Полная перерисовка после браузера
                    needs_redraw = true;
                }
                break;

            case 0x01: // ESC (scancode) - Выход без сохранения
                // Восстанавливаем старые настройки
                settings = settings_backup;
                exit_menu = true;
                break;

            default:
                break;
        }
    }

    // Очищаем экран перед выходом
    clear_screen();

#ifdef DEBUG
    printf("=== SETUP MENU EXIT ===\n");
    printf("Settings: Turbo=%d, Tandy=%d\n", settings.turbo, settings.tandy_enabled);
#endif
}
