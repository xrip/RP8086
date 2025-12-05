#include "setup.h"
#include "graphics.h"
#include "common.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pico/time.h>
#include "../drivers/fatfs/ff.h"
#include "pico/stdio.h"
#ifndef DEBUG
#include "../drivers/usbhid/hid_app.h"
#endif

#define MENU_ITEMS_COUNT (sizeof(menu_items) / sizeof(MenuItem))
#define BROWSER_WIDTH 60
#define BROWSER_HEIGHT 20
#define BROWSER_MAX_VISIBLE 14
#define BROWSER_MAX_FILES 50
#define CONFIG_FILE "/XT/config.sys"

settings_s settings = {
    .tandy_enabled = 0,
    .turbo = 1,
    .fda = "/XT/fdd.img",
    .fdb = "",
    .hdd = "/XT/hdd.img",
};

static const MenuItem menu_items[] = {
    { "Features:", NONE, nullptr, nullptr, 0, .colors = {14, 1} },
    { "  Turbo 6MHz:              %s", ARRAY, &settings.turbo, nullptr, 1, { "DISABLED", "ENABLED" }},
    { "  IBM PCjr/Tandy mode:     %s", ARRAY, &settings.tandy_enabled, nullptr, 1, { "NO", "YES" }},
    { "", NONE, nullptr, nullptr, 0, .colors = {7, 1} },
    { "Storage devices:", NONE, nullptr, nullptr, 0, .colors = {14, 1} },
    { "  Floppy #1:               %s", STRING, &settings.fda, nullptr, 255, {} },
    { "  Floppy #2:               %s", STRING, &settings.fdb, nullptr, 255, {} },
    { "  Hard drive:              %s", STRING, &settings.hdd, nullptr, 255, {} },
    { "", NONE, nullptr, nullptr, 0, .colors = {7, 1} },
    { "Save Settings And Exit", EXIT, nullptr, nullptr, 0, .colors = {10, 1} }
};

extern uint8_t current_scancode;

typedef struct {
    uint8_t current_item;
    uint8_t scroll_offset;
    uint8_t item_count;
    uint8_t max_visible;
} ListNavigation;

typedef struct {
    char name[64];
    bool is_dir;
} FileEntry;

static void list_nav_up(ListNavigation* nav) {
    if (nav->current_item > 0) {
        nav->current_item--;
        if (nav->current_item < nav->scroll_offset) {
            nav->scroll_offset = nav->current_item;
        }
    }
}

static void list_nav_down(ListNavigation* nav) {
    if (nav->current_item < nav->item_count - 1) {
        nav->current_item++;
        if (nav->current_item >= nav->scroll_offset + nav->max_visible) {
            nav->scroll_offset = nav->current_item - nav->max_visible + 1;
        }
    }
}

static void menu_nav_up(uint8_t* current_item, const MenuItem* items, uint8_t item_count) {
    do {
        *current_item = (*current_item == 0) ? item_count - 1 : *current_item - 1;
    } while (items[*current_item].type == NONE);
}

static void menu_nav_down(uint8_t* current_item, const MenuItem* items, uint8_t item_count) {
    do {
        *current_item = (*current_item + 1) % item_count;
    } while (items[*current_item].type == NONE);
}

static inline void clear_screen(void) {
    memset(VIDEORAM, 0, TEXTMODE_COLS * 2 * TEXTMODE_ROWS);
}

static inline void cycle_array_value(const MenuItem* item) {
    uint8_t* value = (uint8_t*)item->value;
    *value = (*value >= item->max_value) ? 0 : *value + 1;
}

static uint8_t wait_for_scancode(void) {
#ifdef DEBUG
    static enum { ANSI_NORMAL, ANSI_ESC, ANSI_SS3, ANSI_CSI } ansi_state = ANSI_NORMAL;

    while (true) {
        const int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            busy_wait_ms(16);
            continue;
        }

        switch (ansi_state) {
            case ANSI_NORMAL:
                if (c == 0x1B) {
                    const int next = getchar_timeout_us(50000);
                    if (next == PICO_ERROR_TIMEOUT) return 0x01;
                    ansi_state = (next == 'O') ? ANSI_SS3 : (next == '[') ? ANSI_CSI : ANSI_NORMAL;
                    if (ansi_state == ANSI_NORMAL) return 0x01;
                } else if (c == 0x0D || c == '\n') return 0x1C;
                else if (c == 0x20) return 0x39;
                else if (c == 0x7F || c == 0x08) return 0x0E;
                else return c;
                break;

            case ANSI_ESC:
                ansi_state = (c == 'O') ? ANSI_SS3 : ANSI_NORMAL;
                if (ansi_state == ANSI_NORMAL && c != '[') return 0x01;
                break;

            case ANSI_SS3:
            case ANSI_CSI:
                if (ansi_state == ANSI_CSI && (c >= '0' && c <= '9' || c == ';')) break;
                ansi_state = ANSI_NORMAL;
                switch (c) {
                    case 'A': return 0x48;
                    case 'B': return 0x50;
                    case 'C': return 0x4D;
                    case 'D': return 0x4B;
                    default: break;
                }
                break;
        }
    }
#else
    while (true) {
        keyboard_tick();
        if (current_scancode != 0) {
            uint8_t scancode = current_scancode;
            current_scancode = 0;
            return scancode;
        }
        busy_wait_ms(16);
    }
#endif
}

bool save_settings(void) {
    FIL file;
    UINT bw;
    char line[300];

    if (f_open(&file, CONFIG_FILE, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;

    const struct { const char* key; const void* val; bool is_str; } cfg[] = {
        {"TANDY", &settings.tandy_enabled, false},
        {"TURBO", &settings.turbo, false},
        {"FDA", settings.fda, true},
        {"FDB", settings.fdb, true},
        {"HDD", settings.hdd, true}
    };

    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
        if (cfg[i].is_str) {
            snprintf(line, sizeof(line), "%s=%s\n", cfg[i].key, (const char*)cfg[i].val);
        } else {
            snprintf(line, sizeof(line), "%s=%d\n", cfg[i].key, *(const uint8_t*)cfg[i].val);
        }
        f_write(&file, line, strlen(line), &bw);
    }

    f_close(&file);
    return true;
}

bool load_settings(void) {
    FIL file;
    char line[300];

    if (f_open(&file, CONFIG_FILE, FA_READ) != FR_OK) return false;

    while (f_gets(line, sizeof(line), &file)) {
        char* newline = strchr(line, '\n');
        if (newline) *newline = '\0';

        char* eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        const char* val = eq + 1;

        if (strcmp(line, "TANDY") == 0) settings.tandy_enabled = atoi(val);
        else if (strcmp(line, "TURBO") == 0) settings.turbo = atoi(val);
        else if (strcmp(line, "FDA") == 0) strncpy(settings.fda, val, sizeof(settings.fda) - 1);
        else if (strcmp(line, "FDB") == 0) strncpy(settings.fdb, val, sizeof(settings.fdb) - 1);
        else if (strcmp(line, "HDD") == 0) strncpy(settings.hdd, val, sizeof(settings.hdd) - 1);
    }

    f_close(&file);
    return true;
}

bool file_browser(char* selected_path, uint8_t max_path_len, const char* filter) {
    DIR *dir = malloc(sizeof(DIR));
    FILINFO *fno = malloc(sizeof(FILINFO));
    FileEntry *files = malloc(BROWSER_MAX_FILES * sizeof(FileEntry));
    char *current_path = malloc(256);

    if (!dir || !fno || !files || !current_path) {
        if (dir) free(dir);
        if (fno) free(fno);
        if (files) free(files);
        if (current_path) free(current_path);
        return false;
    }

    const uint8_t BROWSER_X = (TEXTMODE_COLS - BROWSER_WIDTH) / 2;
    const uint8_t BROWSER_Y = (TEXTMODE_ROWS - BROWSER_HEIGHT) / 2;

    ListNavigation nav = {0, 0, 0, BROWSER_MAX_VISIBLE};
    bool exit_browser = false, file_selected = false;

    strcpy(current_path, "/XT");

    while (!exit_browser) {
        nav.item_count = 0;

        if (strcmp(current_path, "/") != 0) {
            strcpy(files[nav.item_count].name, "..");
            files[nav.item_count++].is_dir = true;
        }

        if (f_opendir(dir, current_path) != FR_OK) {
            strcpy(current_path, "/");
            f_opendir(dir, current_path);
        }

        while (nav.item_count < BROWSER_MAX_FILES) {
            if (f_readdir(dir, fno) != FR_OK || fno->fname[0] == 0) break;
            if (fno->fname[0] == '.') continue;

            if (!(fno->fattrib & AM_DIR)) {
                const char* ext = strrchr(fno->fname, '.');
                if (!ext || strcmp(ext, filter) != 0) continue;
            }

            strncpy(files[nav.item_count].name, fno->fname, sizeof(files[0].name) - 1);
            files[nav.item_count].name[sizeof(files[0].name) - 1] = '\0';
            files[nav.item_count++].is_dir = (fno->fattrib & AM_DIR) != 0;
        }
        f_closedir(dir);

        if (nav.item_count == 0) {
            strcpy(files[0].name, "..");
            files[0].is_dir = true;
            nav.item_count = 1;
        }

        if (nav.current_item >= nav.item_count) {
            nav.current_item = nav.item_count - 1;
        }

        char title[BROWSER_WIDTH + 1];
        if (strlen(current_path) > BROWSER_WIDTH - 4) {
            snprintf(title, sizeof(title), "...%s", current_path + strlen(current_path) - (BROWSER_WIDTH - 7));
        } else {
            snprintf(title, sizeof(title), "%s", current_path);
        }

        draw_window(title, "ENTER: Select  ESC: Disable disk", BROWSER_X, BROWSER_Y, BROWSER_WIDTH, BROWSER_HEIGHT);

        for (uint8_t i = 0; i < BROWSER_MAX_VISIBLE && (nav.scroll_offset + i) < nav.item_count; i++) {
            uint8_t file_idx = nav.scroll_offset + i;
            char line[BROWSER_WIDTH + 1];

            snprintf(line, sizeof(line), files[file_idx].is_dir ? "  [%s]" : "  %s", files[file_idx].name);

            size_t len = strlen(line);
            const size_t max_len = BROWSER_WIDTH - 4;
            if (len > max_len) {
                line[max_len] = '\0';
                len = max_len;
            }
            while (len < max_len) line[len++] = ' ';
            line[len] = '\0';

            uint8_t color = (file_idx == nav.current_item) ? 0 : (files[file_idx].is_dir ? 11 : 15);
            uint8_t bgcolor = (file_idx == nav.current_item) ? 15 : 1;

            draw_text(line, BROWSER_X + 2, BROWSER_Y + 3 + i, color, bgcolor);
        }

        uint8_t scancode = wait_for_scancode();

        switch (scancode) {
            case 0x48: list_nav_up(&nav); break;
            case 0x50: list_nav_down(&nav); break;

            case 0x1C:
                if (files[nav.current_item].is_dir) {
                    if (strcmp(files[nav.current_item].name, "..") == 0) {
                        char* last_slash = strrchr(current_path, '/');
                        if (last_slash && last_slash != current_path) {
                            *last_slash = '\0';
                        } else {
                            strcpy(current_path, "/");
                        }
                    } else {
                        if (strcmp(current_path, "/") != 0) strcat(current_path, "/");
                        strcat(current_path, files[nav.current_item].name);
                    }
                    nav.current_item = nav.scroll_offset = 0;
                } else {
                    snprintf(selected_path, max_path_len, (strcmp(current_path, "/") != 0) ? "%s/%s" : "/%s",
                             current_path, files[nav.current_item].name);
                    file_selected = exit_browser = true;
                }
                break;

            case 0x01:
                selected_path[0] = '\0';
                file_selected = exit_browser = true;
                break;
        }
    }

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
