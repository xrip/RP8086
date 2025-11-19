// ========================================
// USB HID Application Layer
// Поддержка клавиатуры и мыши через TinyUSB
//
// Архитектура:
//   - Keyboard: USB HID → XT Scancode → handleScancode() в main app
//   - Mouse: USB HID → Microsoft Serial Mouse protocol → COM1
// ========================================

#include "hid_app.h"
#include "tusb.h"
#include "class/hid/hid.h"
#include "pico/util/queue.h"
#include "../src/hardware/uart16550.h"  // Для отправки данных мыши через COM1

// ========================================
// === Клавиатура (USB HID → XT Scancode) ===
// ========================================
static hid_keyboard_report_t prev_report = {0, 0, {0}};

// ========================================
// === Мышь (Microsoft Serial Mouse) ===
// ========================================
// Обрабатываем HID mouse reports и конвертируем их напрямую в
// Microsoft Serial Mouse protocol (3 байта через COM1 UART)
//
// Автоопределение мыши:
// 1. tuh_hid_mount_cb() устанавливает флаг usb_mouse_connected = true
// 2. DOS драйвер устанавливает DTR/RTS на COM1
// 3. uart_write() в uart16550.h проверяет is_usb_mouse_connected()
// 4. Если мышь подключена → отправляет идентификатор "M"
// 5. DOS драйвер распознает Microsoft Serial Mouse

static bool usb_mouse_connected = false;

// The main emulator wants XT ("set 1") scancodes. Who are we to disappoint them?
// ref1: adafrhit_hid/keycode.py
// ref2: https://www.scs.stanford.edu/10wi-cs140/pintos/specs/kbd/scancodes-9.html
// ref3: https://kbdlayout.info/kbdusx/scancodes?arrangement=ANSI104
typedef struct { const char *d[2]; } usb_to_xt;
static const usb_to_xt conversion[] = {
    [ 53] = { "\x29"          , "\xa9"           }, // ` ~
    [ 30] = { "\x02"          , "\x82"           }, // 1 !
    [ 31] = { "\x03"          , "\x83"           }, // 2 @
    [ 32] = { "\x04"          , "\x84"           }, // 3 #
    [ 33] = { "\x05"          , "\x85"           }, // 4 $
    [ 34] = { "\x06"          , "\x86"           }, // 5 % E
    [ 35] = { "\x07"          , "\x87"           }, // 6 ^
    [ 36] = { "\x08"          , "\x88"           }, // 7 &
    [ 37] = { "\x09"          , "\x89"           }, // 8 *
    [ 38] = { "\x0a"          , "\x8a"           }, // 9 (
    [ 39] = { "\x0b"          , "\x8b"           }, // 0 )
    [ 45] = { "\x0c"          , "\x8c"           }, // - _
    [ 46] = { "\x0d"          , "\x8d"           }, // = +
    [ 42] = { "\x0e"          , "\x8e"           }, // Backspace
    [ 43] = { "\x0f"          , "\x8f"           }, // Tab
    [ 20] = { "\x10"          , "\x90"           }, // Q
    [ 26] = { "\x11"          , "\x91"           }, // W
    [  8] = { "\x12"          , "\x92"           }, // E
    [ 21] = { "\x13"          , "\x93"           }, // R
    [ 23] = { "\x14"          , "\x94"           }, // T
    [ 28] = { "\x15"          , "\x95"           }, // Y
    [ 24] = { "\x16"          , "\x96"           }, // U
    [ 12] = { "\x17"          , "\x97"           }, // I
    [ 18] = { "\x18"          , "\x98"           }, // O
    [ 19] = { "\x19"          , "\x99"           }, // P
    [ 47] = { "\x1a"          , "\x9a"           }, // [ {
    [ 48] = { "\x1b"          , "\x9b"           }, // ] }
    [ 49] = { "\x2b"          , "\xab"           }, // \ |
    [ 57] = { "\x3a"          , "\xba"           }, // CapsLock
    [  4] = { "\x1e"          , "\x9e"           }, // A
    [ 22] = { "\x1f"          , "\x9f"           }, // S
    [  7] = { "\x20"          , "\xa0"           }, // D
    [  9] = { "\x21"          , "\xa1"           }, // F
    [ 10] = { "\x22"          , "\xa2"           }, // G
    [ 11] = { "\x23"          , "\xa3"           }, // H
    [ 13] = { "\x24"          , "\xa4"           }, // J
    [ 14] = { "\x25"          , "\xa5"           }, // K
    [ 15] = { "\x26"          , "\xa6"           }, // L
    [ 51] = { "\x27"          , "\xa7"           }, // ; :
    [ 52] = { "\x28"          , "\xa8"           }, // ' "
    [ 50] = { "\x00"          , "\x80"           }, // non-US-1
    [ 40] = { "\x1c"          , "\x9c"           }, // Enter
    [225] = { "\x2a"          , "\xaa"           }, // LShift
    [ 29] = { "\x2c"          , "\xac"           }, // Z
    [ 27] = { "\x2d"          , "\xad"           }, // X
    [  6] = { "\x2e"          , "\xae"           }, // C
    [ 25] = { "\x2f"          , "\xaf"           }, // V
    [  5] = { "\x30"          , "\xb0"           }, // B
    [ 17] = { "\x31"          , "\xb1"           }, // N
    [ 16] = { "\x32"          , "\xb2"           }, // M
    [ 54] = { "\x33"          , "\xb3"           }, // , <
    [ 55] = { "\x34"          , "\xb4"           }, // . >
    [ 56] = { "\x35"          , "\xb5"           }, // / ?
    [229] = { "\x36"          , "\xb6"           }, // RShift
    [224] = { "\x1d"          , "\x9d"           }, // LCtrl
    [226] = { "\x38"          , "\xb8"           }, // LAlt
    [ 44] = { "\x39"          , "\xb9"           }, // space
    [230] = { "\xe0\x38"      , "\xe0\xb8"       }, // RAlt
    [228] = { "\xe0\x1d"      , "\xe0\x9d"       }, // RCtrl
    [ 73] = { "\xe0\x52"      , "\xe0\xd2"       }, // Insert
    [ 76] = { "\xe0\x53"      , "\xe0\xd3"       }, // Delete
    [ 74] = { "\xe0\x47"      , "\xe0\xc7"       }, // Home
    [ 77] = { "\xe0\x4f"      , "\xe0\xcf"       }, // End
    [ 75] = { "\xe0\x49"      , "\xe0\xc9"       }, // PgUp
    [ 78] = { "\xe0\x51"      , "\xe0\xd1"       }, // PgDn
    [ 80] = { "\xe0\x4b"      , "\xe0\xcb"       }, // Left
    [ 82] = { "\xe0\x48"      , "\xe0\xc8"       }, // Up
    [ 81] = { "\xe0\x50"      , "\xe0\xd0"       }, // Down
    [ 79] = { "\xe0\x4d"      , "\xe0\xcd"       }, // Right
    [ 83] = { "\x45"          , "\xc5"           }, // NumLock
    [ 95] = { "\x47"          , "\xc7"           }, // KP-7 / Home
    [ 92] = { "\x4b"          , "\xcb"           }, // KP-4 / Left
    [ 89] = { "\x4f"          , "\xcf"           }, // KP-1 / End
    [ 84] = { "\xe0\x35"      , "\xe0\xb5"       }, // KP-/
    [ 96] = { "\x48"          , "\xc8"           }, // KP-8 / Up
    [ 93] = { "\x4c"          , "\xcc"           }, // KP-5
    [ 90] = { "\x50"          , "\xd0"           }, // KP-2 / Down
    [ 98] = { "\x52"          , "\xd2"           }, // KP-0 / Ins
    [ 85] = { "\x37"          , "\xb7"           }, // KP-*
    [ 97] = { "\x49"          , "\xc9"           }, // KP-9 / PgUp
    [ 94] = { "\x4d"          , "\xcd"           }, // KP-6 / Right
    [ 91] = { "\x51"          , "\xd1"           }, // KP-3 / PgDn
    [ 99] = { "\x53"          , "\xd3"           }, // KP-. / Del
    [ 86] = { "\x4a"          , "\xca"           }, // KP--
    [ 87] = { "\x4e"          , "\xce"           }, // KP-+
    [ 88] = { "\xe0\x1c"      , "\xe0\x9c"       }, // KP-Enter
    [ 41] = { "\x01"          , "\x81"           }, // Esc
    [ 58] = { "\x3b"          , "\xbb"           }, // F1
    [ 59] = { "\x3c"          , "\xbc"           }, // F2
    [ 60] = { "\x3d"          , "\xbd"           }, // F3
    [ 61] = { "\x3e"          , "\xbe"           }, // F4
    [ 62] = { "\x3f"          , "\xbf"           }, // F5
    [ 63] = { "\x40"          , "\xc0"           }, // F6
    [ 64] = { "\x41"          , "\xc1"           }, // F7
    [ 65] = { "\x42"          , "\xc2"           }, // F8
    [ 66] = { "\x43"          , "\xc3"           }, // F9
    [ 67] = { "\x44"          , "\xc4"           }, // F10
    [ 68] = { "\x57"          , "\xd7"           }, // F11
    [ 69] = { "\x58"          , "\xd8"           }, // F12
    [ 70] = { "\xe0\x37"      , "\xe0\xb7"       }, // PrtScr
    [154] = { "\x54"          , "\xd4"           }, // Alt+SysRq
    [ 71] = { "\x46"          , "\xc6"           }, // ScrollLock
    [ 72] = { "\xe1\x1d\x45\xe1\x9d\xc5", ""     }, // Pause
    [227] = { "\xe0\x5b"      , "\xe0\xdb"       }, // LWin (USB: LGUI)
    [231] = { "\xe0\x5c"      , "\xe0\xdc"       }, // RWin (USB: RGUI)
};

static queue_t kq;
void keyboard_init(void) {
    queue_init(&kq, /* element_size */ sizeof(uint8_t), /* element_count */ 32);
}

void mouse_init() {
    // Microsoft Serial Mouse автоинициализация:
    // 1. При установке DTR/RTS драйвером → UART отправит идентификатор "M" (uart16550.h)
    // 2. HID mouse reports → автоматически конвертируются в Serial Mouse packets через COM1
    // 3. DOS драйвер (CTMOUSE, MOUSE.COM) читает данные из COM1 (порт 0x3F8)
    usb_mouse_connected = false;
}

bool is_usb_mouse_connected(void) {
    return usb_mouse_connected;
}

static void kbd_add_sequence(const uint8_t *sequence) {
    if (!sequence)
        return;
    while(*sequence) {
        queue_try_add(&kq, sequence++);
    }
}

// Конвертирует USB HID keycode в XT scancode и добавляет в очередь
// is_release: 0 = make code (нажатие), 1 = break code (отпускание)
static void kbd_raw_key(int usb_code, int is_release) {
    const uint8_t *sequence = (const uint8_t*)conversion[usb_code].d[is_release];
    kbd_add_sequence(sequence);
}

static void kbd_raw_key_down(int usb_code) {
    kbd_raw_key(usb_code, 0);
}

static void kbd_raw_key_up(int usb_code) {
    kbd_raw_key(usb_code, 1);
}

static inline bool find_key_in_report(hid_keyboard_report_t const* report, uint8_t keycode) {
    for (uint8_t i = 0; i < 6; i++) {
        if (report->keycode[i] == keycode) {
            return true;
        }
    }
    return false;
}

static void process_kbd_report(hid_keyboard_report_t const* r1, hid_keyboard_report_t const* r2,
                               void (*kbd_raw_key_cb)(int code)) {

    for(int bit = 8; bit--;) {
        int weight = 1 << bit;
        if((r1->modifier & weight) && !(r2->modifier & weight)) {
            kbd_raw_key_cb(bit + 0xe0);
        }
    }

    // Process keycodes
    for (int i = 0; i < 6; i++) {
        if (r1->keycode[i]) {
            int keycode = r1->keycode[i];
            if (!find_key_in_report(r2, keycode)) {
                kbd_raw_key_cb(keycode);
            }
        }
    }
}

static void find_pressed_keys(hid_keyboard_report_t const* report) {
    process_kbd_report(report, &prev_report, &kbd_raw_key_down);
}

static void find_released_keys(hid_keyboard_report_t const* report) {
    process_kbd_report(&prev_report, report, &kbd_raw_key_up);
}

// ========================================
// === Mouse (Microsoft Serial Mouse) ===
// ========================================

// Обработка HID mouse report и конвертация в Microsoft Serial Mouse protocol
// Формат протокола (3 байта):
//   Byte 0: [0 1 L R Y7 Y6 X7 X6] - sync byte (биты 7,6=0,1; L,R - кнопки; X7-X6,Y7-Y6 - старшие биты координат)
//   Byte 1: [0 0 X5 X4 X3 X2 X1 X0] - младшие 6 бит X смещения
//   Byte 2: [0 0 Y5 Y4 Y3 Y2 Y1 Y0] - младшие 6 бит Y смещения
static void process_mouse_report(uint8_t const* report, uint16_t len) {
    if (len < 3) return; // Минимальный размер HID mouse report

    // ========================================
    // Шаг 1: Парсинг HID mouse report
    // ========================================
    // Стандартный HID mouse report format:
    // Byte 0: buttons (bit0=L, bit1=R, bit2=Middle)
    // Byte 1: X movement (signed 8-bit)
    // Byte 2: Y movement (signed 8-bit)

    const uint8_t buttons = report[0] & 0x03; // Берём только L и R кнопки (Microsoft Serial Mouse = 2-button)
    int8_t dx = (int8_t)report[1];
    int8_t dy = (int8_t)report[2];

    // ========================================
    // Шаг 2: Конвертация в Microsoft Serial Mouse protocol
    // ========================================
    // Microsoft Serial Mouse использует 7-битные координаты со знаком
    // Диапазон: -64..+63 (6 бит + знак в старшем бите синхронизации)

    // Ограничиваем координаты до ±63 (чтобы влезли в 6 бит + знак)
    if (dx > 63) dx = 63;
    if (dx < -64) dx = -64;
    if (dy > 63) dy = 63;
    if (dy < -64) dy = -64;

    // Формируем 3 байта протокола Microsoft Serial Mouse
    uint8_t packet[3];

    // Byte 0: [0 1 L R Y7 Y6 X7 X6]
    //   биты 7,6 = 0,1 (синхронизация)
    //   бит 5 = Left button (1=нажата)
    //   бит 4 = Right button (1=нажата)
    //   биты 3,2 = старшие биты Y (Y7, Y6)
    //   биты 1,0 = старшие биты X (X7, X6)
    packet[0] = 0x40;  // Биты 7,6 = 0,1
    packet[0] |= ((buttons & 0x01) << 5);  // Left button → bit 5
    packet[0] |= ((buttons & 0x02) << 3);  // Right button → bit 4
    packet[0] |= ((dy >> 4) & 0x0C);       // Y7,Y6 → bits 3,2
    packet[0] |= ((dx >> 6) & 0x03);       // X7,X6 → bits 1,0

    // Byte 1: [0 0 X5 X4 X3 X2 X1 X0]
    packet[1] = dx & 0x3F;

    // Byte 2: [0 0 Y5 Y4 Y3 Y2 Y1 Y0]
    packet[2] = dy & 0x3F;

    // ========================================
    // Шаг 3: Отправка через COM1 (UART)
    // ========================================
    uart_write_byte(packet[0]);
    uart_write_byte(packet[1]);
    uart_write_byte(packet[2]);
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    (void)desc_len;
    (void)desc_report;

    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    // Отслеживаем подключение USB мыши
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        usb_mouse_connected = true;
        // printf("HID Mouse connected (VID:PID %04X:%04X)\n", vid, pid);
    }

    (void)vid;
    (void)pid;

    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    (void)instance;
    (void)dev_addr;

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
        const hid_keyboard_report_t *kbd_report = (const hid_keyboard_report_t*)report;

        find_pressed_keys(kbd_report);
        find_released_keys(kbd_report);
        memcpy(&prev_report, report, sizeof(hid_keyboard_report_t));
    } else if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        process_mouse_report(report, len);
    }

    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    // Проверяем, была ли отключена мышь
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        usb_mouse_connected = false;
        // printf("HID Mouse disconnected\n");
    }
}


void keyboard_tick(void) {
    tuh_task();
    uint8_t xt_code;
    if (queue_try_remove(&kq, &xt_code)) {
        handleScancode(xt_code);
    }
}
