#pragma once

#include <stdint.h>
#include <stdbool.h>

// ========================================
// USB HID Driver API
// Поддержка клавиатуры и мыши через USB
// ========================================

// === Клавиатура ===
void keyboard_init(void);
void keyboard_tick(void);

// External handler for scancodes (implemented in main application)
bool handleScancode(uint32_t ps2scancode);

// === Мышь (Microsoft Serial Mouse protocol) ===
void mouse_init(void);

// TODO: Добавить функцию для получения состояния мыши
// mouse_state_t* mouse_get_state(void);
// void mouse_send_to_uart(void); // Отправка данных через COM1
