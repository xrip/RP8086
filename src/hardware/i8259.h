#pragma once
#include "common.h"
extern i8259_s i8259;

__force_inline static uint8_t i8259_read(const uint16_t port_number) {
    switch (port_number) {
        case 0x20:
            return i8259.register_read_mode ? i8259.in_service_register : i8259.interrupt_request_register;
        case 0x21: //read mask register
            return i8259.interrupt_mask_register;
    }
}

// ============================================================================
// Helper Functions - получение не маскированных прерываний
// ============================================================================
__force_inline static uint8_t i8259_get_pending_irqs() {
    // Возвращает битовую маску IRQ, которые запрошены, но не замаскированы
    return i8259.interrupt_request_register & ~i8259.interrupt_mask_register;
}

/*
__force_inline static uint8_t i8259_get_pending_irqs() {
// Базовые pending прерывания (IRR & ~IMR)
uint8_t pending = i8259.interrupt_request_register & ~i8259.interrupt_mask_register;

// Если есть прерывание в обслуживании, применить Fully Nested приоритеты
if (i8259.in_service_register && pending) {
// Найти наивысший приоритет в ISR (наименьший номер бита)
uint8_t highest_isr_bit = __builtin_ctz(i8259.in_service_register);

// Блокировать все более низкие приоритеты (биты старше)
// Пример: если ISR[2] = 1, заблокировать биты 3-7
uint8_t block_mask = ~((1 << (highest_isr_bit + 1)) - 1);
pending &= block_mask;
}

return pending;
}
*/
__force_inline static void i8259_write(const uint16_t port_number, const uint8_t register_value) {
    switch (port_number) {
        case 0x20:
            if (register_value & 0x10) {
                //ICW1
                i8259.interrupt_mask_register = 0x00;
                i8259.initialization_command_words_1 = register_value;
                i8259.initialization_command_word_step = 2;
                i8259.register_read_mode = 0;
            } else if ((register_value & 0x08) == 0) {
                //OCW2
                //                i8259.ocw[2] = register_value;
                switch (register_value & 0xE0) {
                    case 0x20: //non-specific EOI
                        i8259.interrupt_request_register &= ~i8259.in_service_register;
                        i8259.in_service_register = 0x00;
                        break;
                    case 0x60: //specific EOI
                        i8259.interrupt_request_register &= ~(1 << (register_value & 0x03));
                        i8259.in_service_register &= ~(1 << (register_value & 0x03));
                        break;
                    default: //other

                        break;
                }
            } else {
                //OCW3
                //                i8259.ocw[3] = register_value;
                if (register_value & 0x02) {
                    i8259.register_read_mode = register_value & 1;
                }
            }
            break;
        case 0x21:
            switch (i8259.initialization_command_word_step) {
                case 2: //ICW2
                    // i8259.initialization_command_words[2] = register_value;
                    i8259.interrupt_vector_offset = register_value & 0xF8;
                    if (i8259.initialization_command_words_1 & 0x02) {
                        i8259.initialization_command_word_step = 4;
                    } else {
                        i8259.initialization_command_word_step = 3;
                    }
                    break;
                case 3: //ICW3
                    // i8259.initialization_command_words[3] = register_value;
                    if (i8259.initialization_command_words_1 & 0x01) {
                        i8259.initialization_command_word_step = 4;
                    } else {
                        i8259.initialization_command_word_step = 5; //done with ICWs
                    }
                    break;
                case 4: //ICW4
                    // i8259.initialization_command_words[4] = register_value;
                    i8259.initialization_command_word_step = 5; //done with ICWs
                    break;
                case 5: //just set IMR value now
                    i8259.interrupt_mask_register = register_value;
                    break;
            }
            break;
    }
}

__force_inline static void i8259_interrupt(const uint8_t irq) {
    // Устанавливаем бит в IRR если IRQ не замаскирован
    i8259.interrupt_request_register |= 1 << irq;
}

__force_inline static uint8_t i8259_nextirq() {
    const uint8_t pending = i8259_get_pending_irqs();
    if (!pending) return 0;

    // Находим IRQ с наивысшим приоритетом (наименьший номер)
    const uint8_t irq = __builtin_ctz(pending);

    // Перемещаем прерывание из IRR в ISR
    i8259.interrupt_request_register &=  ~(1 << irq);
    i8259.in_service_register |= 1 << irq;

    // Возвращаем вектор прерывания (base + IRQ)
    return i8259.interrupt_vector_offset + irq;
}
