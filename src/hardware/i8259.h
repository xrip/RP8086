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

__force_inline static void i8259_write(const uint16_t port_number, const uint8_t data) {
    switch (port_number) {
        case 0x20:
            if (data & 0x10) {
                //ICW1: clear all pending/in-service state on initialization.
                i8259.interrupt_mask_register = 0x00;
                __atomic_store_n(&i8259.interrupt_request_register, 0, __ATOMIC_RELAXED);
                __atomic_store_n(&i8259.in_service_register, 0, __ATOMIC_RELAXED);
                i8259.initialization_command_words_1 = data;
                i8259.initialization_command_word_step = 2;
                i8259.register_read_mode = 0;
            } else if ((data & 0x08) == 0) {
                //OCW2
                //                i8259.ocw[2] = register_value;
                switch (data & 0xE0) {
                    case 0x20: { // non-specific EOI: highest-priority in-service IRQ
                        const uint8_t isr = __atomic_load_n(&i8259.in_service_register, __ATOMIC_RELAXED);
                        const uint8_t bit = (uint8_t)(1u << __builtin_ctz((unsigned)isr | 0x100u));
                        __atomic_fetch_and(&i8259.in_service_register, (uint8_t)~bit, __ATOMIC_RELAXED);
                        __sev();
                        break;
                    }
                    case 0x60: // specific EOI
                        __atomic_fetch_and(&i8259.in_service_register,
                                           (uint8_t)~(1u << (data & 7u)), __ATOMIC_RELAXED);
                        __sev();
                        break;
                    default: //other

                        break;
                }
            } else {
                //OCW3
                if (data & 0x08) { // Bit 3 должен быть 1 для OCW3
                    if (data & 0x02) { // Bit 1 = RR (Read Register)
                        // Bit 0 = RIS (0=IRR, 1=ISR)
                        i8259.register_read_mode = data & 1;
                    }
                }
            }
            break;
        case 0x21:
            switch (i8259.initialization_command_word_step) {
                case 2: //ICW2
                    // i8259.initialization_command_words[2] = register_value;
                    i8259.interrupt_vector_offset = data & 0xF8;
                    if (i8259.initialization_command_words_1 & 0x02) {
                        // IBM PC/XT: single PIC, ICW3 omitted.
                        i8259.initialization_command_word_step =
                            (i8259.initialization_command_words_1 & 0x01) ? 4 : 5;
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
                case 5: // just set IMR value now
                    i8259.interrupt_mask_register = data;
                    __sev(); // a previously masked IRQ may now be serviceable
                    break;
            }
            break;
    }
}

__force_inline static uint8_t i8259_get_pending_irqs() {
    const uint8_t irr = __atomic_load_n(&i8259.interrupt_request_register, __ATOMIC_RELAXED);
    const uint8_t isr = __atomic_load_n(&i8259.in_service_register, __ATOMIC_RELAXED);
    // Fully nested fixed priority: only IRQs above the highest-priority ISR bit.
    // OR 0x100 makes ctz defined when ISR == 0 and yields a mask of 0xFF.
    const uint8_t priority_mask = (uint8_t)((1u << __builtin_ctz((unsigned)isr | 0x100u)) - 1u);
    return irr & (uint8_t)~i8259.interrupt_mask_register & priority_mask;
}

__force_inline static void i8259_interrupt(const uint8_t irq) {
    if (irq >= 8u) return;
    __atomic_fetch_or(&i8259.interrupt_request_register, (uint8_t)(1u << irq), __ATOMIC_RELAXED);
    __sev(); // wake Core1 out of WFE to update INTR
}

__force_inline static uint8_t i8259_nextirq() {
    const uint8_t pending = i8259_get_pending_irqs();
    if (!pending) return 0;

    const uint8_t irq = (uint8_t)__builtin_ctz(pending);
    const uint8_t bit = (uint8_t)(1u << irq);
    __atomic_fetch_and(&i8259.interrupt_request_register, (uint8_t)~bit, __ATOMIC_RELAXED);
    __atomic_fetch_or(&i8259.in_service_register, bit, __ATOMIC_RELAXED);

    return (uint8_t)(i8259.interrupt_vector_offset + irq);
}
