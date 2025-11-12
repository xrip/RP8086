#pragma once
#include "../common.h"

// ============================================================================
// Intel 16550 UART (COM1) - Minimal Emulation for DOS CTTY
// ============================================================================
// Порты: 0x3F8-0x3FF (COM1)
//
// Регистры:
// 0x3F8 (DLAB=0): RBR (Read) / THR (Write) - Receive/Transmit Buffer
// 0x3F8 (DLAB=1): DLL - Divisor Latch Low
// 0x3F9 (DLAB=0): IER - Interrupt Enable Register
// 0x3F9 (DLAB=1): DLH - Divisor Latch High
// 0x3FA: IIR (Read) / FCR (Write) - Interrupt ID / FIFO Control
// 0x3FB: LCR - Line Control Register (bit 7 = DLAB)
// 0x3FC: MCR - Modem Control Register
// 0x3FD: LSR - Line Status Register
// 0x3FE: MSR - Modem Status Register
// 0x3FF: SPR - Scratch Register (not implemented)
// ============================================================================

extern uart_16550_s uart;

// ============================================================================
// LSR (Line Status Register) bits - 0x3FD
// ============================================================================
#define LSR_DATA_READY      0x01  // Bit 0: Data ready in RBR
#define LSR_OVERRUN_ERROR   0x02  // Bit 1: Overrun error
#define LSR_PARITY_ERROR    0x04  // Bit 2: Parity error
#define LSR_FRAMING_ERROR   0x08  // Bit 3: Framing error
#define LSR_BREAK_INTERRUPT 0x10  // Bit 4: Break interrupt
#define LSR_THR_EMPTY       0x20  // Bit 5: Transmit Holding Register empty
#define LSR_TRANSMITTER_EMPTY 0x40 // Bit 6: Transmitter empty (TSR + THR)

// ============================================================================
// IIR (Interrupt Identification Register) bits - 0x3FA
// ============================================================================
#define IIR_NO_INTERRUPT    0x01  // Bit 0: No interrupt pending
#define IIR_MODEM_STATUS    0x00  // Priority 4 (lowest)
#define IIR_THR_EMPTY       0x02  // Priority 3
#define IIR_RX_DATA         0x04  // Priority 2
#define IIR_LINE_STATUS     0x06  // Priority 1 (highest)

// ============================================================================
// uart_read - Чтение регистров UART (порты 0x3F8-0x3FF)
// ============================================================================
__force_inline static uint8_t uart_read(const uint32_t port) {
    switch (port) {
        case 0x3F8: {
            // RBR/DLL - Receive Buffer Register or Divisor Latch Low
            if (uart.lcr & 0x80) {
                // DLAB=1: возвращаем младший байт делителя
                return uart.divisor & 0xFF;
            }
            // DLAB=0: возвращаем принятый байт
            const uint8_t data = uart.rbr;
            uart.data_ready = false; // Сбросить флаг после чтения
            return data;
        }

        case 0x3F9: {
            // IER/DLH - Interrupt Enable Register or Divisor Latch High
            if (uart.lcr & 0x80) {
                // DLAB=1: возвращаем старший байт делителя
                return (uart.divisor >> 8) & 0xFF;
            }
            // DLAB=0: возвращаем IER
            return uart.ier;
        }

        case 0x3FA: {
            // IIR - Interrupt Identification Register
            // Для polling режима всегда возвращаем "no interrupt pending"
            return IIR_NO_INTERRUPT;
        }

        case 0x3FB: {
            // LCR - Line Control Register
            return uart.lcr;
        }

        case 0x3FC: {
            // MCR - Modem Control Register
            return uart.mcr;
        }

        case 0x3FD: {
            // LSR - Line Status Register
            // Критичные биты для DOS:
            // Bit 0: Data Ready (есть данные для чтения)
            // Bit 5: THR Empty (можно писать)
            // Bit 6: Transmitter Empty (полностью свободен)
            uint8_t lsr = LSR_THR_EMPTY | LSR_TRANSMITTER_EMPTY; // Всегда готов к передаче
            if (uart.data_ready) {
                lsr |= LSR_DATA_READY; // Есть данные для чтения
            }
            return lsr;
        }

        case 0x3FE: {
            // MSR - Modem Status Register
            // Эмулируем "все сигналы активны" для простоты
            return 0xB0; // CTS=1, DSR=1, DCD=1
        }

        case 0x3FF: {
            // SPR - Scratch Register (не реализовано)
            return 0x00;
        }

        default:
            return 0xFF;
    }
}

// ============================================================================
// uart_write - Запись в регистры UART (порты 0x3F8-0x3FF)
// ============================================================================
__force_inline static void uart_write(const uint32_t port, const uint8_t data) {
    switch (port) {
        case 0x3F8: {
            // THR/DLL - Transmit Holding Register or Divisor Latch Low
            if (uart.lcr & 0x80) {
                // DLAB=1: устанавливаем младший байт делителя
                uart.divisor = (uart.divisor & 0xFF00) | data;
                return;
            }
            // DLAB=0: отправляем байт в USB терминал
            uart.thr = data;
            // putchar_raw(data); // Немедленный вывод в USB
            return;
        }

        case 0x3F9: {
            // IER/DLH - Interrupt Enable Register or Divisor Latch High
            if (uart.lcr & 0x80) {
                // DLAB=1: устанавливаем старший байт делителя
                uart.divisor = (uart.divisor & 0x00FF) | (data << 8);
                return;
            }
            // DLAB=0: устанавливаем IER
            uart.ier = data;
            return;
        }

        case 0x3FA: {
            // FCR - FIFO Control Register (write-only)
            // Для простоты игнорируем - работаем без FIFO
            return;
        }

        case 0x3FB: {
            // LCR - Line Control Register
            uart.lcr = data;
            return;
        }

        case 0x3FC: {
            // MCR - Modem Control Register
            uart.mcr = data;
            return;
        }

        case 0x3FD: {
            // LSR - Line Status Register (read-only, ignore writes)
            return;
        }

        case 0x3FE: {
            // MSR - Modem Status Register (read-only, ignore writes)
            return;
        }

        case 0x3FF: {
            // SPR - Scratch Register (не реализовано)
            return;
        }

        default:
            return;
    }
}
