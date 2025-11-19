# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RP8086 is a hardware-software complex that uses a Raspberry Pi Pico (RP2350B) as a chipset for the Intel 8086 processor. The RP2350B acts as:
- Bus controller (via PIO state machine)
- ROM/RAM emulator (736KB RAM via external PSRAM + 16KB VRAM + 8KB BIOS)
- I/O controller (Intel 8259A, 8253, 8237, 8272A, 16550 UART emulation)
- Clock generator (4.75-6 MHz PWM, configurable)
- CGA video adapter with hardware VGA output (GPIO 30-37)

The system successfully boots DR-DOS 7 from emulated floppy disk and runs at stable **4.75-6 MHz**.

## Build Commands

```bash
# Build from cmake-build-rp2350b directory
cd cmake-build-rp2350b
cmake --build .

# Output: bin/rp2350b/RP8086.uf2 (flashable to RP2350B)
```

**Build Requirements:**
- Pico SDK (set `PICO_SDK_PATH` environment variable)
- CMake >= 3.13
- ARM GCC toolchain
- C23 standard

**Build Configuration:**
- Optimization: `-Ofast` with LTO enabled
- Binary mode: `copy_to_ram` (executes from RAM for maximum performance)
- System clock: 500 MHz (overclocked with voltage boost to 1.60V)
- i8086 clock: 4.75-6 MHz (configurable via `I8086_CLOCK_SPEED` in common.h)

## Architecture

### Two-Layer Bus Design

**Layer 1: PIO (Hardware - i8086_bus.pio)**
- Runs independently on RP2350B's Programmable I/O state machine at 500 MHz (clkdiv = 1.0, no frequency divider)
- Handles all timing-critical bus operations in hardware
- Manages i8086 bus signals: ALE, RD, WR, M/IO, BHE, INTA, READY
- Controls bidirectional data bus (GPIO 0-15)
- Generates interrupts (IRQ0 for writes, IRQ1 for reads, IRQ3 for INTA)
- 32-bit protocol: ARM returns `[data:16][pindirs_mask:16]` for ISA-compatibility
- PIO clock runs at full system frequency (500 MHz) without divider for maximum timing precision

**Layer 2: ARM Cortex-M33 (Software - cpu_bus.c)**
- Services interrupts from PIO via FIFO
- Implements memory/IO emulation logic
- All handlers use `__time_critical_func` macro to run from RAM
- Interrupts run at `PICO_HIGHEST_IRQ_PRIORITY`

### Multicore Architecture

**Core0 (Main) - User Interface & I/O:**
- System initialization (overclock to 500 MHz, USB setup, PSRAM init)
- Launches Core1 via `multicore_launch_core1(bus_handler_core)`
- Main loop responsibilities:
  - CGA hardware video output: Renders VIDEORAM to VGA port (GPIO 30-37) at 60 FPS
  - Keyboard input: Processes USB serial input and converts to scancodes
  - Debug commands: R (reset), B (bootloader), M (memory dump), V (video dump), C (CTTY mode), P (CGA registers)
  - Scancode injection: ASCII → IBM PC/XT Scancode conversion via i8042 emulation
  - CTTY mode: Direct serial terminal mode via COM1/USB

**Core1 (bus_handler_core) - Bus & IRQ Management:**
- Hardware initialization sequence (critical order):
  1. `start_cpu_clock()` - PWM clock generation (4.75-6 MHz)
  2. `pic_init()` - INTR pin setup
  3. `cpu_bus_init()` - PIO program + IRQ handlers
  4. `reset_cpu()` - Release i8086 from reset (MUST be last)
- Main loop:
  - Timer IRQ generation via hardware `repeating_timer` (18.2 Hz, IBM PC compatible)
  - INTR signal management: Sets INTR=HIGH when pending interrupts exist
  - Uses `__wfe()` for power efficiency when idle

**INTA Protocol:**
- INTA handled in PIO state machine via IRQ 3
- Process: PIO detects INTA=LOW → generates IRQ 3 → ARM calls `i8259_nextirq()` → returns interrupt vector
- Next read cycle returns vector in format `0xFF00 | vector`

### GPIO Pin Mapping (config.h + i8086_bus.pio)

**Critical bus signals (hardcoded in PIO):**
- GPIO 0-15: AD0-AD15 (multiplexed Address/Data bus, bidirectional)
- GPIO 16-19: A16-A19 (Address bus upper 4 bits)
- GPIO 20: ALE (Address Latch Enable)
- GPIO 21: RD (Read strobe, active LOW)
- GPIO 22: WR (Write strobe, active LOW)
- GPIO 23: INTA (Interrupt acknowledge, active LOW)
- GPIO 24: M/IO (Memory/IO select: 1=memory, 0=I/O)
- GPIO 25: BHE (Bus High Enable)
- GPIO 26: INTR (Interrupt request output, active HIGH)
- GPIO 27: READY (Wait state control, managed by PIO sideset)
- GPIO 28: RESET (Reset output, active HIGH)
- GPIO 29: CLK (Clock output, PWM)

**Hardware peripherals:**
- GPIO 30-37: VGA_DATA (8-bit hardware CGA video output)
- GPIO 47: PSRAM_CS (Chip Select for external PSRAM)

**Important:** Pin assignments in `i8086_bus.pio` must match hardware wiring. RP2350B is 5V tolerant on inputs, allowing direct connection to i8086 (5V logic) without level shifters.

## Code Organization

**main.c** - Entry point and Core0 main loop:
- System initialization (USB, PSRAM, video)
- CGA video rendering at 60 FPS (text 80×25, graphics 320×200×4)
- Keyboard input processing (ASCII → Scancode conversion via i8042)
- Debug commands (uppercase): R, B, M, V, C, P
- CTTY mode support (direct COM1/USB terminal mode)

**cpu.c/h** - i8086 CPU control:
- `start_cpu_clock()`: PWM generation for i8086 clock (33% duty cycle)
- `reset_cpu()`: RESET sequence

**cpu_bus.c/h** - Bus controller and memory emulation:
- `cpu_bus_init()`: Loads PIO program, sets up IRQ handlers
- `bus_read_handler()`: Services read requests (IRQ1) and INTA cycles (IRQ3)
- `bus_write_handler()`: Services write requests (IRQ0)
- `i8086_read()` / `i8086_write()`: Highly optimized 16-bit memory/IO operations

**common.h** - Common definitions and structures:
- System configuration (clocks, pins, PIO settings)
- Device structures: `i8259_s`, `i8253_s`, `dma_channel_s`, `i8272_s`, `uart_16550_s`, `mc6845_s`, `cga_s`
- Helper functions: `write_to()` for BHE support

**memory.h** - Memory emulation:
- `memory_read()` / `memory_write()`: Handles RAM/ROM/VIDEORAM/UMB access
- 4-byte aligned arrays for optimal performance
- PSRAM support for 736KB conventional RAM

**ports.h** - I/O ports routing:
- `port_read()` / `port_write()`: 16-bit operations with BHE support
- Direct routing to device emulation functions (no port array)

**hardware/i8259.h** - Intel 8259A PIC:
- Full ICW1-ICW4 and OCW1-OCW3 support
- Fully Nested Mode with automatic priorities
- Non-specific EOI uses `__builtin_ctz()` to find highest priority ISR bit
- Specific EOI clears only ISR without touching IRR
- Functions: `i8259_read()`, `i8259_write()`, `i8259_interrupt()`, `i8259_nextirq()`

**hardware/i8253.h** - Intel 8253 PIT:
- 3 independent channels with reload values
- Access modes: LOBYTE, HIBYTE, TOGGLE, LATCHCOUNT
- **Mode 0 support**: One-shot interrupt on terminal count via hardware `repeating_timer`
- Channel 0 uses `irq0_timer_callback()` for dynamic timer control
- Dynamic timer_interval calculation based on reload value

**hardware/i8237.h** - Intel 8237 DMA Controller:
- 4 channels with auto-initialization support
- Memory-to-memory transfer mode
- Page registers for 20-bit addressing
- Functions: `i8237_read()`, `i8237_write()`, `i8237_readpage()`, `i8237_writepage()`

**hardware/i8272.h** - Intel 8272A Floppy Controller:
- Full command emulation (read/write sectors, seek, recalibrate)
- DMA integration via channel 2
- IRQ6 generation on operation completion
- Supports multiple disk formats (160KB-1.44MB)

**hardware/uart.h** - 16550 UART emulation:
- Full COM1 emulation at ports 0x3F8-0x3FF
- FIFO support for buffered serial I/O
- Used for CTTY mode (direct terminal access)

**hardware/mc6845.h** - Motorola 6845 CRTC:
- Video timing control for CGA adapter
- Cursor position and display parameters
- Register-based configuration via ports 0x3D4/0x3D5

**i8086_bus.pio** - PIO state machine:
- Captures 20-bit address + control signals
- Optimized polling for RD/WR/INTA detection
- Sideset delays for timing optimization
- 32-bit protocol for ISA-compatibility

**bios.h** - BIOS ROM images:
- Turbo XT BIOS v3.1 (default, 8KB)
- Landmark Diagnostic ROM (for testing)
- Ruud's Diagnostic ROM v5.4 (for validation)

## Memory Map

```
0x00000 - 0xB7FFF : RAM (736KB) - 4-byte aligned, external PSRAM via QMI
0xB8000 - 0xBBFFF : Video RAM CGA (16KB) - 4-byte aligned
0xBC000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned
```

Reset vector at 0xFFFF0 points into ROM.

## I/O Ports

**Current emulated ports:**
- 0x00-0x0F: Intel 8237 DMA Controller
- 0x20-0x21: Intel 8259A PIC
- 0x40-0x43: Intel 8253 PIT
- 0x60: Keyboard Data Port (i8042 emulation)
- 0x61: System Control Port B (keyboard reset, speaker enable)
- 0x64: Keyboard Status Port (i8042 emulation)
- 0x81/82/83/87: DMA Page Registers
- 0x3D0-0x3DF: CGA/MC6845 registers
- 0x3F0-0x3F7: Intel 8272A FDC
- 0x3F8-0x3FF: 16550 UART (COM1)

### Adding New I/O Ports

Edit `port_read8()` and `port_write8()` in `ports.h`. Add switch cases for the port range and call the appropriate device emulation function.

## Performance Considerations

**System Clocks:**
- RP2350B: 500 MHz (overclocked with voltage boost to 1.60V)
- PIO: 500 MHz (clkdiv = 1.0, no frequency divider)
- i8086: 4.75-6 MHz (configurable, currently stable at these frequencies)

**Available Ticks & Core 1 Load Analysis:**
- **Total Ticks:** The RP2350B provides 500,000,000 ticks per second for Core 1.
- **Ticks per i8086 Cycle:** Approximately 83-105 RP2350B ticks per i8086 clock cycle (500 MHz / 6 MHz = ~83 ticks).
- **PIO Processing Time:** PIO state machine handles bus protocol in hardware with deterministic timing (2.0 ns per instruction at 500 MHz).
- **Bus Handling:** Core 1's primary critical task is servicing PIO interrupts for i8086 bus cycles.
- **Worst-Case Load:** Even at 6 MHz with continuous bus access, Core 1 load remains below 45%, leaving ample headroom for device emulation.

**Critical optimizations:**
- All bus handlers run from RAM (`__time_critical_func`)
- 4-byte aligned memory arrays for fast access
- Direct 16-bit memory access via pointer casting
- Inline functions (`__force_inline`) for hot paths
- `likely()` / `unlikely()` hints for branch prediction
- Minimal volatile variable usage (local copies in hot paths)
- PIO instruction count: 29/32 used
- Hardware repeating_timer for IRQ0 instead of polling loop
- `__wfe()` on Core1 for power efficiency when idle

## Initialization Order (CRITICAL!)

**Wrong order = i8086 starts before PIO is ready = bus conflicts!**

Core1 (bus_handler_core) initialization sequence:
```c
start_cpu_clock();   // 1. Start clock first
pic_init();          // 2. Initialize INTR pin
cpu_bus_init();      // 3. Load PIO program, setup IRQ handlers
reset_cpu();         // 4. Release i8086 from reset (MUST be last!)
```

Never call `reset_cpu()` before `cpu_bus_init()`.

## Development Guidelines

### Code Style

- Follow KISS principle (Keep It Simple, Stupid)
- Self-documenting code preferred over excessive comments
- Functions only created when code is reused
- Comments in Russian for this project
- No emojis unless explicitly requested

### Modifying PIO Bus Logic

Edit `i8086_bus.pio` carefully - timing-sensitive:
- All comments in Russian
- Test thoroughly - incorrect timing can hang i8086
- RD checked before WR in polling loop (80% of operations are reads)
- PIO program auto-recompiled via `pico_generate_pio_header()`

### USB Debug Commands (main.c)

**Special commands (uppercase, not sent to i8086):**
- **'M'**: Memory dump (interactive - prompts for address in hex)
- **'V'**: Video RAM dump (contents of VRAM at 0xB8000)
- **'R'**: Reset CPU
- **'B'**: Reboot RP2350B to bootloader mode
- **'C'**: Toggle CTTY mode (direct COM1/USB terminal, disables video)
- **'P'**: Dump CGA/CRTC register values

**Regular input:**
- All other characters converted to scancodes and sent to i8042
- Generates IRQ1 (keyboard interrupt) automatically

### IRQ System (Intel 8259A)

**IRQ priorities (Fully Nested Mode):**
- IRQ0 (timer) > IRQ1 (keyboard) > IRQ6 (floppy) > ...

**IRQ0 (Timer) - Core1:**
```c
// Hardware repeating_timer callback
static bool irq0_timer_callback(struct repeating_timer *t) {
    i8259_interrupt(0);  // Sets bit in IRR[0]
    return channel->operating_mode != 0;  // Stop timer for mode 0
}
```

**IRQ1 (Keyboard) - Core0:**
```c
push_scancode(scancode);  // Calls i8259_interrupt(1)
```

**IRQ6 (Floppy) - FDC emulation:**
```c
i8272_irq();  // Calls i8259_interrupt(6) if enabled
```

**INTR Management:**
```c
gpio_put(INTR_PIN, i8259_get_pending_irqs());  // Checks IRR & ~IMR
```

**EOI (End of Interrupt):**
- Non-specific EOI (0x20): Clears highest priority ISR bit via `__builtin_ctz()`
- Specific EOI (0x60 + IRQ): Clears only specified ISR bit
- BIOS sends 0x20 to port 0x20 after interrupt handling

### Keyboard Controller Emulation (i8042)

**Port 0x61 (System Control Port B):**
- Bit 6 (0x40): Keyboard reset trigger
- Rising edge on bit 6 generates scancode 0xAA (self-test passed) and IRQ1
- Used by diagnostic ROMs to test keyboard controller

**Port 0x60 (Keyboard Data Port):**
- Read: Returns current scancode (0 if none available)
- Write: Sets scancode to 0xAA for self-test response

## Workflow Rules

**IMPORTANT:**
- Всегда спрашивать разрешение перед коммитом
- Никогда не коммитить изменения без согласия пользователя
- Никогда не собирать проект автоматически (пользователь собирает сам)
- Никогда не добавлять намеки про наличие AI в коммиты

When user requests commit:
1. Run `git status` and `git diff`
2. Draft concise commit message in Russian
3. Ask for approval before executing commit
4. Never use `--no-verify` or skip hooks

## Testing & Validation

**Diagnostic ROMs:**
- **Landmark Diagnostic ROM**: Tests all hardware components (IRQ, PIT, keyboard, memory)
- **Ruud's Diagnostic ROM v5.4**: Comprehensive PC/XT hardware validation
- **CheckIt**: System board and peripheral testing

**Test Results (after recent interrupt fixes):**
- ✅ Landmark Diagnostic ROM: All tests pass without errors
- ✅ Ruud's Diagnostic ROM v5.4: Complete hardware check passes
- ✅ Boulder Dash: Runs correctly with keyboard input
- ✅ CheckIt: System board and keyboard tests pass
- ✅ DR-DOS 7: Successfully boots from floppy image

**Common issues fixed:**
- Intel 8253 Mode 0: Now correctly generates one-shot interrupts
- Intel 8259A EOI: Non-specific EOI clears only highest priority ISR bit
- Keyboard reset: Port 0x61 bit 6 rising edge triggers self-test response

## Hardware Specifications Reference

Machine-readable specifications are available in `docs/specs/` directory in TOON format (Token-Oriented Object Notation) for efficient token usage:

- `intel_8086_specs.toon`: CPU architecture, registers, interrupts
- `intel_8259a_specs.toon`: PIC configuration and operation
- `intel_8253_specs.toon`: PIT modes and channel configuration
- `intel_8237a_specs.toon`: DMA controller channels and modes
- `intel_8272a_specs.toon`: FDC commands and disk formats
- `i8086_architecture_reference.toon`: Complete system architecture
- `port_addresses.toon`: I/O port map for IBM PC/XT

PDF datasheets from manufacturers are in `docs/datasheets/` for detailed reference.
