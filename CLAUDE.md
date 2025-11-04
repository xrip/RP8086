# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RP8086 is a hardware-software complex that uses a Raspberry Pi Pico (RP2040) as a chipset for the Intel 8086 processor. The RP2040 acts as:
- Bus controller (via PIO state machine)
- ROM/RAM emulator (192KB RAM + 8KB BIOS)
- I/O controller (Intel 8259A, 8253, 8237, 8272A emulation)
- Clock generator (3.5 MHz PWM)

The system successfully boots DR-DOS 7 from emulated floppy disk.

## Build Commands

```bash
# Build from cmake-build-release directory
cd cmake-build-release
cmake --build .

# Output: bin/rp2040/Release/RP8086.uf2 (flashable to RP2040)
```

**Build Requirements:**
- Pico SDK (set `PICO_SDK_PATH` environment variable)
- CMake >= 3.13
- ARM GCC toolchain
- C23 standard

**Build Configuration:**
- Optimization: `-Ofast` with LTO enabled
- Binary mode: `copy_to_ram` (executes from RAM for maximum performance)
- System clock: 400 MHz (overclocked with voltage boost)
- i8086 clock: 3.5 MHz (configurable via `I8086_CLOCK_SPEED` in common.h)

## Architecture

### Two-Layer Bus Design

**Layer 1: PIO (Hardware - i8086_bus.pio)**
- Runs independently on RP2040's Programmable I/O state machine at 400 MHz (clkdiv = 1.0, no frequency divider)
- Handles all timing-critical bus operations in hardware
- Manages i8086 bus signals: ALE, RD, WR, M/IO, BHE, INTA, READY
- Controls bidirectional data bus (GPIO 0-15)
- Generates interrupts (IRQ0 for writes, IRQ1 for reads, IRQ3 for INTA)
- 32-bit protocol: ARM returns `[data:16][pindirs_mask:16]` for ISA-compatibility
- PIO clock runs at full system frequency (400 MHz) without divider for maximum timing precision

**Layer 2: ARM Cortex-M0+ (Software - cpu_bus.c)**
- Services interrupts from PIO via FIFO
- Implements memory/IO emulation logic
- All handlers use `__time_critical_func` macro to run from RAM
- Interrupts run at `PICO_HIGHEST_IRQ_PRIORITY`

### Multicore Architecture

**Core0 (Main) - User Interface & I/O:**
- System initialization (overclock to 400 MHz, USB setup)
- Launches Core1 via `multicore_launch_core1(bus_handler_core)`
- Main loop responsibilities:
  - Video rendering: Outputs VIDEORAM to terminal at 60 FPS
  - Keyboard input: Processes USB serial input and converts to scancodes
  - Debug commands: R (reset), B (bootloader), M (memory dump), V (video dump)
  - Scancode injection: ASCII → IBM PC/XT Scancode conversion

**Core1 (bus_handler_core) - Bus & IRQ Management:**
- Hardware initialization sequence (critical order):
  1. `start_cpu_clock()` - PWM clock generation
  2. `pic_init()` - INTR pin setup
  3. `cpu_bus_init()` - PIO program + IRQ handlers
  4. `reset_cpu()` - Release i8086 from reset (MUST be last)
- Main loop:
  - Timer IRQ generation: IRQ0 every ~54.925ms (18.2 Hz, IBM PC compatible)
  - INTR signal management: Sets INTR=HIGH when pending interrupts exist

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
- GPIO 28: RESET (Reset output, active LOW)
- GPIO 29: CLK (Clock output, PWM)

**Important:** Pin assignments in `i8086_bus.pio` must match hardware wiring.

## Code Organization

**main.c** - Entry point and Core0 main loop:
- System initialization and USB setup
- 60 FPS video rendering (MDA 25×80 text mode)
- Keyboard input processing (ASCII → Scancode conversion)
- Debug commands (uppercase): R, B, M, V

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
- Device structures: `i8259_s`, `i8253_s`, `dma_channel_s`, `i8272_s`
- Helper functions: `write_to()` for BHE support

**memory.h** - Memory emulation:
- `memory_read()` / `memory_write()`: Handles RAM/ROM/VIDEORAM access
- 4-byte aligned arrays for optimal performance

**ports.h** - I/O ports routing:
- `port_read()` / `port_write()`: 16-bit operations with BHE support
- Direct routing to device emulation functions (no port array)

**hardware/i8259.h** - Intel 8259A PIC:
- Full ICW1-ICW4 and OCW1-OCW3 support
- Fully Nested Mode with automatic priorities
- Functions: `i8259_read()`, `i8259_write()`, `i8259_interrupt()`, `i8259_nextirq()`

**hardware/i8253.h** - Intel 8253 PIT:
- 3 independent channels with reload values
- Access modes: LOBYTE, HIBYTE, TOGGLE, LATCHCOUNT
- Dynamic timer_interval calculation for channel 0

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

**i8086_bus.pio** - PIO state machine:
- Captures 20-bit address + control signals
- Optimized polling for RD/WR/INTA detection
- Sideset delays for timing optimization
- 32-bit protocol for ISA-compatibility

**bios.h** - Turbo XT BIOS v3.1 ROM image (8KB)

## Memory Map

```
0x00000 - 0x2FFFF : RAM (192KB) - 4-byte aligned
0x30000 - 0xAFFFF : Unmapped (returns 0xFFFF)
0xB0000 - 0xB0FFF : Video RAM MDA (4KB) - 4-byte aligned
0xB1000 - 0xFDFFF : Unmapped
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned
```

Reset vector at 0xFFFF0 points into ROM.

## I/O Ports

**Current emulated ports:**
- 0x00-0x0F: Intel 8237 DMA Controller
- 0x20-0x21: Intel 8259A PIC
- 0x40-0x43: Intel 8253 PIT
- 0x60: Keyboard Data Port
- 0x61: System Control Port B
- 0x64: Keyboard Status Port
- 0x81/82/83/87: DMA Page Registers
- 0x3BA: VGA Status Register
- 0x3F0-0x3F7: Intel 8272A FDC

### Adding New I/O Ports

Edit `port_read8()` and `port_write8()` in `ports.h`. Add switch cases for the port range and call the appropriate device emulation function.

## Performance Considerations

**System Clocks:**
- RP2040: 400 MHz (overclocked with voltage boost)
- PIO: 400 MHz (clkdiv = 1.0, no frequency divider)
- i8086: 3.5 MHz (configurable, currently stable at this frequency)

**Available Ticks & Core 1 Load Analysis:**
- **Total Ticks:** The RP2040 provides 400,000,000 ticks per second for Core 1.
- **Ticks per i8086 Cycle:** There are approximately 114 RP2040 ticks for every 1 i8086 clock cycle (400 MHz / 3.5 MHz = ~114 ticks).
- **PIO Processing Time:** PIO state machine handles bus protocol in hardware with deterministic timing (2.5 ns per instruction at 400 MHz).
- **Bus Handling:** Core 1's primary critical task is servicing PIO interrupts for i8086 bus cycles. The time spent in these interrupt handlers dictates the number of wait states for the i8086.
- **Worst-Case Load Calculation:**
  - The i8086, at 3.5 MHz, can initiate a theoretical maximum of 875,000 bus cycles per second (assuming 4 clock cycles per bus access).
  - A conservative estimate for the `bus_read_handler` execution time is ~200 RP2040 clock cycles.
  - Total ticks consumed by handlers per second: `875,000 bus_cycles/sec * 200 ticks/cycle = 175,000,000` ticks.
  - **Core 1 Load:** `(175,000,000 / 400,000,000) * 100% = 43.75%`.

**Conclusion:**
- Even in a theoretical worst-case scenario of constant bus access, **Core 1 is loaded at less than 45%**.
- This leaves **over 55% of Core 1's processing power in reserve** for more complex device emulation or future features.
- The background tasks on Core 1 (18.2 Hz timer) are negligible.
- The current implementation is highly efficient, ensuring stable operation with ample performance headroom.

**Critical optimizations:**
- All bus handlers run from RAM (`__time_critical_func`)
- 4-byte aligned memory arrays for fast access
- Direct 16-bit memory access via pointer casting
- Inline functions (`__force_inline`) for hot paths
- `likely()` / `unlikely()` hints for branch prediction
- Minimal volatile variable usage (local copies in hot paths)
- PIO instruction count: 29/32 used

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
- **'M'**: Memory dump (interactive - prompts for address)
- **'V'**: Video RAM dump (first 5 lines × 80 characters)
- **'R'**: Reset CPU
- **'B'**: Reboot RP2040 to bootloader mode

**Regular input:**
- All other characters converted to scancodes and sent to i8086
- Generates IRQ1 (keyboard interrupt) automatically

### IRQ System (Intel 8259A)

**IRQ priorities (Fully Nested Mode):**
- IRQ0 (timer) > IRQ1 (keyboard) > IRQ6 (floppy) > ...

**IRQ0 (Timer) - Core1:**
```c
i8259_interrupt(0);  // Sets bit in IRR[0]
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
- BIOS sends 0x20 to port 0x20 (non-specific EOI)
- i8259 clears ISR, allows next interrupt

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
