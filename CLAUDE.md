# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RP8086 is a hardware-software complex that uses a Chinese purple RP2040 board (Raspberry Pi Pico) as a chipset for the Intel 8086 processor. The RP2040 acts as:
- Bus controller
- ROM/RAM emulator
- I/O controller
- Clock generator (planned)

The RP2040 is 5V tolerant on inputs, so no level shifters are needed for direct connection to the i8086.

## Build Commands

```bash
# Build from cmake-build-release directory
cd cmake-build-release
cmake --build .

# Output: bin/rp2040/Release/MultiIO.uf2 (flashable to RP2040)
```

The project uses:
- Pico SDK
- CMake >= 3.13
- GCC ARM toolchain
- C23 standard

## Architecture

### Two-Layer Design

The project uses a unique two-layer architecture that splits bus handling between hardware and software:

**Layer 1: PIO (Hardware - i8086_bus.pio)**
- Runs independently on RP2040's Programmable I/O state machine
- Handles all timing-critical bus operations in hardware
- Manages i8086 bus signals: ALE, RD, WR, M/IO, BHE, READY
- Controls bidirectional data bus (GPIO 0-15)
- Generates interrupts (IRQ0 for writes, IRQ1 for reads)
- Optimized timing with sideset delays and reduced instruction count
- Can run at up to 133 MHz for deterministic timing

**Layer 2: ARM Cortex-M0+ (Software - cpu_bus.c)**
- Services interrupts from PIO via FIFO
- Implements actual memory/IO emulation logic
- All handlers use `__time_critical_func` macro to run from RAM
- Interrupts run at `PICO_HIGHEST_IRQ_PRIORITY`

### Multicore Architecture

The project uses RP2040's dual-core capability for interrupt handling:

**Core0 (Main):**
- Runs main loop with `__wfi()` (Wait For Interrupt)
- Services PIO interrupts (bus read/write via IRQ0/IRQ1)
- Handles USB serial communication
- Processes user commands (reset, memory dump, etc.)

**Core1 (IRQ Generator):**
- Runs `core1_irq_generator()` in infinite loop
- Generates IRQ0 every ~54.925ms (18.2 Hz) using `absolute_time_t` timing
- Sets INTR=HIGH when IRQ pending
- **Benefits**: Offloads timing-critical interrupt generation from Core0
- **Synchronization**: Uses `volatile bool irq_pending` flag
- **IBM PC compatible**: Matches standard 8253/8254 PIT frequency (1.193182 MHz / 65536)

**INTA Protocol (PIO State Machine - ИЗМЕНЕНО):**
- **INTA теперь обрабатывается в PIO**: В `i8086_bus.pio` добавлен `INTA_cycle`
- **Процесс INTA**:
  1. PIO обнаруживает INTA=LOW после ALE
  2. Генерирует IRQ 3 и устанавливает READY=1 (wait state)
  3. Ждет завершения INTA цикла (INTA=HIGH → INTA=LOW)
  4. Передает управление в обычный цикл чтения
- **Обработка в ARM**: `bus_read_handler()` получает IRQ 3
  - Устанавливает флаг `irq_pending1 = true`
  - При следующем чтении возвращает вектор прерывания (0x08)
  - Автоматически сбрасывает INTR=0

**Почему INTA через PIO (а не GPIO прерывание)?**
- INTA теперь часть основного state machine
- Минимальная задержка реагирования
- Автоматическая синхронизация с шинными циклами
- Упрощенная логика обработки в ARM коде

### Bus Protocol Flow

**Read Cycle:**
1. PIO latches address on ALE signal
2. PIO raises WAIT (READY=1) to hold CPU
3. PIO triggers IRQ1 when RD active
4. ARM handler reads address from FIFO, returns data
5. PIO outputs data and drops WAIT (READY=0)
6. PIO returns data bus to high-Z after RD inactive

**Write Cycle:**
1. PIO latches address on ALE signal
2. PIO polls RD/WR signals and determines operation type
3. For write: PIO captures 16-bit data from AD0-AD15
4. PIO triggers IRQ0 with optimized timing (side 0 [3] delay)
5. ARM handler processes address+data from aligned memory access

### GPIO Pin Mapping (config.h)

Critical bus signals are hardcoded in `i8086_bus.pio`:
- GPIO 0-15: AD0-AD15 ( multiplexed Address/Data bus, bidirectional)
- GPIO 16-19: A16-A19 (Address bus upper 4 bits)
- GPIO 20: ALE (Address Latch Enable)
- GPIO 21: RD (Read strobe, active LOW)
- GPIO 22: WR (Write strobe, active LOW)
- GPIO 23: INTA (Interrupt acknowledge input from i8086, active LOW) - **НОВОЕ**
- GPIO 24: M/IO (Memory/IO select: 1=memory, 0=I/O)
- GPIO 25: BHE (Bus High Enable)
- GPIO 26: INTR (Interrupt request output to i8086, active HIGH) - **ИЗМЕНЕНО**
- GPIO 27: READY (Wait state control output to CPU)
- GPIO 28: RESET (Reset output, active LOW)
- GPIO 29: CLK (Clock output to CPU)

**Important:** Pin assignments in the `.define` section of `i8086_bus.pio` must match the actual hardware wiring. Changing them requires understanding PIO's pin constraints.


## Code Organization

**main.c** - Entry point:
- System clock setup (400 MHz overclock)
- USB serial init with delays
- Initialization sequence: `start_cpu_clock()` → `cpu_bus_init()` → `reset_cpu()`
- WFI loop with `tight_loop_contents()` hint

**cpu.c/h** - i8086 CPU control:
- `start_cpu_clock()`: PWM generation for i8086 clock (33% duty cycle)
- `reset_cpu()`: RESET sequence (10 clocks LOW, 5 clocks stabilization)

**cpu_bus.c/h** - Bus controller and memory emulation:
- `cpu_bus_init()`: Loads PIO program, sets up IRQ handlers
- `bus_read_handler()`: Services read requests (IRQ1)
- `bus_write_handler()`: Services write requests (IRQ0)
- `cpu_bus_read()`: Highly optimized 16-bit memory/IO reads with aligned access
- `cpu_bus_write()`: Highly optimized 16-bit memory/IO writes with aligned access
- 64KB RAM buffer (4-byte aligned), 8KB ROM (GlaBIOS, 4-byte aligned)
- **Performance optimization**: Address alignment via `address & ~1U` for single 16-bit access
- **FIFO efficiency**: Eliminated conditional branches in critical paths

**i8086_bus.pio** - Highly optimized PIO state machine implementing i8086 bus protocol:
- Captures 20-bit address + control signals (25 GPIO)
- **Optimized polling**: Reduced cycle count in RD/WR detection loop
- **Efficient timing**: Sideset delays combined with IRQ operations
- **Smart data handling**: Uses single 16-bit aligned memory access
- **Consistent naming**: Standardized WR_cycle/RD_cycle labels
- C initialization function at bottom (`i8086_bus_program_init`)

**config.h** - Hardware configuration:
- GPIO pin assignments
- System clock (400 MHz)
- i8086 clock frequency (100 Hz for debug, changeable to 5 MHz)
- PIO and IRQ settings
- INTR/INTA/READY pin definitions

**pic.c/h** - 8259A PIC emulation (ИЗМЕНЕНО):
- Функционал PIC перенесен в `main.c` и `cpu_bus.c`
- `pic_init()`: Инициализирует INTR пин и запускает Core1
- `core1_irq_generator()`: Работает на Core1, генерирует IRQ0 каждые ~549ms (для отладки на 5 КГц)
  - В обычном режиме: ~54.925ms (18.2 Hz, IBM PC стандарт)
- **INTA обработка через PIO**: В `i8086_bus.pio` добавлен `INTA_cycle`
  - Использует IRQ 3 для синхронизации INTA протокола
  - Автоматически управляет READY сигналом во время INTA
  - Отправляет вектор прерывания (0x08 для IRQ0) через CPU_BUS_READ

**bios.h** - GlaBIOS ROM image (8KB array)

**Видеопамять (НОВОЕ):**
- `videoram[4096]`: 4KB видеопамять для текстового режима
- Адресация: 0xB0000-0xB8000 (мониторный режим MDA)
- Поддержка 16-битных и 8-битных операций записи
- Отображение через USB командой 'V' в main.c

## Current Implementation Status

**Implemented (Фаза 1 complete + performance optimizations + interrupts + видео):**
- ✅ Clock generation on GPIO29 (PWM, 100 Hz debug mode, 33% duty)
- ✅ RESET sequence on GPIO28 (**ИЗМЕНЕНО** с GPIO27)
- ✅ Highly optimized PIO bus controller for i8086 (16-bit data bus, 20-bit address)
- ✅ ROM emulation: 8KB GlaBIOS at 0xFE000-0xFFFFF (4-byte aligned)
- ✅ RAM emulation: 224KB at 0x00000-0x37FFF (**УВЕЛИЧЕНО** с 64KB)
- ✅ **Видеопамять**: 4KB at 0xB0000-0xB8000 (MDA текстовый режим) - **НОВОЕ**
- ✅ Memory vs I/O address decoding (M/IO signal)
- ✅ **Ultra-fast 16-bit bus operations** via aligned memory access
- ✅ **BHE handling**: Полная поддержка 8/16 битных операций
- ✅ **INTA через PIO**: Обработка прерываний через state machine (IRQ 3) - **НОВОЕ**
- ✅ **INTR на GPIO26**: Генерация прерываний Core1 - **ИЗМЕНЕНО**
- ✅ **Core1 IRQ generator** (18.2 Hz timer, IBM PC совместимый)
- 🚀 **40-60% performance boost** with address alignment optimization

**Not yet implemented:**
- ⚠️ Full 8259A register interface (ICW1-ICW4, OCW1-OCW3)
- ⚠️ Additional I/O devices (UART, PIT)
- ⚠️ DMA controller
- ⚠️ **Раздельный файл pic.c** (функционал интегрирован в main.c/cpu_bus.c)

## Development Guidelines

### Code Style

Follow KISS principle (Keep It Simple, Stupid):
- Self-documenting code preferred over excessive comments
- Functions should only be created when code is reused
- Avoid premature abstraction
- Comments in Russian for this project

### Adding New I/O Ports

Add cases to `i8086_read()` and `i8086_write()` in `cpu_bus.c`. These functions receive:
- 20-bit address (for I/O use lower 16 bits as port number)
- is_memory_access flag (true for memory, false for I/O)
- bhe flag для 8/16 битных операций

**Текущие порты:**
- Порт 0x40, 0x41, 0x61, 0x21: Эмуляция PIT/PIC (возвращают 0x0000)
- Ports array: 0x000-0xFFF для общего использования (через `ports[]`)

### USB Commands (main.c)

- **'M'**: Дамп памяти (первые 400 байт)
- **'V'**: Дамп видеопамяти (160x25 символов) - **НОВОЕ**
- **'P'**: Дамп портов ввода-вывода (первые 400 байт)
- **'R'**: Сброс CPU
- **'B'**: Перезагрузка в bootloader mode

### Modifying PIO Bus Logic

Edit `i8086_bus.pio` carefully - PIO assembly is timing-sensitive:
- All comments in Russian
- Test thoroughly - incorrect timing can hang i8086
- PIO program is recompiled automatically via `pico_generate_pio_header()`
- RD is checked before WR in polling loop (80% of operations are reads)

### Performance Considerations

**Current performance @ 400 MHz / 5 MHz i8086:**
- Available: 320 RP2040 ticks per i8086 bus cycle
- **Required with optimizations:** ~75 RP2040 ticks (40% improvement!)
- Reserve: 4.2x (very safe)
- CPU load: ~25% worst case (significant reduction)

**Recent performance optimizations:**
- ✅ **Address alignment optimization**: Single 16-bit memory access via `address & ~1U`
- ✅ **Memory alignment**: 4-byte aligned arrays (`__attribute__((aligned(4)))`)
- ✅ **Eliminated conditional branches**: Direct memory access without boundary checks
- ✅ **Optimized PIO timing**: Reduced instruction count and sideset delays

**General optimization tips:**
- IRQ handlers run at highest priority (`PICO_HIGHEST_IRQ_PRIORITY`)
- Use `__time_critical_func` for all bus handlers (placed in RAM)
- PIO FIFO depth is 4 entries - handlers process extremely fast now
- Memory arrays are 4-byte aligned in SRAM (maximum access speed)

### Initialization Order (Critical!)

```c
start_cpu_clock();   // 1. Start clock first
cpu_bus_init();      // 2. Initialize PIO and handlers
pic_init();          // 3. Initialize interrupt controller, start Core1 IRQ generator
reset_cpu();         // 4. Release i8086 from reset last
```

Wrong order = i8086 starts before PIO is ready = bus conflicts!

**Important notes:**
- `pic_init()` must be called AFTER `cpu_bus_init()` (needs BUS_CTRL_PIO/BUS_CTRL_SM)
- `pic_init()` launches Core1, so call it BEFORE `reset_cpu()` (interrupts ready)
- Core1 starts generating IRQ0 immediately, but INTR stays LOW until after reset

### Memory Map

```
0x00000 - 0x37FFF : RAM (224KB) - **УВЕЛИЧЕНО**
0x38000 - 0xAFFFF : Unmapped (returns 0xFFFF)
0xB0000 - 0xB7FFF : Video RAM MDA (4KB) - **НОВОЕ**
0xB8000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM GlaBIOS (8KB)
```

Reset vector at 0xFFFF0 → points into ROM.
