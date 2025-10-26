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
- Optimization flags: -Ofast, -ffunction-sections, -fdata-sections
- Binary mode: copy_to_ram (executes from RAM for maximum performance)

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

**Core1 (IRQ Generator + Log Processor):**
- Runs `core1_irq_generator()` in infinite loop
- Generates IRQ0 every ~5492ms (adapted for 500 KHz CPU) using `absolute_time_t` timing
  - At 5 MHz: ~54.925ms (18.2 Hz, IBM PC compatible)
  - At 500 KHz: ~5492ms (for stable operation during development)
- Sets INTR=HIGH when IRQ pending
- **Asynchronous logging**: Processes log events from Core0 via multicore FIFO
- **Real-time video output**: Displays video memory writes to terminal via ANSI escape sequences
- **Benefits**: Offloads both IRQ generation and non-critical I/O from Core0
- **Synchronization**: Uses multicore FIFO for lockless communication
- **IBM PC compatible**: Matches standard 8253/8254 PIT frequency when running at full speed

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
- System clock setup (400 MHz overclock with voltage boost)
- USB serial init with delays
- Initialization sequence: `start_cpu_clock()` → `reset_cpu()` → `pic_init()` → `cpu_bus_init()`
- **Core1 IRQ generator**: Generates timer interrupts + processes async logs
- **Multicore logging system**: Shared buffer for bus event logging
- WFI loop with `tight_loop_contents()` hint
- USB command processor (M/V/P/R/B)

**cpu.c/h** - i8086 CPU control:
- `start_cpu_clock()`: PWM generation for i8086 clock (33% duty cycle)
- `reset_cpu()`: RESET sequence (10 clocks LOW, 5 clocks stabilization)

**cpu_bus.c/h** - Bus controller and memory emulation:
- `cpu_bus_init()`: Loads PIO program, sets up IRQ handlers
- `bus_read_handler()`: Services read requests (IRQ1)
- `bus_write_handler()`: Services write requests (IRQ0)
- `i8086_read()`: Highly optimized 16-bit memory/IO reads with aligned access
- `i8086_write()`: Highly optimized 16-bit memory/IO writes with aligned access
- 128KB RAM buffer (4-byte aligned), 8KB ROM (Turbo XT BIOS v3.1, 4-byte aligned)
- 4KB Video RAM (MDA text mode, 0xB0000-0xB7FFF)
- **Asynchronous logging**: `log_event()` function sends events to Core1 via FIFO
- **Performance optimization**: Address alignment via `address & ~1U` for single 16-bit access
- **FIFO efficiency**: Eliminated conditional branches in critical paths
- **Port emulation**: VGA status register at 0x3BA with vsync bit toggling

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
- i8086 clock frequency (500 KHz, changeable to 5 MHz for production)
- PIO and IRQ settings
- INTR/INTA/READY pin definitions
- **Logging structures**: `log_entry_t`, `shared_log_buffer_t` for async event logging
- **Circular buffer**: 256-entry log buffer with timestamp support

**pic.c/h** - 8259A PIC emulation (ИНТЕГРИРОВАНО В main.c/cpu_bus.c):
- Функционал PIC интегрирован в `main.c` и `cpu_bus.c`
- `pic_init()`: Инициализирует INTR пин, очищает FIFO, запускает Core1
- `core1_irq_generator()`: Работает на Core1, выполняет две функции:
  - Генерирует IRQ0 каждые ~5492ms (адаптировано для 500 КГц)
  - Обрабатывает логи из FIFO и выводит их асинхронно
  - В режиме 5 МГц: ~54.925ms (18.2 Hz, IBM PC стандарт)
- **INTA обработка через PIO**: В `i8086_bus.pio` реализован `INTA_cycle`
  - Использует IRQ 3 для синхронизации INTA протокола
  - Автоматически управляет READY сигналом во время INTA
  - Отправляет вектор прерывания (0x08 для IRQ0) через bus_read_handler

**bios.h** - Turbo XT BIOS v3.1 (10/28/2017) ROM image (8KB array)

**Видеопамять (НОВОЕ):**
- `videoram[4096]`: 4KB видеопамять для текстового режима
- Адресация: 0xB0000-0xB8000 (мониторный режим MDA)
- Поддержка 16-битных и 8-битных операций записи
- Отображение через USB командой 'V' в main.c

## Current Implementation Status

**Implemented (Фаза 1 complete + performance optimizations + interrupts + видео + логирование):**
- ✅ Clock generation on GPIO29 (PWM, 500 KHz, 33% duty cycle)
- ✅ RESET sequence on GPIO28
- ✅ Highly optimized PIO bus controller for i8086 (16-bit data bus, 20-bit address)
- ✅ ROM emulation: 8KB Turbo XT BIOS v3.1 at 0xFE000-0xFFFFF (4-byte aligned)
- ✅ RAM emulation: 128KB at 0x00000-0x1FFFF (4-byte aligned)
- ✅ **Видеопамять**: 4KB at 0xB0000-0xB7FFF (MDA текстовый режим)
- ✅ Memory vs I/O address decoding (M/IO signal)
- ✅ **Ultra-fast 16-bit bus operations** via aligned memory access
- ✅ **BHE handling**: Полная поддержка 8/16 битных операций
- ✅ **INTA через PIO**: Обработка прерываний через state machine (IRQ 3)
- ✅ **INTR на GPIO26**: Генерация прерываний Core1
- ✅ **Core1 IRQ generator + log processor** (асинхронный вывод логов)
- ✅ **Асинхронное логирование**: Multicore FIFO, кольцевой буфер на 256 записей
- ✅ **Real-time video output**: Вывод видеопамяти в терминал через ANSI
- ✅ **VGA port emulation**: Порт 0x3BA с vsync битами
- ✅ **Compiler optimizations**: -Ofast, copy_to_ram, size optimizations
- 🚀 **100x performance boost**: 5 KHz → 500 KHz (готов к 5 MHz)

**Not yet implemented (Фаза 2):**
- ⚠️ Full 8259A register interface (ICW1-ICW4, OCW1-OCW3)
- ⚠️ Additional I/O devices (UART, PIT hardware emulation)
- ⚠️ DMA controller
- ⚠️ Full speed operation (5 MHz+)
- ⚠️ Additional video modes (CGA/EGA compatibility)

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
- Порт 0x3BA: VGA status register (эмуляция vsync битов)
- Порты 0x000-0xFFF: Общие порты (возвращают 0xFFFF для неопределенных портов)

### USB Commands (main.c)

- **'M'**: Дамп памяти (первые 400 байт RAM)
- **'V'**: Дамп видеопамяти (первые 5 строк x 80 символов)
- **'P'**: Дамп портов ввода-вывода (первые 400 байт)
- **'R'**: Сброс CPU (очищает INTR и выполняет reset_cpu())
- **'B'**: Перезагрузка RP2040 в bootloader mode

**Note**: Видеопамять автоматически выводится в терминал в real-time через Core1

### Modifying PIO Bus Logic

Edit `i8086_bus.pio` carefully - PIO assembly is timing-sensitive:
- All comments in Russian
- Test thoroughly - incorrect timing can hang i8086
- PIO program is recompiled automatically via `pico_generate_pio_header()`
- RD is checked before WR in polling loop (80% of operations are reads)

### Performance Considerations

**Current performance @ 400 MHz / 500 KHz i8086:**
- Available: 3200 RP2040 ticks per i8086 bus cycle (at 500 KHz)
- **Required with optimizations:** ~75 RP2040 ticks (40% improvement!)
- Reserve: 42x (extremely safe, ready for 5 MHz operation)
- CPU load: ~2.5% worst case at 500 KHz
- **At 5 MHz**: 320 ticks available, ~75 required, 4.2x reserve, ~25% CPU load

**Performance achievements:**
- Successfully running at 500 KHz (100x faster than initial 5 KHz)
- Stable BIOS execution with vsync port emulation
- Real-time video output to terminal
- Zero blocking in IRQ handlers (async logging via FIFO)

**Recent performance optimizations:**
- ✅ **Address alignment optimization**: Single 16-bit memory access via `address & ~1U`
- ✅ **Memory alignment**: 4-byte aligned arrays (`__attribute__((aligned(4)))`)
- ✅ **Eliminated conditional branches**: Direct memory access without boundary checks
- ✅ **Optimized PIO timing**: Reduced instruction count and sideset delays
- ✅ **Compiler optimizations**: -Ofast, -ffunction-sections, -fdata-sections
- ✅ **RAM execution**: copy_to_ram binary mode for maximum speed
- ✅ **Asynchronous logging**: Non-blocking event logging via multicore FIFO
- ✅ **Reduced RAM**: 224KB → 128KB to free memory for logging buffers

**General optimization tips:**
- IRQ handlers run at highest priority (`PICO_HIGHEST_IRQ_PRIORITY`)
- Use `__time_critical_func` for all bus handlers (placed in RAM)
- PIO FIFO depth is 4 entries - handlers process extremely fast now
- Memory arrays are 4-byte aligned in SRAM (maximum access speed)

### Initialization Order (Critical!)

```c
start_cpu_clock();   // 1. Start clock first (PWM on GPIO29)
reset_cpu();         // 2. Reset CPU (hold in reset while setting up)
pic_init();          // 3. Initialize interrupt controller, clear FIFO, start Core1
cpu_bus_init();      // 4. Initialize PIO and IRQ handlers last
// CPU is now ready - it will start executing after reset is released
```

Wrong order = i8086 starts before PIO is ready = bus conflicts!

**Important notes:**
- `pic_init()` launches Core1 and initializes the logging system
- `pic_init()` clears multicore FIFO to prevent stale data
- `cpu_bus_init()` must be last to ensure Core1 is ready for FIFO communication
- Core1 starts immediately: generates IRQ0 + processes async logs
- Reset clears INTR signal before CPU starts

### Memory Map

```
0x00000 - 0x1FFFF : RAM (128KB) - **ОПТИМИЗИРОВАНО**
0x20000 - 0xAFFFF : Unmapped (returns 0xFFFF)
0xB0000 - 0xB7FFF : Video RAM MDA (4KB)
0xB8000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB)
```

Reset vector at 0xFFFF0 → points into ROM.

### Logging System Architecture

**Асинхронное логирование событий шины:**
- Core0 (IRQ handlers) → записывают события в кольцевой буфер
- Core0 → Core1: отправка индекса через multicore FIFO (неблокирующая)
- Core1 → читает события из буфера, форматирует и выводит через USB
- **Преимущества**: Нулевая задержка в IRQ handlers, нет блокировок printf

**Структуры данных:**
```c
typedef struct {
    uint64_t timestamp;   // Счетчик событий для отладки
    log_type_t type;      // LOG_READ, LOG_WRITE, LOG_INTA
    uint32_t address;     // 20-битный адрес
    uint16_t data;        // 16-битные данные
    bool bhe;             // Состояние BHE
    bool mio;             // Состояние MIO
} log_entry_t;

typedef struct {
    log_entry_t buffer[256];      // Кольцевой буфер
    volatile uint32_t head;       // Указатель записи
} shared_log_buffer_t;
```

**Особенности реализации:**
- Только события видеопамяти (0xB0000-0xB7FFF) логируются для оптимизации
- FIFO заполняется только при наличии места (неблокирующая проверка)
- Core1 выводит символы в терминал через ANSI escape sequences
- Формат вывода: `\x1b[row;col]Hchar` для позиционирования
