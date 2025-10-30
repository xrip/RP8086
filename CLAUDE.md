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

The project uses RP2040's dual-core capability with clearly separated responsibilities:

**Core0 (Main) - User Interface & I/O:**
- System initialization (overclock to 400 MHz, USB setup)
- Launches Core1 via `multicore_launch_core1(bus_handler_core)`
- Main loop responsibilities:
  - **Video rendering**: Outputs VIDEORAM to terminal at 60 FPS (every 16.666ms)
  - **Keyboard input**: Processes USB serial input and converts to scancodes
  - **Debug commands**: Handles uppercase commands (R, B, M, V, P)
  - **Scancode injection**: ASCII → IBM PC/XT Scancode conversion via `push_scancode()`
- No WFI - runs continuously with `tight_loop_contents()` hint

**Core1 (bus_handler_core) - Bus & IRQ Management:**
- Hardware initialization sequence:
  1. `start_cpu_clock()` - PWM clock generation
  2. `pic_init()` - INTR pin setup
  3. `cpu_bus_init()` - PIO program + IRQ handlers
  4. `reset_cpu()` - Release i8086 from reset
- Main loop responsibilities:
  - **Timer IRQ generation**: IRQ0 every 54.925ms (18.2 Hz, IBM PC 8253/8254 compatible)
  - **INTR signal management**: Sets INTR=HIGH when `current_irq_vector != 0`
  - **Priority handling**: IRQ0 (timer) has implicit priority over IRQ1 (keyboard)

**IRQ Handlers (Both Cores):**
- `bus_read_handler()` and `bus_write_handler()` run at `PICO_HIGHEST_IRQ_PRIORITY`
- Triggered by PIO interrupts (independent of which core is active)
- Handle i8086 bus transactions in real-time
- INTA protocol handled via PIO IRQ 3

**INTA Protocol (PIO State Machine):**
- **INTA обрабатывается в PIO**: В `i8086_bus.pio` реализован `INTA_cycle`
- **Процесс INTA**:
  1. PIO обнаруживает INTA=LOW после ALE
  2. Генерирует IRQ 3 и устанавливает READY=1 (wait state)
  3. Ждет завершения INTA цикла (INTA=HIGH → INTA=LOW)
  4. Передает управление в обычный цикл чтения
- **Обработка в ARM**: `bus_read_handler()` получает IRQ 3
  - Устанавливает статический флаг `irq_pending = true`
  - Сбрасывает INTR=0 через `gpio_put(INTR_PIN, 0)`
  - При следующем чтении возвращает `current_irq_vector` (0xFF08 для IRQ0, 0xFF09 для IRQ1)
  - Очищает вектор: `irq_pending = current_irq_vector = 0`

**Почему INTA через PIO (а не GPIO прерывание)?**
- INTA интегрирован в основной state machine
- Минимальная задержка реагирования (hardware-level)
- Автоматическая синхронизация с шинными циклами
- Упрощенная логика обработки в ARM коде

### Bus Protocol Flow

**Read Cycle (with ISA-compatibility protocol):**
1. PIO latches address on ALE signal
2. PIO raises WAIT (READY=0) to hold CPU
3. PIO triggers IRQ1 when RD active
4. ARM handler reads address from FIFO
5. **ARM sends 32-bit response: [data:16][pindirs_mask:16]**
   - Current implementation: always `(data << 16) | 0xFFFF` (all addresses handled by RP2040)
   - Future ISA mode: `0x00000000` for addresses handled by external ISA devices
6. PIO applies pindirs mask (0xFFFF → outputs, 0x0000 → high-Z for ISA devices)
7. PIO outputs data and drops WAIT (READY=1)
8. PIO returns data bus to high-Z after RD inactive

**32-bit Protocol Format:**
```
Bits [31:16] = Data to output on AD0-AD15
Bits [15:0]  = Pin direction mask (0xFFFF = our address, 0x0000 = ISA device)
```

**ISA-Compatibility Design:**
- When mask = 0x0000: AD0-AD15 remain in high-Z (input mode)
- External ISA devices can drive the bus without conflict
- READY signal still controlled by PIO (limitation: need external logic for full ISA support)
- Future enhancement: external pull-up resistors on AD0-AD15 for proper open-collector behavior

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

**main.c** - Entry point and user interface:
- System clock setup (400 MHz overclock with voltage boost)
- USB serial init (waits for connection via `stdio_usb_connected()`)
- Launches Core1: `multicore_launch_core1(bus_handler_core)`
- **Core0 main loop**:
  - 60 FPS video rendering (25×80 MDA text mode via ANSI escape codes)
  - USB keyboard input processing
  - Debug commands (uppercase): R (reset), B (bootloader), M/V/P (dumps)
  - ASCII → Scancode conversion via `ascii_to_scancode()`
  - Scancode buffering via `push_scancode()`
- **IRQ System**: Manages `current_irq_vector` (shared with cpu_bus.c)
- **Keyboard buffer**: 16-byte circular buffer for scancodes

**cpu.c/h** - i8086 CPU control:
- `start_cpu_clock()`: PWM generation for i8086 clock (33% duty cycle)
- `reset_cpu()`: RESET sequence (10 clocks LOW, 5 clocks stabilization)

**cpu_bus.c/h** - Bus controller and memory emulation:
- `cpu_bus_init()`: Loads PIO program, sets up IRQ handlers at highest priority
- `bus_read_handler()`: Services read requests (IRQ1) and INTA cycles (IRQ3)
  - Returns 32-bit value: `(data << 16) | 0xFFFF` (all addresses currently handled by RP2040)
  - Protocol ready for ISA expansion: can return `0x00000000` for external device addresses
  - INTA cycle: вызывает `i8259_nextirq()` для получения вектора прерывания
- `bus_write_handler()`: Services write requests (IRQ0)
- `i8086_read()`: Highly optimized 16-bit memory/IO reads with `__force_inline`
- `i8086_write()`: Highly optimized 16-bit memory/IO writes with `__force_inline`
- **Memory layout**:
  - 192KB RAM (4-byte aligned, 0x00000-0x2FFFF)
  - 8KB ROM - Turbo XT BIOS v3.1 (4-byte aligned, 0xFE000-0xFFFFF)
  - 4KB Video RAM - MDA text mode (4-byte aligned, 0xB0000-0xB7FFF)
- **Performance optimization**:
  - Direct 16-bit aligned access: `*(uint16_t *)&RAM[address]`
  - Local copies of volatile variables to minimize memory access

**common.h** - Common definitions and structures:
- `i8259_s`: структура для Intel 8259A PIC (IMR, IRR, ISR, ICW steps, vector offset)
- `i8253_s`: структура для Intel 8253 PIT (3 channels, reload values, access modes, frequencies)
- `write_to()`: inline функция для записи с поддержкой BHE

**memory.h** - Memory emulation:
- `memory_read()`: Чтение из RAM/ROM/VIDEORAM с выравниванием адресов
- `memory_write()`: Запись в RAM/VIDEORAM с поддержкой BHE

**ports.h** - I/O ports routing and device emulation:
- `port_read8()` / `port_write8()`: 8-битные операции для отдельных устройств
- `port_read()` / `port_write()`: 16-битные операции с поддержкой BHE и A0
- **Прямая маршрутизация портов** (switch/case):
  - 0x20-0x21 → i8259_read() / i8259_write()
  - 0x40-0x43 → i8253_read() / i8253_write()
  - 0x60, 0x64 → Keyboard emulation
  - 0x3BA → VGA status register

**hardware/i8259.h** - Intel 8259A PIC (Programmable Interrupt Controller):
- `i8259_read()`: Чтение регистров (IMR, IRR/ISR в зависимости от OCW3)
- `i8259_write()`: Запись ICW1-ICW4, OCW1-OCW3
- `i8259_interrupt()`: Установка бита в IRR для заданного IRQ
- `i8259_nextirq()`: Получение вектора с наивысшим приоритетом, перемещение IRR → ISR
- `i8259_get_pending_irqs()`: Возвращает IRR & ~IMR (немаскированные прерывания)
- **Полная поддержка**:
  - ICW1-ICW4 (Initialization Command Words)
  - OCW1-OCW3 (Operation Command Words)
  - Non-specific EOI (0x20) и Specific EOI (0x60 + IRQ)
  - Fully Nested Mode с автоматическими приоритетами

**hardware/i8253.h** - Intel 8253 PIT (Programmable Interval Timer):
- `i8253_read()`: Чтение счетчиков каналов с поддержкой LOBYTE/HIBYTE/TOGGLE режимов
- `i8253_write()`: Запись reload values и control word
- **3 независимых канала**:
  - Channel 0: System timer (динамический timer_interval)
  - Channel 1-2: Доступны для других целей (память, speaker)
- **Режимы доступа**: LOBYTE, HIBYTE, TOGGLE, LATCHCOUNT
- **Динамическая частота**: Расчет frequency = 1193182 / reload_value
- **Speaker support**: флаг speakerenabled для будущей эмуляции

**i8086_bus.pio** - Highly optimized PIO state machine implementing i8086 bus protocol:
- Captures 20-bit address + control signals (25 GPIO)
- **32-bit protocol**: ARM→PIO format is [data:16][pindirs_mask:16] for ISA-compatibility
- **Optimized polling**: Reduced cycle count in RD/WR detection loop
- **Efficient timing**: Sideset delays combined with IRQ operations
- **ISA-ready**: pindirs mask allows external devices to drive bus (when mask=0x0000)
- **Instruction count**: 29 out of 32 available (3 instructions reserve for future)
- **Smart data handling**: Direct pindirs/pins output without conditional jumps
- **Consistent naming**: Standardized WR_cycle/RD_cycle/INTA_cycle labels
- C initialization function at bottom (`i8086_bus_program_init`)

**config.h** - Hardware configuration:
- GPIO pin assignments (INTR_PIN=26, RESET_PIN=28, CLOCK_PIN=29)
- System clock: 400 MHz (PICO_CLOCK_SPEED)
- i8086 clock: 4 MHz (I8086_CLOCK_SPEED, configurable)
- PIO and IRQ settings (BUS_CTRL_PIO, BUS_CTRL_SM, WRITE_IRQ, READ_IRQ)
- Compiler hints: `likely()`, `unlikely()` macros for branch prediction optimization

**IRQ System** (интегрировано в main.c/cpu_bus.c):
- `pic_init()`: Инициализирует INTR пин как output, устанавливает LOW
- `bus_handler_core()`: Работает на Core1, выполняет:
  - Инициализацию железа (clock → pic → bus → reset)
  - Генерацию IRQ0 каждые ~54.925ms (18.2 Hz, IBM PC 8253/8254 совместимость)
  - Управление сигналом INTR на основе `i8259_get_pending_irqs()`
- **Система управления прерываниями (Intel 8259A)**:
  - `i8259`: глобальная структура с регистрами IMR, IRR, ISR
  - `i8259_interrupt(irq)`: Устанавливает бит в IRR (Interrupt Request Register)
  - `i8259_nextirq()`: Находит IRQ с наивысшим приоритетом, перемещает из IRR в ISR
  - `i8259_get_pending_irqs()`: Возвращает IRR & ~IMR (немаскированные прерывания)
  - Приоритет: IRQ0 > IRQ1 > ... > IRQ7 (Fully Nested Mode)
  - INTA обработка через PIO IRQ 3 → `i8259_nextirq()` → возврат вектора
- **Keyboard IRQ**:
  - `push_scancode()` вызывает `i8259_interrupt(1)` при наличии данных
  - Приоритет управляется автоматически через i8259 (проверка IMR)
  - Чтение порта 0x60 сбрасывает `current_scancode` автоматически

**bios.h** - Turbo XT BIOS v3.1 (10/28/2017) ROM image:
- 8KB array `BIOS[]` (renamed from GLABIOS_0_4_1_8T_ROM)
- 4-byte aligned for optimal access

**Keyboard System** (в main.c):
- `current_scancode`: Один байт для текущего скан-кода (0 = нет данных)
  - Simplified: нет буфера - один символ достаточно для человеческого ввода
  - Между нажатиями клавиш ~100ms, обработка IRQ ~100μs
- `ascii_to_scancode()`: Конвертер ASCII → IBM PC/XT Scancode Set 1
  - Поддержка a-z/A-Z (QWERTY раскладка)
  - Поддержка цифр 0-9
  - Специальные клавиши: Space, Enter, Backspace, Tab, Escape
- `push_scancode()`: Установка скан-кода + установка IRQ1 (если нет других IRQ)

## Current Implementation Status

**Implemented (Полная поддержка клавиатуры + оптимизированная архитектура):**
- ✅ Clock generation on GPIO29 (PWM, 4 MHz, 33% duty cycle)
- ✅ RESET sequence on GPIO28
- ✅ Highly optimized PIO bus controller for i8086 (16-bit data bus, 20-bit address)
- ✅ ROM emulation: 8KB Turbo XT BIOS v3.1 at 0xFE000-0xFFFFF (4-byte aligned)
- ✅ RAM emulation: 192KB at 0x00000-0x2FFFF (4-byte aligned)
- ✅ **Видеопамять**: 4KB at 0xB0000-0xB7FFF (MDA текстовый режим, 60 FPS рендеринг)
- ✅ Memory vs I/O address decoding (M/IO signal)
- ✅ **Ultra-fast 16-bit bus operations** via aligned memory access
- ✅ **BHE handling**: Полная поддержка 8/16 битных операций
- ✅ **INTA через PIO**: Обработка прерываний через state machine (IRQ 3)
- ✅ **INTR на GPIO26**: Генерация прерываний Core1
- ✅ **IRQ System**: IRQ0 (timer 18.2 Hz) + IRQ1 (keyboard) с приоритетами
- ✅ **Keyboard controller**: Порты 0x60/0x64, IBM PC/XT совместимость
- ✅ **ASCII → Scancode converter**: QWERTY layout, буквы, цифры, спецклавиши
- ✅ **Real-time video output**: 60 FPS вывод VIDEORAM через USB (Core0)
- ✅ **VGA port emulation**: Порт 0x3BA с vsync битами
- ✅ **Dual-core architecture**: Core0 (UI/keyboard), Core1 (bus/IRQ)
- ✅ **Compiler optimizations**: -Ofast, copy_to_ram, likely/unlikely hints
- 🚀 **Performance**: 4 MHz stable operation with keyboard & video

**Recently implemented:**
- ✅ **Intel 8259A PIC**: Полная эмуляция контроллера прерываний (ICW1-ICW4, OCW1-OCW3, IMR/IRR/ISR)
- ✅ **Intel 8253 PIT**: Программируемый таймер с 3 каналами (порты 0x40-0x43)
- ✅ **Прямая эмуляция портов**: Удален массив PORTS[], каждое устройство эмулируется напрямую

**Not yet implemented (Фаза 2):**
- ⚠️ Additional I/O devices (UART, speaker hardware emulation)
- ⚠️ External interrupts (IRQ2-IRQ7 via 8259A)
- ⚠️ DMA controller (Intel 8237A)
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
- Порты 0x20-0x21: Intel 8259A PIC (Programmable Interrupt Controller)
  - Полная эмуляция ICW1-ICW4 (Initialization Command Words)
  - Полная эмуляция OCW1-OCW3 (Operation Command Words)
  - Регистры: IMR, IRR, ISR, interrupt_vector_offset
- Порты 0x40-0x43: Intel 8253 PIT (Programmable Interval Timer)
  - 3 независимых канала с reload values
  - Режимы доступа: LOBYTE, HIBYTE, TOGGLE
  - Динамическая настройка timer_interval
- Порт 0x60: Keyboard Data Port (читает `current_scancode`, сбрасывает в 0)
- Порт 0x64: Keyboard Status Port (bit 0 = `current_scancode != 0`)
- Порт 0x3BA: VGA status register (эмуляция vsync битов, приоритет проверки)

**Архитектура портов (НОВОЕ):**
- **Прямая эмуляция**: Массив `PORTS[]` удален, каждое устройство эмулируется через отдельные функции
- **Разделение функций**: `port_read8()`/`port_write8()` для 8-битных операций
- **16-битная обработка**: `port_read()`/`port_write()` корректно обрабатывают BHE и A0
- **Оптимизация**: Приоритетная проверка часто используемых портов (VGA → 8259A → 8253 → Keyboard)
- **Экономия памяти**: Удален массив 4KB, используются только структуры устройств

### USB Commands (main.c)

**Специальные команды (uppercase, не отправляются в i8086):**
- **'M'**: Дамп памяти (первые 1024 байт RAM)
- **'V'**: Дамп видеопамяти (первые 5 строк × 80 символов)
- **'R'**: Сброс CPU (сбрасывает INTR и выполняет reset_cpu())
- **'B'**: Перезагрузка RP2040 в bootloader mode (для прошивки новой версии)

**Примечание**: Команда 'P' (дамп портов) удалена - массив PORTS[] больше не существует

**Обычный ввод (любые другие символы):**
- Все символы, кроме uppercase команд, автоматически конвертируются в скан-коды
- Отправляются в i8086 через keyboard buffer (порт 0x60)
- Генерируется IRQ1 при добавлении скан-кода
- Поддерживаемые символы: a-z, A-Z, 0-9, Space, Enter, Backspace, Tab, Escape

**Автоматический вывод:**
- Видеопамять (MDA 25×80) рендерится в терминал с частотой 60 FPS (Core0)
- ANSI escape codes для позиционирования курсора

### Modifying PIO Bus Logic

Edit `i8086_bus.pio` carefully - PIO assembly is timing-sensitive:
- All comments in Russian
- Test thoroughly - incorrect timing can hang i8086
- PIO program is recompiled automatically via `pico_generate_pio_header()`
- RD is checked before WR in polling loop (80% of operations are reads)

### Performance Considerations

**Current performance @ 400 MHz RP2040 / 4 MHz i8086:**
- Available: 100 RP2040 ticks per i8086 bus cycle (400 MHz / 4 MHz)
- **Required with optimizations:** ~75 RP2040 ticks (measured)
- Reserve: 1.33x (sufficient for stable operation)
- CPU load: ~75% worst case (bus handlers + IRQ processing)

**Performance achievements:**
- Successfully running at 4 MHz (800x faster than initial 5 KHz)
- Stable BIOS execution with full keyboard support
- 60 FPS video rendering (Core0) + 18.2 Hz timer (Core1)
- Zero blocking in IRQ handlers (optimized with `__force_inline` and `likely()`/`unlikely()`)
- Keyboard input latency: <1ms (IRQ1 generation on scancode push)

**Recent performance optimizations:**
- ✅ **Address alignment optimization**: Single 16-bit memory access via direct pointer cast
- ✅ **Memory alignment**: 4-byte aligned arrays for RAM/ROM/VIDEORAM
- ✅ **Eliminated conditional branches**: Direct memory access without boundary checks
- ✅ **Optimized PIO timing**: Reduced instruction count and sideset delays
- ✅ **32-bit protocol optimization**: Removed conditional jumps in PIO RD_cycle (13→8 instructions)
- ✅ **ISA-ready architecture**: pindirs mask allows future external device support
- ✅ **PIO instruction count**: 29/32 used (reserve 3 for future features)
- ✅ **Compiler optimizations**: -Ofast, -ffunction-sections, -fdata-sections, likely/unlikely
- ✅ **RAM execution**: copy_to_ram binary mode for maximum speed
- ✅ **Increased RAM**: 128KB → 192KB (removed logging system overhead)
- ✅ **Port range optimization**: Keyboard ports checked via `(port & 0xFF0) == 0x60`
- ✅ **Volatile minimization**: Local copies of volatile variables in hot paths
- ✅ **Inline functions**: `__force_inline` for i8086_read/write (eliminates call overhead)

**General optimization tips:**
- IRQ handlers run at highest priority (`PICO_HIGHEST_IRQ_PRIORITY`)
- Use `__time_critical_func` for all bus handlers (placed in RAM)
- PIO FIFO depth is 4 entries - handlers process extremely fast now
- Memory arrays are 4-byte aligned in SRAM (maximum access speed)

### Initialization Order (Critical!)

**Core0 (main):**
```c
// 1. System setup
hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
set_sys_clock_hz(PICO_CLOCK_SPEED, true);   // Overclock to 400 MHz

// 2. USB initialization
stdio_usb_init();
while (!stdio_usb_connected()) { ... }       // Wait for USB connection

// 3. Launch Core1 (handles all hardware initialization)
multicore_launch_core1(bus_handler_core);

// 4. Main loop: video rendering + keyboard input
while (true) { ... }
```

**Core1 (bus_handler_core) - критическая последовательность:**
```c
start_cpu_clock();   // 1. Start clock first (PWM on GPIO29, 4 MHz)
pic_init();          // 2. Initialize INTR pin as output, set LOW
cpu_bus_init();      // 3. Load PIO program, setup IRQ handlers
reset_cpu();         // 4. Release i8086 from reset (NOW safe to start)
// 5. Enter infinite loop: IRQ0 generation + INTR management
```

Wrong order = i8086 starts before PIO is ready = bus conflicts!

**Important notes:**
- Core1 handles ALL hardware initialization (clock, PIO, reset)
- Core0 only launches Core1 and handles UI
- `cpu_bus_init()` must be called BEFORE `reset_cpu()`
- Reset is the LAST step (releases i8086 to start executing)
- No multicore FIFO synchronization needed (simplified architecture)

### Memory Map

```
0x00000 - 0x2FFFF : RAM (192KB) - основная память, 4-byte aligned
0x30000 - 0xAFFFF : Unmapped (returns 0xFFFF)
0xB0000 - 0xB0FFF : Video RAM MDA (4KB) - текстовый режим 25×80
0xB1000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned
```

**Особенности:**
- Reset vector at 0xFFFF0 → points into ROM
- RAM увеличена с 128KB до 192KB (удалена система логирования)
- Video RAM адресуется через отдельный массив `VIDEORAM[4096]`
- I/O порты эмулируются напрямую через структуры устройств (i8259_s, i8253_s) - массив PORTS удален

### IRQ Priority System (Intel 8259A Compatible)

**Intel 8259A PIC Architecture:**
- **Регистры**: IMR (Interrupt Mask Register), IRR (Interrupt Request Register), ISR (In-Service Register)
- **Автоматические приоритеты**: IRQ0 > IRQ1 > IRQ2 > ... > IRQ7 (Fully Nested Mode)
- **Программируемый вектор**: `interrupt_vector_offset` (по умолчанию 0x08 для IBM PC)

**IRQ0 (Timer) - Core1:**
```c
if (absolute_time_diff_us(next_irq0, get_absolute_time()) >= 0) {
    i8259_interrupt(0);  // Устанавливает бит IRR[0]
    next_irq0 = delayed_by_us(next_irq0, timer_interval);  // Динамический интервал
}
```

**IRQ1 (Keyboard) - Core0:**
```c
void push_scancode(uint8_t scancode) {
    current_scancode = scancode;
    i8259_interrupt(1);  // Устанавливает бит IRR[1]
}
```

**INTR Management - Core1:**
```c
gpio_put(INTR_PIN, i8259_get_pending_irqs());  // Проверяет IRR & ~IMR
```

**INTA Protocol - cpu_bus.c:**
```c
// В bus_read_handler() при получении IRQ 3 (INTA cycle):
const uint8_t vector = i8259_nextirq();  // Получить вектор, переместить IRR → ISR
if (vector) {
    irq_pending_vector = 0xFF00 | vector;  // Формат: 0xFF00 | вектор
}
```

**EOI (End of Interrupt):**
- BIOS отправляет команду 0x20 на порт 0x20 для завершения обработки IRQ
- i8259 очищает соответствующий бит в ISR, разрешая новые прерывания

## Hardware Reference Documentation

### Intel Component Specifications (JSON)

**Машиночитаемые технические характеристики основных компонентов:**
- **Intel 8086**: [`docs/intel_8086_specs.json`](docs/intel_8086_specs.json) - Полная спецификация процессора, пины, регистры, временные диаграммы
- **Intel 8259A**: [`docs/intel_8259a_specs.json`](docs/intel_8259a_specs.json) - Контроллер прерываний, регистры ICW/OCW, каскадное подключение
- **Intel 8253**: [`docs/intel_8253_specs.json`](docs/intel_8253_specs.json) - Программируемый таймер, 6 режимов работы, порты I/O
- **Intel 8237A**: [`docs/intel_8237a_specs.json`](docs/intel_8237a_specs.json) - DMA контроллер, 4 канала, приоритеты, режимы передачи
- **Intel 8272A**: [`docs/intel_8272a_specs.json`](docs/intel_8272a_specs.json) - Контроллер дискет, команды чтения/записи, форматы дисков

**Использование:**
```bash
# Для быстрого доступа к спецификациям:
cat docs/intel_8086_specs.json | jq '.intel_8086_specifications.pin_configuration'
cat docs/intel_8259a_specs.json | jq '.intel_8259a_specifications.register_map'
cat docs/intel_8253_specs.json | jq '.intel_8253_specifications.operating_modes'
cat docs/intel_8237a_specs.json | jq '.intel_8237a_specifications.transfer_modes'
cat docs/intel_8272a_specs.json | jq '.intel_8272a_specifications.commands'
```

**Architecture Reference:**
- **System Architecture**: [`docs/i8086_architecture_reference.json`](docs/i8086_architecture_reference.json) - Полное руководство по архитектуре взаимодействия компонентов

**Key References:**
- IBM PC compatible interrupt vectors (IRQ0 = 0x08, IRQ1 = 0x09)
- 8253 system timer configuration (18.2 Hz, port 0x40-0x43)
- 8259A initialization sequence (ICW1 → ICW2 → ICW3 → ICW4)
- 8086 bus timing requirements (ALE, RD/WR, READY signals)
