# RP8086 - Чипсет i8086 на базе RP2040

## Описание проекта

RP8086 - это программно-аппаратный комплекс, использующий китайскую фиолетовую плату RP2040 (Raspberry Pi Pico) в качестве чипсета для процессора Intel 8086. RP2040 эмулирует:

- ✅ Контроллер системной шины (PIO hardware)
- ✅ Эмулятор ROM (8KB Turbo XT BIOS v3.1)
- ✅ Эмулятор RAM (192KB)
- ✅ Видеопамять MDA (4KB с real-time выводом 60 FPS)
- ✅ I/O контроллер с эмуляцией VGA и клавиатурных портов
- ✅ Генератор тактового сигнала (PWM, 4 MHz, 33% duty cycle)
- ✅ **Intel 8259A PIC** - Контроллер прерываний с полной поддержкой ICW/OCW (порты 0x20-0x21)
- ✅ **Intel 8253 PIT** - Программируемый таймер с 3 каналами (порты 0x40-0x43)
- ✅ Клавиатура IBM PC/XT (ASCII → Scancode конвертер, порты 0x60/0x64)

**Преимущество**: RP2040 толерантна к 5В на входе, поэтому дополнительные level shifters не требуются для прямого подключения к i8086.

**Статус**: Работает стабильно на 4 МГц с полной поддержкой клавиатуры и видео.

## Архитектура

### Двухуровневая архитектура

Проект использует уникальную двухуровневую архитектуру, которая разделяет обработку шины между аппаратным и программным обеспечением:

**Уровень 1: PIO (Аппаратный уровень - i8086_bus.pio)**
- Работает независимо на state machine PIO RP2040
- Обрабатывает все критичные по времени операции шины в аппаратном обеспечении
- Управляет сигналами шины i8086: ALE, RD, WR, M/IO, BHE, READY
- Контролирует двунаправленную шину данных (GPIO 0-15)
- Генерирует прерывания (IRQ0 для записи, IRQ1 для чтения, IRQ3 для INTA)
- Оптимизирован по времени с sideset задержками и сокращенным количеством инструкций
- Может работать на частоте до 133 МГц для детерминированных таймингов

**Уровень 2: ARM Cortex-M0+ (Программный уровень - cpu_bus.c)**
- Обслуживает прерывания от PIO через FIFO
- Реализует фактическую логику эмуляции памяти/I/O
- Все обработчики используют макрос `__time_critical_func` для выполнения из RAM
- Прерывания работают с приоритетом `PICO_HIGHEST_IRQ_PRIORITY`

### Многоядерная архитектура

Проект использует двухъядерную архитектуру RP2040 с четким разделением задач:

**Core0 (User Interface & I/O):**
- Инициализация системы (overclock 400 MHz, USB setup)
- Запуск Core1 через `multicore_launch_core1(bus_handler_core)`
- Основной цикл:
  - **Рендеринг видео**: 60 FPS вывод VIDEORAM (25×80 MDA) в терминал
  - **Обработка клавиатуры**: USB serial → ASCII → IBM PC/XT Scancode
  - **Debug команды**: R (reset), B (bootloader), M/V (dumps)
  - **Инжект скан-кодов**: `push_scancode()` → keyboard buffer → IRQ1
- Работает непрерывно с `tight_loop_contents()`, без WFI

**Core1 (Bus & IRQ Management):**
- Инициализация железа: `start_cpu_clock()` → `pic_init()` → `cpu_bus_init()` → `reset_cpu()`
- Основной цикл:
  - **Генерация таймера**: IRQ0 каждые ~54.925ms (динамический `timer_interval` через Intel 8253 PIT)
  - **Управление INTR**: Устанавливает INTR=HIGH при наличии немаскированных прерываний через `i8259_get_pending_irqs()`
  - **Приоритеты**: Автоматическое управление через Intel 8259A (IRQ0 > IRQ1 > ... > IRQ7)
- IRQ handlers (`bus_read_handler`, `bus_write_handler`) работают на обоих ядрах с `PICO_HIGHEST_IRQ_PRIORITY`

**Протокол INTA (State Machine PIO + Intel 8259A):**
- **INTA обрабатывается в PIO**: В `i8086_bus.pio` реализован `INTA_cycle`
- **Процесс INTA**:
  1. PIO обнаруживает INTA=LOW после ALE
  2. Генерирует IRQ 3 и устанавливает READY=1 (wait state)
  3. Ждет завершения первого цикла INTA (INTA=HIGH → INTA=LOW)
  4. Передает управление в обычный цикл чтения для второго INTA
- **Обработка в ARM (Intel 8259A Compatible)**: `bus_read_handler()` получает IRQ 3
  - Вызывает `i8259_nextirq()` для получения вектора прерывания с наивысшим приоритетом
  - i8259 автоматически перемещает прерывание из IRR в ISR (Interrupt Request → In-Service)
  - Сохраняет вектор: `irq_pending_vector = 0xFF00 | vector` (например, 0xFF08 для IRQ0)
  - При следующем чтении возвращает полный вектор прерывания
  - BIOS отправит команду EOI (End of Interrupt, порт 0x20, значение 0x20) для очистки ISR

### Назначение GPIO

| GPIO | Сигнал | Направление | Описание |
|------|--------|-------------|----------|
| 0-15 | AD0-AD15 | Двунаправленная | Мультиплексированная шина адреса/данных (16 бит) |
| 16-19 | A16-A19 | Вход | Старшие биты адреса (4 бита) |
| 20 | ALE | Вход | Address Latch Enable - защелка адреса |
| 21 | RD | Вход | Read strobe - сигнал чтения (активный LOW) |
| 22 | WR | Вход | Write strobe - сигнал записи (активный LOW) |
| 23 | INTA | Вход | Interrupt acknowledge от i8086 (активный LOW) |
| 24 | M/IO | Вход | Memory/IO select (1=память, 0=I/O) |
| 25 | BHE | Вход | Bus High Enable (полная поддержка 8/16 бит) |
| 26 | INTR | Выход | Interrupt request для i8086 (активный HIGH) |
| 27 | READY | Выход | Wait state control для CPU (0=wait, 1=ready) |
| 28 | RESET | Выход | Reset для i8086 (активный LOW) |
| 29 | CLK | Выход | Тактовый сигнал для i8086 (PWM, 4 МГц) |


## Протокол работы шины

### Цикл чтения (RD)

1. **T1 фаза (wait_ALE)**: CPU выставляет адрес на AD0-AD15 + A16-A19, активирует ALE=HIGH
   - PIO находится в состоянии `wait_ALE` и ждет `wait 1 gpio ALE_PIN`

2. **T2 фаза (capture_address)**:
   - PIO выполняет `wait 0 gpio ALE_PIN side 1` - захватывает адрес, устанавливает READY=1 (wait state)
   - `in pins, 26` захватывает 26 GPIO пинов (адрес + управляющие сигналы)
   - `push` отправляет 26 бит в FIFO для обработки ARM
   - **ISR структура**: `[AD0-AD15][A16-A19][ALE][RD][WR][INTA][MIO][BHE]`

3. **T2-T3 фаза (wait_strobe)**:
   - PIO опрашивает управляющие сигналы через `mov osr, pins`
   - `out null, 21` выбрасывает GPIO 0-20, оставляя RD/WR/INTA
   - Последовательно проверяет RD, WR, INTA через `out x, 1` и `jmp !x`

4. **T3 фаза (RD_cycle)** - с ISA-совместимым протоколом:
   - `irq 1` генерирует прерывание чтения для ARM
   - `pull block` блокирующе ждет **32-битный** ответ от ARM: `[data:16][pindirs_mask:16]`
   - **Формат ответа**:
     - `(data << 16) | 0xFFFF` - наш адрес, RP2040 отвечает (текущая реализация)
     - `0x00000000` - не наш адрес, внешнее ISA устройство ответит (будущее)
   - `out pindirs, 16` применяет маску направления пинов:
     - `0xFFFF` → AD0-AD15 переводятся в output mode
     - `0x0000` → AD0-AD15 остаются в high-Z (для ISA устройств)
   - `out pins, 16 side 1` выводит данные на шину, опускает READY=1 (готово)

5. **T4 фаза (cleanup)**:
   - `wait 1 gpio RD_PIN` ждет завершения цикла чтения (RD=HIGH)
   - `wait 1 gpio INTA_PIN` ждет завершения INTA (если был)
   - `mov osr, null` + `out pindirs, 16` возвращает AD0-AD15 в Z-состояние
   - `.wrap` автоматический возврат в начальное состояние `wait_ALE`

### Цикл записи (WR)

1. **T1-T2 фазы**: Аналогичны циклу чтения (захват адреса и управляющих сигналов)

2. **T3 фаза (WR_cycle)**:
   - `in pins, 16` захватывает 16 бит данных с AD0-AD15
   - `push side 1` отправляет данные в FIFO, опускает READY=0
   - `irq 0` генерирует прерывание записи для ARM с оптимизированной синхронизацией
   - `wait 1 gpio WR_PIN` ждет завершения цикла записи

3. **Завершение**:
   - PIO возвращается в состояние `wait_ALE` для следующего цикла

### Цикл INTA (Interrupt Acknowledge)

i8086 использует двухфазный INTA протокол для получения вектора прерывания:

1. **Обнаружение INTA**: PIO определяет INTA=LOW в цикле `wait_strobe`

2. **INTA_cycle (Первый INTA pulse)**:
   - `irq 3 side 1` генерирует IRQ 3 для ARM, устанавливает READY=1 (wait state)
   - `wait 1 gpio INTA_PIN` ждет завершения первого INTA pulse (переход в HIGH)
   - Синхронизация с ALE для определения начала второго INTA
   - `wait 0 gpio INTA_PIN` подтверждает нахождение во втором цикле INTA

3. **Второй INTA pulse (чтение вектора)**: После завершения первого INTA управление передается в обычный цикл чтения
   - ARM обработчик при получении IRQ 3 вызывает `i8259_nextirq()`:
     - Находит IRQ с наивысшим приоритетом в IRR & ~IMR (немаскированные прерывания)
     - Перемещает прерывание из IRR в ISR (устанавливает бит In-Service)
     - Возвращает полный вектор: `interrupt_vector_offset + irq_number` (например, 0x08 для IRQ0)
   - Вектор сохраняется в `irq_pending_vector = 0xFF00 | vector`
   - Следующий цикл чтения возвращает вектор i8086
   - BIOS обработает прерывание и отправит команду EOI (0x20 на порт 0x20) для очистки ISR

### Извлечение состояния шины (inline bit extraction)

В `cpu_bus.c` состояние шины извлекается напрямую из 32-битного слова PIO через битовые маски:

```c
// Структура 32-битного bus_state из PIO FIFO:
// Bits [31:26] - не используются (padding)
// Bits [25:0]  - данные шины (26 GPIO)

const uint32_t bus_state = BUS_CTRL_PIO->rxf[BUS_CTRL_SM];

// Прямое извлечение через побитовые операции:
#define MIO (1 << 24)  // Memory/IO signal
#define BHE (1 << 25)  // Bus High Enable

// Использование:
const bool is_memory = bus_state & MIO;  // true = память, false = I/O
const bool bhe = bus_state & BHE;        // Bus High Enable для 8/16-бит

// Адрес выравнивается при использовании:
const uint32_t address = bus_state & 0xFFFFE;  // Для памяти (выравнивание по 16-бит)
const uint32_t port = bus_state & 0xFFF;       // Для I/O портов (12-бит адрес)
```

**Битовая структура bus_state** (26 бит данных + 6 бит padding):
- `[19:0]` - AD0-AD15 (16 бит) + A16-A19 (4 бита) = 20-битный адрес
- `[20]` - ALE (Address Latch Enable)
- `[21]` - RD (Read strobe)
- `[22]` - WR (Write strobe)
- `[23]` - INTA (Interrupt Acknowledge)
- `[24]` - M/IO (Memory/IO select)
- `[25]` - BHE (Bus High Enable)

### Оптимизация FIFO и обработки прерываний

**IRQ0 (Write):**
- Обработчик `bus_write_handler()` читает 2 записи из FIFO:
  1. Адрес + управляющие сигналы
  2. 16-битные данные с шины AD0-AD15
- Вызывает `i8086_write(address, data, is_memory, bhe)`
- Использует выровненный доступ для максимальной производительности

**IRQ1 (Read)** - с ISA-совместимым протоколом:
- Обработчик `bus_read_handler()` читает адрес из FIFO
- Вызывает `i8086_read(address, is_memory)` или возвращает вектор прерывания
- **Отправляет 32-битный ответ** в PIO FIFO: `[data:16][pindirs_mask:16]`
  - Текущая реализация: `(data << 16) | 0xFFFF` (все адреса обрабатываются RP2040)
  - Будущее: `0x00000000` для адресов, обрабатываемых внешними ISA устройствами
- Преимущества:
  - PIO напрямую применяет маску без условных переходов (оптимизация)
  - Готовность к подключению ISA video карт, звуковых карт и других устройств
  - Сохранена обратная совместимость с текущим кодом

**IRQ3 (INTA) - Intel 8259A Interrupt Acknowledge:**
- Вызывает `i8259_nextirq()` для получения вектора с наивысшим приоритетом
- i8259 автоматически перемещает прерывание из IRR в ISR
- Сохраняет вектор: `irq_pending_vector = 0xFF00 | vector`
- Следующий цикл чтения возвращает полный вектор (например, 0xFF08 для IRQ0)
- **Примечание**: INTR остается активным до EOI (End of Interrupt) от BIOS

## Текущая реализация

### Эмуляция памяти

Память эмулируется через модуль `src/memory.h` с функциями `memory_read()` и `memory_write()`:

**RAM (192KB):**
- **Диапазон**: 0x00000 - 0x2FFFF
- **Массив**: `RAM[RAM_SIZE]` с 4-byte alignment (определен в `src/main.c`)
- **Поддержка 8/16 бит**: Полная обработка BHE и A0 через `write_to()` в `src/common.h`
- **Оптимизация**: Прямой 16-битный доступ через `*(uint16_t *)&RAM[address]`
- **Увеличен со 128KB**: Удалена система логирования, освобождена память

**ROM (8KB Turbo XT BIOS v3.1):**
- **Диапазон**: 0xFE000 - 0xFFFFF
- **Массив**: `BIOS[]` с 4-byte alignment (определен в `rom/bios.h`)
- **Только чтение**: 16-битный доступ, попытки записи игнорируются
- **Reset vector**: Адрес 0xFFFF0 указывает в ROM (точка входа после сброса)
- **Версия**: Turbo XT BIOS v3.1 (10/28/2017)

**Видеопамять MDA (4KB):**
- **Диапазон**: 0xB0000 - 0xB0FFF
- **Массив**: `VIDEORAM[4096]` для текстового режима 25×80 символов (определен в `src/main.c`)
- **Поддержка 8/16 бит**: Полная обработка записи через BHE и A0
- **Real-time вывод**: Core0 рендерит в терминал с частотой 60 FPS через ANSI escape codes
- **Формат**: Стандартный MDA text mode (символ + атрибут на каждую позицию)
- **Debug**: Ручной дамп через USB команду 'V' (первые 5 строк)

**Карта памяти (memory map):**
```
0x00000 - 0x2FFFF : RAM (192KB) - основная память, 4-byte aligned
0x30000 - 0xAFFFF : Unmapped (returns 0xFFFF при чтении)
0xB0000 - 0xB0FFF : Video RAM MDA (4KB) - текстовый режим 25×80
0xB1000 - 0xFDFFF : Unmapped (returns 0xFFFF при чтении)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned, read-only
```

### Эмулируемые I/O порты

**Аппаратные порты (эмуляция):**
| Порт | Описание | Реализация |
|------|----------|------------|
| 0x20-0x21 | Intel 8259A PIC (Programmable Interrupt Controller) | Полная эмуляция ICW1-ICW4, OCW1-OCW3, регистры IMR/IRR/ISR |
| 0x40-0x43 | Intel 8253 PIT (Programmable Interval Timer) | 3 канала, режимы LOBYTE/HIBYTE/TOGGLE, динамическая частота |
| 0x60 | Keyboard Data Port | Скан-коды IBM PC/XT Set 1 |
| 0x64 | Keyboard Status Port | Bit 0 = данные доступны (1 = есть, 0 = нет) |
| 0x3BA | VGA Status Register | Переключение vsync бита (эмуляция) |

**Архитектура портов:**
- **Прямая эмуляция устройств**: Массив `PORTS[]` удален, каждое устройство эмулируется напрямую
- **8/16-битная поддержка**: Все порты корректно обрабатывают BHE и A0
- **Разделение функций**: `port_read8()`/`port_write8()` для 8-битных операций
- **Оптимизация**: Приоритетная проверка часто используемых портов (VGA → 8259A → 8253 → Keyboard)
- Неопределенные порты возвращают 0xFF (8-bit) или 0xFFFF (16-bit)

### Обработка BHE (Bus High Enable)

Функции `i8086_read()` и `i8086_write()` поддерживают полную 8/16 битную совместимость:

```c
// Определение типа операции по BHE и A0
if (!bhe && !a0) {
    // 16-bit transfer: BHE=0, A0=0
    *(uint16_t *) &ram[address] = data;
} else if (bhe && !a0) {
    // Byte transfer на четном адресе: BHE=1, A0=0 (младший байт)
    ram[address] = data & 0xFF;
} else if (!bhe && a0) {
    // Byte transfer на нечетном адресе: BHE=0, A0=1 (старший байт)
    ram[address] = (data >> 8) & 0xFF;
}
// BHE=1, A0=1 - недопустимая комбинация, игнорируется
```

### Система прерываний (Intel 8259A Compatible)

**Intel 8259A PIC (Programmable Interrupt Controller):**
- **Полная эмуляция регистров**: IMR (Interrupt Mask Register), IRR (Interrupt Request Register), ISR (In-Service Register)
- **ICW поддержка**: ICW1-ICW4 (Initialization Command Words) для полной IBM PC совместимости
- **OCW поддержка**: OCW1-OCW3 (Operation Command Words) для управления маскированием и режимами
- **Fully Nested Mode**: Автоматическое управление приоритетами IRQ (IRQ0 > IRQ1 > ... > IRQ7)
- **EOI команды**: Non-specific EOI (0x20) и Specific EOI (0x60 + IRQ number)
- **Вектор прерываний**: Программируемый offset (по умолчанию 0x08 для IBM PC)

**IRQ0 (Timer) - Core1:**
- Генерируется через `i8259_interrupt(0)` каждые ~54.925ms (18.2 Hz, IBM PC стандарт)
- Динамический интервал настраивается через Intel 8253 PIT (порты 0x40-0x43)
- Использует `absolute_time_t` для точного тайминга
- Вектор прерывания: 0x08 (IRQ0 base + 0)

**IRQ1 (Keyboard) - Core0:**
- Генерируется через `i8259_interrupt(1)` при добавлении скан-кода
- Приоритет управляется автоматически через i8259 (проверка маски IMR)
- Один байт `current_scancode` (упрощенная архитектура для человеческого ввода)
- Вектор прерывания: 0x09 (IRQ0 base + 1)

**INTA Protocol (через i8259):**
- PIO обрабатывает INTA=LOW через `INTA_cycle` (IRQ 3)
- ARM вызывает `i8259_nextirq()` для получения вектора прерывания
- i8259 автоматически перемещает прерывание из IRR в ISR
- Возвращает полный вектор: `i8259.interrupt_vector_offset + irq_number`
- EOI команда (порт 0x20, значение 0x20) очищает ISR и готовит следующее прерывание

### USB команды (main.c)

**Специальные команды (uppercase, НЕ отправляются в i8086):**

| Команда | Описание | Функциональность |
|---------|----------|-----------------|
| **'M'** | Дамп памяти | Выводит первые 1024 байт RAM |
| **'V'** | Дамп видеопамяти | Выводит первые 5 строк × 80 символов из VIDEORAM |
| **'R'** | Сброс CPU | Сбрасывает INTR, выполняет последовательность reset_cpu() |
| **'B'** | Bootloader | Перезагружает RP2040 в bootloader mode (для прошивки) |

**Обычный ввод (любые другие символы):**
- Все символы, кроме uppercase команд, конвертируются в IBM PC/XT скан-коды
- Отправляются в i8086 через keyboard buffer (порт 0x60)
- Автоматическая генерация IRQ1 при добавлении скан-кода
- Поддержка: a-z, A-Z, 0-9, Space, Enter, Backspace, Tab, Escape

**Автоматический вывод:**
- Видеопамять (MDA 25×80) рендерится в терминал с частотой 60 FPS (Core0)

### Карта памяти

```
0x00000 - 0x2FFFF : RAM (192KB) - основная память, 4-byte aligned
0x30000 - 0xAFFFF : Unmapped (returns 0xFFFF)
0xB0000 - 0xB0FFF : Video RAM MDA (4KB) - текстовый режим 25×80
0xB1000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned
```

**Особенности:**
- Reset vector at 0xFFFF0 → points into ROM
- RAM увеличена с 128KB до 192KB
- Video RAM в отдельном массиве `VIDEORAM[4096]`
- I/O порты эмулируются напрямую (Intel 8259A, 8253, Keyboard, VGA) - массив PORTS удален

## Структура проекта

```
RP8086/
├── src/
│   ├── main.c                # Core0: UI, video rendering, keyboard input
│   ├── cpu.c/h               # Управление i8086 (clock, reset)
│   ├── cpu_bus.c/h           # Core1: bus handlers, memory emulation
│   ├── common.h              # Общие определения, структуры i8259_s и i8253_s
│   ├── memory.h              # Memory emulation (RAM, ROM, VIDEORAM)
│   ├── ports.h               # I/O ports routing и эмуляция
│   ├── hardware/
│   │   ├── i8259.h          # Intel 8259A PIC - контроллер прерываний
│   │   └── i8253.h          # Intel 8253 PIT - программируемый таймер
│   └── i8086_bus.pio         # PIO программа контроллера шины с INTA
├── rom/
│   └── bios.h                # Turbo XT BIOS v3.1 ROM образ (8KB)
├── config.h                  # Конфигурация GPIO, системы, макросы
├── CMakeLists.txt            # Конфигурация сборки с оптимизациями
├── README.md                 # Документация проекта
└── CLAUDE.md                 # Документация для Claude Code
```

## Ключевые функции

### cpu.c
- `start_cpu_clock()` - Генерация PWM clock для i8086 (33% duty, 4 МГц)
- `reset_cpu()` - RESET sequence (10 clocks LOW, 5 stabilization)

### cpu_bus.c
- `cpu_bus_init()` - Инициализация PIO и IRQ обработчиков с высоким приоритетом
- `bus_read_handler()` - Обработчик чтения (IRQ1) + INTA cycles (IRQ3)
- `bus_write_handler()` - Обработчик записи (IRQ0)
- `i8086_read()` - `__force_inline` чтение памяти/I/O с оптимизацией портов
- `i8086_write()` - `__force_inline` запись памяти/I/O с BHE поддержкой

### main.c
- `bus_handler_core()` - Работает на Core1:
  - Инициализация железа (clock → pic → bus → reset)
  - Генерация IRQ0 каждые 54.925ms (18.2 Hz)
  - Управление сигналом INTR
- `ascii_to_scancode()` - Конвертер ASCII → IBM PC/XT Scancode Set 1
- `push_scancode()` - Добавление скан-кода в buffer + генерация IRQ1
- `pic_init()` - Инициализация INTR пина как output

### Детальная реализация ключевых компонентов

**cpu_bus.c - Контроллер шины и память:**
- `i8086_read()`: Оптимизированное чтение с маршрутизацией memory vs I/O
  - Передает BHE для корректной обработки 8/16-битных операций
  - Вызывает `memory_read()` или `port_read()` в зависимости от M/IO
- `i8086_write()`: Полная поддержка 8/16 битных операций через BHE и A0
- `bus_read_handler()`: Обрабатывает INTA (IRQ3) + обычное чтение (IRQ1)
  - INTA path: вызывает `i8259_nextirq()`, возвращает вектор прерывания (IRR → ISR)
  - Обычное чтение: вызывает `i8086_read()` с оптимизацией `unlikely()`

**main.c - Keyboard & Video:**
- Упрощенная клавиатура: один байт `current_scancode` (достаточно для человеческого ввода)
- 60 FPS рендеринг видеопамяти через ANSI escape codes (Core0)
- Поддержка букв, цифр, специальных клавиш (Space, Enter, Backspace, Tab, Escape)
- QWERTY layout для ASCII → Scancode конвертации
- Генерация IRQ1 через `i8259_interrupt(1)` при вызове `push_scancode()`

**i8086_bus.pio - Высокооптимизированный state machine:**

**Структура программы:**
- `.side_set 1 opt` - один sideset бит для управления READY
- `.define` секция с жестко заданными GPIO pin assignments
- Основной цикл: `wait_ALE` → `capture_address` → `wait_strobe` → `operation_cycle`

**Ключевые оптимизации:**
- **Оптимизированный polling**: Сокращено количество инструкций в цикле опроса RD/WR/INTA
- **Эффективные тайминги**: Sideset delays совмещены с IRQ операциями (`irq 0 side 1`)
- **Быстрая обработка данных**: Единый 16-битный доступ через `in pins, 16` / `out pins, 16`
- **Детерминированная логика**: Последовательная проверка сигналов без условных переходов

**Состояния PIO:**
1. `wait_ALE`: Ожидание начала цикла шины (ALE=HIGH)
2. `capture_address`: Захват 26 пинов (адрес + управление) при ALE=LOW
3. `wait_strobe`: Опрос управляющих сигналов (RD/WR/INTA)
4. `RD_cycle`: Цикл чтения с выводом данных на шину
5. `WR_cycle`: Цикл записи с захватом данных с шины
6. `INTA_cycle`: Обработка прерывания через state machine
7. `cleanup`: Возврат шины данных в Z-состояние

**C инициализация PIO:**
```c
void i8086_bus_program_init(PIO pio, uint sm, uint offset)
{
    // Настройка shift регистров
    sm_config_set_in_shift(&cfg, false, false, 32);   // IN: сдвиг влево
    sm_config_set_out_shift(&cfg, true, false, 32);   // OUT: сдвиг вправо

    // Назначение пинов
    sm_config_set_in_pins(&cfg, i8086_bus_AD_BUS_PINS);      // IN base = GPIO 0
    sm_config_set_out_pins(&cfg, i8086_bus_AD_BUS_PINS, 16); // OUT base = GPIO 0
    sm_config_set_sideset_pins(&cfg, i8086_bus_READY_PIN);   // SIDESET = GPIO 27
}
```

**Прерывания и многоядерность (Intel 8259A Compatible):**
- `pic_init()`: Инициализирует INTR пин (GPIO26) как output, LOW по умолчанию
- `bus_handler_core()`: Работает на Core1:
  - Инициализация железа: clock → pic → bus → reset
  - Генерирует IRQ0 каждые ~54.925ms через `i8259_interrupt(0)` (динамический timer_interval)
  - Управляет INTR: устанавливает HIGH при наличии немаскированных IRQ через `i8259_get_pending_irqs()`
- **Intel 8259A PIC**: Полная эмуляция контроллера прерываний
  - Регистры: IMR (маски), IRR (запросы), ISR (обслуживание)
  - Автоматические приоритеты: IRQ0 > IRQ1 > ... > IRQ7 (Fully Nested Mode)
  - EOI команды для завершения обработки прерываний
- **INTA через PIO**: Полностью реализован в state machine (IRQ 3)
- **Keyboard IRQ**: Core0 вызывает `i8259_interrupt(1)` при `push_scancode()`
- **Упрощенная архитектура**: Нет multicore FIFO, прямое взаимодействие через i8259

## Особенности реализации

### Производительность и оптимизации

**Текущая производительность @ 400 MHz RP2040 / 4 MHz i8086:**
- **Доступно**: 100 тактов RP2040 на bus cycle (400 MHz / 4 MHz)
- **Требуется**: ~75 тактов RP2040 (измерено с оптимизациями)
- **Запас**: 1.33x (достаточно для стабильной работы)
- **CPU нагрузка**: ~75% worst case (bus handlers + IRQ processing)

**Достижения производительности:**
- ✅ **800x ускорение**: 5 КГц → 4 МГц (стабильная работа)
- ✅ **60 FPS видео**: Core0 рендерит VIDEORAM в терминал
- ✅ **18.2 Hz timer**: Core1 генерирует IRQ0 (IBM PC совместимость)
- ✅ **Keyboard latency**: <1ms (IRQ1 generation on scancode push)
- ✅ **Нулевые блокировки**: `__force_inline` + `likely()`/`unlikely()` hints

**Ключевые оптимизации:**
- ✅ **32-битный протокол PIO**: `[data:16][pindirs_mask:16]` для ISA-совместимости
- ✅ **Упрощенный RD_cycle**: 13 → 8 инструкций PIO (-5 инструкций)
- ✅ **PIO instruction count**: 29/32 используется (запас 3 для будущего)
- ✅ **Без условных переходов**: PIO напрямую применяет pindirs маску
- ✅ **Прямой доступ к памяти**: `*(uint16_t *)&RAM[address]` без boundary checks
- ✅ **4-byte alignment**: RAM, ROM, VIDEORAM для оптимального доступа
- ✅ **Range check портов**: `(port & 0xFF0) == 0x60` экономит 2-3 такта
- ✅ **Volatile minimization**: Локальные копии `kb_head`, `kb_tail`
- ✅ **Inline functions**: `__force_inline` для i8086_read/write
- ✅ **Compiler hints**: `likely()`/`unlikely()` для branch prediction
- ✅ **Увеличенная RAM**: 128KB → 192KB (удалена система логирования)
- ✅ **Компилятор**: -Ofast, copy_to_ram, -ffunction-sections

### PIO State Machine

- Работает независимо от ARM ядра
- Частота: до 133 МГц (зависит от разгона RP2040)
- Детерминированный timing для сигналов шины
- FIFO для обмена данными с ARM
- **29 инструкций из 32**: Эффективное использование памяти PIO (запас 3)
- **ISA-ready протокол**: 32-битный формат `[data:16][mask:16]`
- **Без условных переходов**: Прямая установка pindirs через маску
- **Оптимизированный polling**: Сокращено количество инструкций в RD/WR циклах
- **Эффективная обработка данных**: Единый 16-битный доступ с выравниванием

## Статус реализации

### ✅ ПОЛНАЯ IBM PC СОВМЕСТИМОСТЬ + INTEL CHIPSET ЭМУЛЯЦИЯ
- ✅ Полнофункциональный эмулятор Intel 8086 @ 4 МГц
- ✅ 192KB RAM + 8KB ROM (Turbo XT BIOS v3.1) + 4KB видеопамять MDA
- ✅ Высокопроизводительный контроллер шины на PIO
- ✅ PWM генератор тактов (4 МГц, 33% duty cycle)
- ✅ Полная поддержка 8/16 битных операций (BHE handling)
- ✅ **Intel 8259A PIC**: Полная эмуляция контроллера прерываний (ICW1-ICW4, OCW1-OCW3, IMR/IRR/ISR)
- ✅ **Intel 8253 PIT**: Программируемый таймер с 3 каналами, динамической частотой
- ✅ **INTA через PIO**: Обработка прерываний в state machine (IRQ 3)
- ✅ **Keyboard controller**: Порты 0x60/0x64, IBM PC/XT совместимость
- ✅ **ASCII → Scancode**: QWERTY layout, буквы, цифры, спецклавиши
- ✅ **Real-time video**: 60 FPS вывод VIDEORAM через USB (Core0)
- ✅ **VGA порт 0x3BA**: Эмуляция vsync битов
- ✅ **Dual-core architecture**: Core0 (UI/keyboard), Core1 (bus/IRQ)

### Последние достижения 🚀
- **Intel 8259A PIC**: Полная эмуляция контроллера прерываний с ICW/OCW регистрами
- **Intel 8253 PIT**: Программируемый таймер с 3 каналами и динамической частотой
- **Прямая эмуляция портов**: Удален массив PORTS[], каждое устройство эмулируется напрямую
- **8/16-битные порты**: Корректная обработка BHE и A0 во всех I/O операциях
- **ISA-ready протокол**: 32-битный формат PIO-ARM для будущей поддержки внешних устройств
- **Оптимизация PIO**: RD_cycle сокращен с 13 до 8 инструкций (-5 инструкций)
- **29/32 инструкции PIO**: Эффективное использование памяти с запасом для расширений
- **Без условных переходов**: Прямое применение pindirs маски в hot path
- **800x ускорение**: 5 КГц → 4 МГц (стабильная работа)
- **Keyboard support**: Полная эмуляция IBM PC/XT клавиатуры
- **60 FPS video**: Real-time рендеринг в терминал
- **Inline optimization**: `__force_inline` для критичных функций
- **RAM увеличена**: 128KB → 192KB
- **Branch prediction**: `likely()`/`unlikely()` hints

### TODO (Фаза 2)

- [x] Обработка прерываний INTR/INTA ✅ ЗАВЕРШЕНО
- [x] Поддержка BHE для 8/16 битных операций ✅ ЗАВЕРШЕНО
- [x] Видеопамять MDA ✅ ЗАВЕРШЕНО
- [x] Real-time видеовывод ✅ ЗАВЕРШЕНО
- [x] 800x ускорение (5 КГц → 4 МГц) ✅ ЗАВЕРШЕНО
- [x] Keyboard controller (IBM PC/XT) ✅ ЗАВЕРШЕНО
- [x] Dual-core architecture optimization ✅ ЗАВЕРШЕНО
- [x] ISA-ready протокол (32-бит ARM→PIO) ✅ ЗАВЕРШЕНО
- [x] **Intel 8259A PIC** (ICW1-ICW4, OCW1-OCW3, IMR/IRR/ISR) ✅ ЗАВЕРШЕНО
- [x] **Intel 8253 PIT** (3 канала, LOBYTE/HIBYTE/TOGGLE режимы) ✅ ЗАВЕРШЕНО
- [ ] ISA bus support: подключение внешних устройств (video cards, sound cards)
  - Требуется: pull-up резисторы на AD0-AD15, решение вопроса с READY signal
  - Протокол готов: маска 0x0000 для ISA устройств вместо 0xFFFF
- [ ] Увеличение частоты до 5+ МГц (запас 3 PIO инструкции для оптимизаций)
- [ ] Расширение I/O устройств (UART, speaker hardware emulation)
- [ ] Внешние прерывания (IRQ2-IRQ7 через 8259A)
- [ ] DMA контроллер (Intel 8237A)
- [ ] Дополнительные видеорежимы (CGA/EGA совместимость)

## Сборка проекта

```bash
mkdir cmake-build-release
cd cmake-build-release
cmake --build .
```

Результат: `cmake-build-release/bin/rp2040/Release/MultiIO.uf2` для загрузки на RP2040

## Требования

- Pico SDK
- CMake >= 3.13
- GCC ARM toolchain
- Китайская фиолетовая плата RP2040 (все 30 GPIO доступны)


## Лицензия

Не указана
