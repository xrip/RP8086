# RP8086 - Чипсет i8086 на базе RP2040

## Описание проекта

RP8086 - это программно-аппаратный комплекс, использующий китайскую фиолетовую плату RP2040 (Raspberry Pi Pico) в качестве чипсета для процессора Intel 8086. RP2040 эмулирует:

- ✅ Контроллер системной шины (PIO hardware)
- ✅ Эмулятор ROM (8KB Turbo XT BIOS v3.1)
- ✅ Эмулятор RAM (192KB)
- ✅ Видеопамять MDA (4KB с real-time выводом 60 FPS)
- ✅ I/O контроллер с эмуляцией VGA и клавиатурных портов
- ✅ Генератор тактового сигнала (PWM, 4 MHz, 33% duty cycle)
- ✅ Контроллер прерываний с приоритетами (IRQ0 timer + IRQ1 keyboard)
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
  - **Debug команды**: R (reset), B (bootloader), M/V/P (dumps)
  - **Инжект скан-кодов**: `push_scancode()` → keyboard buffer → IRQ1
- Работает непрерывно с `tight_loop_contents()`, без WFI

**Core1 (Bus & IRQ Management):**
- Инициализация железа: `start_cpu_clock()` → `pic_init()` → `cpu_bus_init()` → `reset_cpu()`
- Основной цикл:
  - **Генерация таймера**: IRQ0 каждые 54.925ms (18.2 Hz, IBM PC 8253/8254 стандарт)
  - **Управление INTR**: Устанавливает INTR=HIGH при `current_irq_vector != 0`
  - **Приоритеты**: IRQ0 (timer) > IRQ1 (keyboard)
- IRQ handlers (`bus_read_handler`, `bus_write_handler`) работают на обоих ядрах с `PICO_HIGHEST_IRQ_PRIORITY`

**Протокол INTA (State Machine PIO):**
- **INTA обрабатывается в PIO**: В `i8086_bus.pio` реализован `INTA_cycle`
- **Процесс INTA**:
  1. PIO обнаруживает INTA=LOW после ALE
  2. Генерирует IRQ 3 и устанавливает READY=1 (wait state)
  3. Ждет завершения цикла INTA (INTA=HIGH → INTA=LOW)
  4. Передает управление в обычный цикл чтения
- **Обработка в ARM**: `bus_read_handler()` получает IRQ 3
  - Устанавливает статический флаг `irq_pending = true`
  - Сбрасывает INTR=0 через `gpio_put(INTR_PIN, 0)`
  - При следующем чтении возвращает `current_irq_vector` (0xFF08 для IRQ0, 0xFF09 для IRQ1)
  - Очищает вектор: `irq_pending = current_irq_vector = 0`

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

4. **T3 фаза (RD_cycle)**:
   - `irq 1` генерирует прерывание чтения для ARM
   - `mov osr, !null` + `out pindirs, 16` переводит AD0-AD15 в режим выхода
   - `pull block` блокирующе ждет данные от ARM
   - `out pins, 16 side 1` выводит данные на шину, опускает READY=0

5. **T4 фаза (cleanup)**:
   - `wait 1 gpio RD_PIN` ждет завершения цикла чтения
   - `mov osr, null` + `out pindirs, 16` возвращает AD0-AD15 в Z-состояние
   - `jmp wait_ALE` возврат в начальное состояние

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

1. **Обнаружение INTA**: PIO определяет INTA=LOW в цикле `wait_strobe`

2. **INTA_cycle**:
   - `irq 3 side 1` генерирует прерывание для ARM, устанавливает READY=1 (wait state)
   - `wait 1 gpio INTA_PIN` ждет завершения INTA (переход в HIGH)
   - Последовательность ожидания синхронизации с ALE
   - `wait 0 gpio INTA_PIN` подтверждает нахождение во втором цикле INTA

3. **Переход в чтение**: После завершения INTA управление передается в обычный цикл чтения
   - ARM обработчик устанавливает `irq_pending1 = true`
   - Следующий цикл чтения возвращает вектор прерывания (0x08)

### Парсинг состояния шины (parse_bus_state)

Функция `parse_bus_state()` в cpu_bus.c извлекает информацию из 26-битного слова PIO:

```c
info.address = bus_data & 0xFFFFF;       // Bits [19:0] - AD0-AD15 + A16-A19
info.ale     = (bus_data >> 20) & 1;     // Bit [20] - ALE
info.rd      = (bus_data >> 21) & 1;     // Bit [21] - RD
info.wr      = (bus_data >> 22) & 1;     // Bit [22] - WR
info.inta    = (bus_data >> 23) & 1;     // Bit [23] - INTA
info.m_io    = (bus_data >> 24) & 1;     // Bit [24] - M/IO
info.bhe     = (bus_data >> 25) & 1;     // Bit [25] - BHE
```

### Оптимизация FIFO и обработки прерываний

**IRQ0 (Write):**
- Обработчик `bus_write_handler()` читает 2 записи из FIFO:
  1. Адрес + управляющие сигналы
  2. 16-битные данные с шины AD0-AD15
- Вызывает `i8086_write(address, data, is_memory, bhe)`
- Использует выровненный доступ для максимальной производительности

**IRQ1 (Read):**
- Обработчик `bus_read_handler()` читает адрес из FIFO
- Вызывает `i8086_read(address, is_memory)` или возвращает вектор прерывания
- Отправляет 16-битные данные обратно в PIO FIFO

**IRQ3 (INTA):**
- Устанавливает флаг `irq_pending1 = true`
- При следующем чтении возвращает вектор прерывания 0x08
- Автоматически сбрасывает INTR=0

## Текущая реализация

### Эмуляция памяти

**RAM (192KB):**
- Диапазон: 0x00000 - 0x2FFFF
- Массив `RAM[RAM_SIZE]` с 4-byte alignment
- Поддержка 8/16 битных операций через BHE и A0
- **Оптимизация**: Прямой 16-битный доступ через `*(uint16_t *)&RAM[address]`
- **Увеличен со 128KB**: Удалена система логирования, освобождена память

**ROM (8KB Turbo XT BIOS v3.1):**
- Диапазон: 0xFE000 - 0xFFFFF
- Массив `BIOS[]` с 4-byte alignment (переименован из GLABIOS_0_4_1_8T_ROM)
- Только чтение, 16-битный доступ
- Вектор сброса по адресу 0xFFFF0 указывает в ROM
- Дата релиза BIOS: 10/28/2017

**Видеопамять MDA (4KB):**
- Диапазон: 0xB0000 - 0xB0FFF
- Массив `VIDEORAM[4096]` для текстового режима (25×80 символов)
- Поддержка 8/16 битных операций записи
- **Real-time вывод**: Core0 рендерит в терминал с частотой 60 FPS через ANSI escape codes
- Ручной дамп через USB команду 'V' (первые 5 строк)

### Эмулируемые I/O порты

**Аппаратные порты (эмуляция):**
| Порт | Описание | Возвращаемое значение |
|------|----------|---------------------|
| 0x3BA | VGA Status Register | Переключение vsync бита (эмуляция) |
| 0x60 | Keyboard Data Port | Скан-коды из circular buffer (IBM PC/XT Set 1) |
| 0x64 | Keyboard Status Port | Bit 0 = данные доступны (1 = есть, 0 = нет) |

**Общие порты:**
- Порты 0x000-0xFFF доступны для эмуляции (массив `PORTS[]`)
- Поддержка 8/16 битных операций через BHE
- Чтение возвращает 0xFFFF для неопределенных портов

**Оптимизация проверки портов:**
- VGA порт (0x3BA) проверяется первым (самый частый доступ)
- Клавиатурные порты (0x60-0x6F) проверяются через диапазонную маску: `(port & 0xFF0) == 0x60`
- Экономия 2-3 такта RP2040 на каждой I/O операции

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

### Система прерываний

**IRQ Management:**
- Единая глобальная переменная: `current_irq_vector` (uint16_t)
- Значения: 0 (нет IRQ), 0xFF08 (IRQ0 timer), 0xFF09 (IRQ1 keyboard)
- Приоритет: IRQ0 > IRQ1 (через условную проверку в `push_scancode()`)

**IRQ0 (Timer) - Core1:**
- Генерируется каждые 54.925ms (18.2 Hz, IBM PC 8253/8254 совместимость)
- Использует `absolute_time_t` для точного тайминга
- Безусловно устанавливает `current_irq_vector = 0xFF08`

**IRQ1 (Keyboard) - Core0:**
- Генерируется при добавлении скан-кода в буфер
- Проверка приоритета: `if (!current_irq_vector)` перед установкой 0xFF09
- Circular buffer на 16 байт для скан-кодов

**INTA Protocol:**
- PIO обрабатывает INTA=LOW через `INTA_cycle` (IRQ 3)
- ARM устанавливает `irq_pending = true` при получении IRQ3
- Сбрасывает INTR=0 через `gpio_put(INTR_PIN, 0)`
- Следующий цикл чтения возвращает `current_irq_vector`
- Очищает: `irq_pending = current_irq_vector = 0`

### USB команды (main.c)

**Специальные команды (uppercase, НЕ отправляются в i8086):**

| Команда | Описание | Функциональность |
|---------|----------|-----------------|
| **'M'** | Дамп памяти | Выводит первые 1024 байт RAM |
| **'V'** | Дамп видеопамяти | Выводит первые 5 строк × 80 символов из VIDEORAM |
| **'P'** | Дамп портов | Выводит первые 1024 байт массива PORTS |
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
- I/O порты в массиве `PORTS[0xFFF]` (2-byte aligned)

## Структура проекта

```
RP8086/
├── main.c              # Core0: UI, video rendering, keyboard input
├── cpu.c/h             # Управление i8086 (clock, reset)
├── cpu_bus.c/h         # Core1: bus handlers, memory emulation, IRQ system
├── i8086_bus.pio       # PIO программа контроллера шины с INTA поддержкой
├── config.h            # Конфигурация GPIO, системы, likely/unlikely макросы
├── bios.h              # Turbo XT BIOS v3.1 ROM образ (8KB array BIOS[])
├── CMakeLists.txt      # Конфигурация сборки с оптимизациями
├── README.md           # Документация проекта
└── CLAUDE.md           # Документация для Claude Code
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
- `i8086_read()`: Оптимизированное чтение с приоритетом проверок:
  1. VGA port (0x3BA) - самый частый
  2. Keyboard ports (0x60-0x6F) - range check через `(port & 0xFF0) == 0x60`
  3. Локальные копии volatile переменных для минимизации доступа к памяти
- `i8086_write()`: Полная поддержка 8/16 битных операций через BHE и A0
- `bus_read_handler()`: Обрабатывает INTA (IRQ3) + обычное чтение (IRQ1)
  - INTA path: возвращает `current_irq_vector`, очищает флаги
  - Обычное чтение: вызывает `i8086_read()` с оптимизацией `unlikely()`

**main.c - Keyboard & Video:**
- Circular buffer на 16 байт для keyboard scancodes
- 60 FPS рендеринг видеопамяти через ANSI escape codes
- Поддержка букв, цифр, специальных клавиш (Space, Enter, Backspace, Tab, Escape)
- QWERTY layout для ASCII → Scancode конвертации

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

**Прерывания и многоядерность:**
- `pic_init()`: Инициализирует INTR пин (GPIO26) как output, LOW по умолчанию
- `bus_handler_core()`: Работает на Core1:
  - Инициализация железа: clock → pic → bus → reset
  - Генерирует IRQ0 каждые 54.925ms (18.2 Hz, IBM PC совместимость)
  - Управляет INTR: устанавливает HIGH при `current_irq_vector != 0`
- **INTA через PIO**: Полностью реализован в state machine (IRQ 3)
- **Keyboard IRQ**: Core0 устанавливает IRQ1 при `push_scancode()` с проверкой приоритета
- **Упрощенная архитектура**: Нет multicore FIFO, одна переменная `current_irq_vector`

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
- **Оптимизированный polling**: Сокращено количество инструкций
- **Эффективная обработка данных**: Единый 16-битный доступ

## Статус реализации

### ✅ ПОЛНАЯ ПОДДЕРЖКА КЛАВИАТУРЫ + ОПТИМИЗИРОВАННАЯ АРХИТЕКТУРА
- ✅ Полнофункциональный эмулятор Intel 8086 @ 4 МГц
- ✅ 192KB RAM + 8KB ROM (Turbo XT BIOS v3.1) + 4KB видеопамять MDA
- ✅ Высокопроизводительный контроллер шины на PIO
- ✅ PWM генератор тактов (4 МГц, 33% duty cycle)
- ✅ Полная поддержка 8/16 битных операций (BHE handling)
- ✅ **INTA через PIO**: Обработка прерываний в state machine (IRQ 3)
- ✅ **IRQ System**: IRQ0 (timer 18.2 Hz) + IRQ1 (keyboard) с приоритетами
- ✅ **Keyboard controller**: Порты 0x60/0x64, IBM PC/XT совместимость
- ✅ **ASCII → Scancode**: QWERTY layout, буквы, цифры, спецклавиши
- ✅ **Real-time video**: 60 FPS вывод VIDEORAM через USB (Core0)
- ✅ **VGA порт 0x3BA**: Эмуляция vsync битов
- ✅ **Dual-core architecture**: Core0 (UI/keyboard), Core1 (bus/IRQ)

### Последние достижения 🚀
- **800x ускорение**: 5 КГц → 4 МГц (стабильная работа)
- **Keyboard support**: Полная эмуляция IBM PC/XT клавиатуры
- **60 FPS video**: Real-time рендеринг в терминал
- **Упрощенная архитектура**: Удалена система логирования
- **Оптимизация портов**: Range checks для keyboard (экономия 2-3 тактов)
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
- [ ] Увеличение частоты до 5+ МГц (требует дополнительных оптимизаций)
- [ ] Расширение I/O устройств (UART, PIT hardware emulation)
- [ ] Полный интерфейс 8259A (ICW1-ICW4, OCW1-OCW3, маски IRQ)
- [ ] Внешние прерывания и DMA
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
