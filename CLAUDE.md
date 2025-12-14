# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RP8086 is a hardware-software complex that uses a Raspberry Pi Pico (RP2350B) as a complete chipset for the Intel 8086 processor. The RP2350B acts as:
- Bus controller (via PIO0 state machine at 500 MHz)
- ROM/RAM emulator (736KB RAM via external PSRAM + 16KB VRAM + 8KB BIOS)
- I/O controller (Intel 8259A, 8253, 8237, 8272A, XTIDE, 16550 UART, i8042 keyboard controller emulation)
- Clock generator (4.75-6 MHz PWM, 33% duty cycle, configurable)
- CGA/PCjr video adapter with hardware VGA output (GPIO 30-37 via PIO1 and R-2R DAC)
  - Text mode: 80×25 (16 colors)
  - CGA graphics: 320×200×4 colors
  - PCjr graphics: 160×200×16 and 320×200×16 colors (IBM PCjr/Tandy 1000 compatible)
- PC Speaker controller (GPIO 46 via PWM, emulates i8253 Channel 2)
- SD Card reader (SPI1 on GPIO 40-43 for loading disk images from FAT filesystem)
- USB Host controller (built-in RP2350B USB controller with TinyUSB stack)
  - USB HID keyboard: auto-conversion USB HID → XT Scancodes → i8042 controller
  - USB HID mouse: emulation of Microsoft Serial Mouse protocol via COM1 (ports 0x3F8-0x3FF)
  - USB hub support: up to 4 HID devices simultaneously
  - Plug & Play: automatic device detection on Type-C port connection

The system successfully boots DR-DOS 7 and other DOS operating systems from SD card floppy/HDD images and runs at stable **4.75-6 MHz**.

## Build Commands

```bash
# Build from cmake-build-rp2350b directory
cd cmake-build-rp2350b
cmake --build .

# Output files (in bin/ directory):
# - RP8086-<branch>-<build_number>.uf2 - Flashable to RP2350B via USB bootloader
# - RP8086-<branch>-<build_number>.elf - Debug symbols
# - RP8086-<branch>-<build_number>.hex - Intel HEX format
# - RP8086-<branch>-<build_number>.bin - Raw binary
```

**Build modes:**
- **DEBUG**: `cmake -DDEBUG=1 ..` - USB serial stdio, no USB HID, verbose logging
- **RELEASE** (default): USB HID enabled, optimized binary, no debug output

**Flash to device:**
```bash
# 1. Hold BOOTSEL button while connecting RP2350B to USB
# 2. Drag & drop .uf2 file to mounted drive (RPI-RP2)
# 3. Device automatically reboots and starts firmware
```

**Build Requirements:**
- Pico SDK (set `PICO_SDK_PATH` environment variable)
- CMake >= 3.13
- ARM GCC toolchain (arm-none-eabi-gcc)
- C23 standard support

**Hardware Requirements:**
- **WeAct Studio RP2350B Core Board** (WEACT_STUDIO_RP2350B_CORE)
  - RP2350B microcontroller (ARM Cortex-M33 dual-core @ 500 MHz max)
  - 16MB QSPI Flash (W25Q128)
  - 8MB QSPI PSRAM (APS6404L-3SQR)
  - USB Type-C connector (USB 1.1 Host/Device)
  - 3.3V LDO regulator (supports up to 500mA)
  - BOOTSEL button for firmware flashing
  - Onboard LED (GPIO 25 or board-specific)
- **Intel 8086 CPU** (or 8086 compatible: NEC V40)
- **External components**: VGA R-2R DAC, level shifters (optional), SD card module

**Build Configuration:**
- **Target Board**: `WEACT_STUDIO_RP2350B_CORE` (WeAct Studio RP2350B development board)
- **Platform**: `rp2350-arm-s` (ARM Secure mode, enables TrustZone features)
  - **Why ARM Secure?** Provides access to full 512KB RAM (vs 264KB in Non-Secure mode)
  - Enables unrestricted peripheral access (required for PIO, DMA, USB Host)
  - TrustZone features not actively used, but Secure mode is necessary for performance
- **Flash Size**: 16MB (PICO_FLASH_SIZE_BYTES = 16777216)
- **Optimization**: `-Ofast` with LTO enabled (`-flto -fwhole-program`)
- **Binary mode**: `copy_to_ram` (executes from RAM for maximum performance)
- **Linker script**: Custom `memmap.ld` for optimized memory layout
  - **Internal RAM**: 512KB at 0x20000000 (used for code execution via copy_to_ram)
  - **External PSRAM**: 8MB at 0x11000000 (736KB used for i8086 RAM emulation)
  - **SCRATCH_X/Y**: 4KB each at 0x20080000/0x20081000 (fast scratchpad memory)
- **Size optimizations**: `--wrap=atexit`, `--wrap=abort`, `--strip-all`, `--gc-sections`
- **Standards**: C23 and C++23
- **System clock**: 500 MHz (overclocked with voltage boost to 1.60V)
- **i8086 clock**: 4.75-6 MHz (configurable via `I8086_CLOCK_SPEED` in common.h)

**Conditional compilation:**
- **DEBUG mode**: USB Host disabled, USB serial stdio enabled for debugging
- **RELEASE mode**: USB Host enabled (TinyUSB + usbhid), stdio via serial terminal

**Memory Layout (memmap.ld):**
```
FLASH (16MB)         : Code, constants, read-only data
RAM (512KB)          : copy_to_ram execution, stack, heap
PSRAM (8MB @ QMI)    : i8086 RAM/ROM/VRAM emulation (736KB used, 7.26MB available)
SCRATCH_X (4KB)      : Core0 fast scratchpad
SCRATCH_Y (4KB)      : Core1 fast scratchpad
```

**Performance implications:**
- All critical code runs from internal 512KB RAM (zero wait states)
- PSRAM accessed via QMI (Quad SPI Memory Interface) with ~6 cycle latency
- SCRATCH_X/Y can be used for ultra-fast temporary storage in IRQ handlers

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
- System initialization (overclock to 500 MHz, USB Host init, PSRAM init, SD card mount, VGA init)
- Launches Core1 via `multicore_launch_core1(bus_handler_core)`
- Main loop responsibilities:
  - **USB HID processing (TinyUSB stack)**:
    - USB keyboard: Converts USB HID reports → XT Scancodes → i8042 controller (via `handleScancode()`)
    - USB mouse: Converts HID reports → Microsoft Serial Mouse protocol → COM1 (ports 0x3F8-0x3FF)
    - Supports up to 4 HID devices via USB hubs (CFG_TUH_HID = 4)
    - Plug & Play device detection via `tuh_hid_mount_cb()` / `tuh_hid_umount_cb()`
  - CGA/PCjr hardware video output: Renders VIDEORAM to VGA port (GPIO 30-37) at 60 FPS via PIO1 + DMA
    - Supports multiple video modes: text 80×25, CGA 320×200×4, PCjr 160×200×16 and 320×200×16
  - Debug commands: R (reset), B (bootloader), M (memory dump), V (video dump), C (CTTY mode), P (CGA registers)
  - CTTY mode: Direct serial terminal mode via COM1/USB
  - PC Speaker: PWM output on GPIO 46 controlled by i8253 Channel 2 emulation (port 0x61)
  - SD Card: Loads .img disk images (floppy and HDD) from FAT filesystem for FDC and XTIDE emulation

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
- GPIO 30-37: VGA_DATA[0:7] (8-bit hardware CGA video output, controlled by PIO1)
- GPIO 40: SD_MISO (SPI1 - SD Card Master In Slave Out)
- GPIO 41: SD_CS (SPI1 - SD Card Chip Select)
- GPIO 42: SD_SCK (SPI1 - SD Card Clock)
- GPIO 43: SD_MOSI (SPI1 - SD Card Master Out Slave In)
- GPIO 46: BEEPER (PC Speaker PWM output, emulates i8253 PIT Channel 2)
- GPIO 47: PSRAM_CS (Chip Select for external PSRAM, 736KB RAM via QMI)

**Important:**
- Pin assignments in `i8086_bus.pio` must match hardware wiring
- RP2350B is 5V tolerant on inputs, allowing direct connection to i8086 (5V logic) without level shifters
- VGA video driver uses PIO1 with R-2R DAC for analog signal generation
- SD Card uses SPI1 bus for loading floppy disk images and file systems
- Total GPIO usage: 44 pins (0-15: AD bus, 16-19: A bus, 20-29: control signals, 30-37: VGA, 40-43: SD, 46: speaker, 47: PSRAM)

### PIO Resources Allocation

RP2350B has 2 PIO blocks (PIO0, PIO1), each with 4 state machines and 32 instructions memory. Current usage:

**PIO0 (Bus Controller):**
- State Machine 0: i8086 bus protocol handler (i8086_bus.pio)
- Instruction memory: 29/32 instructions used (3 reserved for future)
- IRQ usage: IRQ0 (writes), IRQ1 (reads), IRQ3 (INTA cycles)
- Runs at 500 MHz (clkdiv = 1.0) for maximum timing precision
- Sideset: READY signal for wait state control

**PIO1 (Video Controller):**
- State Machine: VGA timing and signal generation (vga-nextgen driver)
- Generates hsync/vsync timing for CGA-compatible modes
- DMA-driven VIDEORAM → VGA_DATA transfer at 60 FPS
- Outputs 8-bit RGBI signal (GPIO 30-37) with R-2R DAC conversion
- Runs at VGA pixel clock frequency (25.175 MHz for standard VGA)

**Available Resources:**
- PIO0: State machines 1-3 available for future expansion (ISA bus, additional peripherals)
- PIO1: State machines 1-3 available for future features (audio, additional video modes)

## Project Structure

**Build System (CMakeLists.txt):**
- **Main executable**: Compiled from `src/main.c`, `src/cpu.c`, `src/cpu_bus.c`, `src/common.c`
- **PIO generation**: `i8086_bus.pio` auto-compiled to header via `pico_generate_pio_header()`
- **Drivers linked**:
  - `drivers/graphics` - Graphics utilities
  - `drivers/vga-nextgen` - VGA video output driver (PIO1 based)
  - `drivers/sdcard` - SD Card SPI driver
  - `drivers/fatfs` - FAT filesystem support
  - `drivers/usbhid` - USB HID driver (TinyUSB, RELEASE mode only)
- **Pico SDK libraries**:
  - `pico_runtime`, `pico_stdlib`, `pico_stdio` - Core SDK
  - `pico_multicore` - Dual-core support
  - `hardware_pio`, `hardware_pwm` - Hardware peripherals
  - `tinyusb_host`, `tinyusb_board` - USB Host stack (RELEASE only)
- **Build artifacts**: `bin/` directory with auto-incremented build numbers
- **Version control**: Git branch and commit hash embedded in binary via compile definitions

**Compile-time definitions:**
- `VGA` - Enables VGA video driver
- `VGA_BASE_PIN=30` - VGA data output starts at GPIO 30
- `SDCARD_PIN_SPI0_MISO=40`, `CS=41`, `SCK=42`, `MOSI=43` - SD Card SPI pins
- `PICO_BUILD_NAME`, `PICO_GIT_BRANCH`, `PICO_GIT_COMMIT` - Version information

## Code Organization

**main.c** - Entry point and Core0 main loop:
- System initialization (USB Host via TinyUSB, PSRAM, VGA video, SD card mounting)
- **USB HID integration**:
  - Calls `tuh_task()` in main loop for TinyUSB processing
  - Receives scancodes from USB keyboard via `handleScancode()` callback
  - USB mouse data automatically sent to COM1 by USB HID driver
- CGA video rendering at 60 FPS with multiple mode support:
  - Text mode: 80×25 characters
  - CGA graphics: 320×200×4 colors
  - PCjr graphics: 160×200×16 colors, 320×200×16 colors (Tandy 1000 compatible)
- Keyboard input processing:
  - USB keyboard: USB HID → XT Scancodes → i8042 (via USB HID driver)
  - Serial terminal: ASCII → Scancode conversion (for debug)
- Debug commands (uppercase): R, B, M, V, C, P
- CTTY mode support (direct COM1/USB terminal mode)
- SD card management: loads floppy and HDD disk images (.img) from FAT filesystem

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
- Device structures: `i8259_s`, `i8253_s`, `dma_channel_s`, `i8272_s`, `ide_s`, `uart_16550_s`, `mc6845_s`, `cga_s`
- Helper functions: `write_to()` for BHE support

**common.c** - Binary file imports and global data structures:
- **IMPORT_BIN macro**: Imports binary files directly into FLASH memory at compile time
  - Syntax: `IMPORT_BIN("path/to/file.bin", symbol_name)`
  - Creates read-only array `uint8_t symbol_name[]` in `.flashdata` section
  - **CRITICAL**: Imported data is READ-ONLY (stored in FLASH, not RAM)
  - **Cannot be modified at runtime** - any "patching" must be done via conditional logic in memory_read()
  - Example: `IMPORT_BIN("./binary/GLABIOS.ROM", BIOS)` creates `extern uint8_t BIOS[]`
- **Current binary imports**:
  - `BIOS[]` - Turbo XT BIOS v3.1 (8KB, read-only)
  - `IDE[]` - XTIDE ROM (read-only)
  - `FLOPPY[]` - Floppy disk image (read-only)
- **Tandy mode patching example**: Since BIOS is read-only, patching is done in `memory_read()`:
  ```c
  // In memory.h - dynamic "patch" via conditional return
  if (address == 0xFFFFE && settings.tandy_enabled) {
      return 0xFF;  // Return patched value instead of BIOS[8190]
  }
  ```
- Global device structures: `i8259`, `i8253`, `i8272`, `dma_channels[]`, `uart`, `mc6845`, `cga`, `ide`
- RAM arrays: `RAM[]`, `UMB[]`, `VIDEORAM[]` (these ARE writable, in PSRAM/SRAM)

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
- **Channel 0 (System Timer)**: One-shot interrupt on terminal count via hardware `repeating_timer`, uses `irq0_timer_callback()` for dynamic timer control
- **Channel 1 (Memory Refresh)**: Emulated for compatibility (not actively used)
- **Channel 2 (PC Speaker)**: Controls PWM output on GPIO 46 via port 0x61 bits 0-1, frequency = 1193182 / reload_value
- Dynamic timer_interval calculation based on reload value
- PC Speaker hardware controlled by `pwm_set_gpio_level()` when enabled via port 0x61

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

**hardware/ide.h** - XTIDE/ATA Hard Disk Controller:
- Full XTIDE controller emulation at ports 0x300-0x308
- LBA28 and CHS addressing modes support
- Implemented commands: IDENTIFY DEVICE (0xEC), READ SECTOR(S) (0x20), WRITE SECTOR(S) (0x30), READ VERIFY (0x40/0x41)
- Works with HDD image file (hdd.img) on SD card via FatFS
- 512-byte sector buffer for efficient I/O operations
- Status register with BUSY, DRDY, DSC, DRQ, ERR flags
- Multi-sector transfers with automatic LBA increment
- Geometry: 16 heads, 63 sectors per track (standard CHS)
- Functions: `ide_read()`, `ide_write()`, `ide_identify()`, `read_sector()`, `write_sector()`

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

**drivers/vga-nextgen/** - Hardware VGA video driver:
- Uses PIO1 to generate VGA timing signals (hsync, vsync)
- Outputs 8-bit RGBI signal on GPIO 30-37 (VGA_BASE_PIN)
- Supported video modes:
  - **CGA Text**: 80×25 characters (16 colors)
  - **CGA Graphics**: 320×200×4 colors (standard IBM CGA)
  - **PCjr Low-Res**: 160×200×16 colors (IBM PCjr/Tandy 1000)
  - **PCjr High-Res**: 320×200×16 colors (IBM PCjr/Tandy 1000)
- R-2R DAC resistor ladder converts digital to analog VGA signal
- DMA-based refresh from VIDEORAM at 60 FPS
- Mode switching via MC6845 CRTC registers (ports 0x3D4/0x3D5)

**drivers/sdcard/** - SD Card SPI driver:
- Uses SPI1 bus (GPIO 40-43: MISO, CS, SCK, MOSI)
- Mounts FAT12/FAT16/FAT32 filesystem from SD card at boot
- Loads floppy disk images (.img files) for i8272A FDC emulation
- Loads hard disk images (hdd.img) for XTIDE controller emulation
- Supported disk image formats:
  - 160KB (SS/SD, 8 sectors/track)
  - 180KB (SS/SD, 9 sectors/track)
  - 320KB (DS/SD, 8 sectors/track)
  - 360KB (DS/DD, 9 sectors/track, standard 5.25")
  - 720KB (DS/DD, 9 sectors/track, 3.5")
  - 1.2MB (DS/HD, 15 sectors/track, 5.25")
  - 1.44MB (DS/HD, 18 sectors/track, 3.5")
- Auto-detection of disk geometry from image file size
- Configurable pin assignments via CMakeLists.txt compile definitions
- Integrated with FAT filesystem driver for DOS compatibility
- Primary use: boot DR-DOS 7 and other DOS-based operating systems from floppy/HDD images stored on SD card
- HDD images support: LBA28 addressing with up to 8GB capacity (limited by FatFS file size)

**drivers/usbhid/** - USB HID driver (TinyUSB stack):
- **tusb_config.h**: TinyUSB configuration
  - CFG_TUH_ENABLED = 1 (USB Host mode enabled)
  - CFG_TUH_HID = 4 (supports up to 4 HID interfaces - keyboard + mouse)
  - CFG_TUH_HUB = 1 (USB hub support, up to 4 devices)
  - CFG_TUH_DEVICE_MAX = 4 (maximum 4 devices simultaneously)
- **usb_to_xt_scancodes.h**: USB HID → XT Scancode conversion table
  - Full keyboard mapping (104-key layout)
  - Supports all keys: F1-F12, NumPad, arrows, modifiers (Shift, Ctrl, Alt)
  - Extended scancodes for special keys (Insert, Delete, Home, End, etc.)
- **hid_app.c**: USB HID application layer
  - `tuh_hid_mount_cb()`: Detects keyboard/mouse on connection
  - `tuh_hid_report_received_cb()`: Processes HID reports
  - `tuh_hid_umount_cb()`: Handles device disconnection
  - Keyboard: Converts USB HID → XT Scancodes → calls `handleScancode()` in main app
  - Mouse: Converts HID → Microsoft Serial Mouse protocol (3 bytes) → COM1 UART
  - Auto-detection: When DOS driver sets DTR/RTS on COM1, sends identifier "M"
- Works with DOS mouse drivers: CTMOUSE, MOUSE.COM (Microsoft Serial Mouse compatible)

**setup.h / setup.c** - Configuration menu system:
- **settings_s structure**: Project configuration (versioned for compatibility)
  - `version` - Settings file version (uint16_t, current = SETTINGS_VERSION)
  - `cpu_freq_index` - CPU frequency selector (0=1MHz, 1=4.75MHz, 2=6MHz)
  - `tandy_enabled` - IBM PCjr/Tandy 1000 compatibility mode
  - `fda`, `fdb`, `hdd` - Disk image paths (256 chars each)
- **Settings versioning**: File format validation prevents loading incompatible configs
  - Increment `SETTINGS_VERSION` when changing `settings_s` structure
  - `load_settings()` checks version and file size - rejects mismatches
  - Rejected configs → use default settings (prevents crashes from old/corrupt files)
- **File browser**: SD card file selection with directory navigation
- **Menu system**: Arrow key navigation, ENTER/ESC controls
- **Persistence**: Saves to `/XT/config.sys` on SD card (binary format)
- **Called before Core1 launch**: Settings loaded → SETUP menu → Core1 starts with selected config

## Memory Map

**i8086 Address Space (1MB):**
```
0x00000 - 0xB7FFF : RAM (736KB) - 4-byte aligned, external PSRAM via QMI
0xB8000 - 0xBBFFF : Video RAM CGA (16KB) - 4-byte aligned
0xBC000 - 0xFDFFF : Unmapped (returns 0xFFFF)
0xFE000 - 0xFFFFF : ROM Turbo XT BIOS v3.1 (8KB) - 4-byte aligned
```

Reset vector at 0xFFFF0 points into ROM.

**Special Memory Addresses (Tandy/PCjr compatibility):**
- **0xFC000**: Tandy signature detection
  - Returns `0x21` when `settings.tandy_enabled == 1`
  - Returns `0xFFFF` (unmapped) when Tandy mode disabled
  - Used by software to detect IBM PCjr/Tandy 1000 hardware
- **0xFFFFE**: BIOS compatibility byte (dynamic patching)
  - Returns `0xFF` when `settings.tandy_enabled == 1`
  - Returns original BIOS value when Tandy mode disabled
  - BIOS is read-only in FLASH - patching done via conditional logic in `memory_read()`
  - See `common.c` section above for why IMPORT_BIN arrays cannot be modified at runtime

**RP2350B Physical Memory (memmap.ld):**
```
0x10000000 - 0x10FFFFFF : FLASH (16MB) - Code and constants
0x20000000 - 0x2007FFFF : RAM (512KB) - Execution, stack, heap (copy_to_ram)
0x20080000 - 0x20080FFF : SCRATCH_X (4KB) - Core0 fast scratchpad
0x20081000 - 0x20081FFF : SCRATCH_Y (4KB) - Core1 fast scratchpad
0x11000000 - 0x117FFFFF : PSRAM (8MB) - i8086 memory emulation
                          Currently used: 736KB RAM + 16KB VRAM + 8KB ROM = 760KB
                          Available for expansion: 7.26MB (can support extended/expanded memory)
```

**Potential expansions:**
- **EMS (Expanded Memory Specification)**: Up to 7MB of page-switched memory
- **XMS (Extended Memory Specification)**: High memory area (HMA) emulation
- **RAM disk**: Fast storage in unused PSRAM
- **Additional video buffers**: For page flipping or multiple video modes

## I/O Ports

**Current emulated ports:**
- 0x00-0x0F: Intel 8237 DMA Controller (4 channels, auto-init, memory-to-memory)
- 0x20-0x21: Intel 8259A PIC (IRQ0-IRQ7 management, ICW/OCW support)
- 0x40-0x43: Intel 8253 PIT (3 channels: timer, memory refresh, PC speaker)
- 0x60: Keyboard Data Port (i8042 emulation, scancode read/write)
- 0x61: System Control Port B (keyboard reset, speaker enable via bit 0-1, timer gate)
- 0x64: Keyboard Status Port (i8042 emulation, output buffer status)
- 0x81/82/83/87: DMA Page Registers (20-bit addressing for DMA channels)
- 0x300-0x308: XTIDE/ATA Hard Disk Controller (LBA28/CHS, sector I/O, status/command)
- 0x3D0-0x3DF: CGA/MC6845 CRTC registers (video timing, cursor position)
- 0x3F0-0x3F7: Intel 8272A FDC (floppy disk controller, DMA channel 2)
- 0x3F8-0x3FF: 16550 UART (COM1, used for CTTY mode and DOS serial I/O)

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
- 4-byte aligned memory arrays for fast access (RAM, VIDEORAM, BIOS)
- Direct 16-bit memory access via pointer casting
- Inline functions (`__force_inline`) for hot paths
- `likely()` / `unlikely()` hints for branch prediction
- Minimal volatile variable usage (local copies in hot paths)
- PIO instruction count: 29/32 used (bus controller)
- Hardware repeating_timer for IRQ0 instead of polling loop
- `__wfe()` on Core1 for power efficiency when idle
- DMA-driven VGA refresh (zero CPU overhead on Core0)
- PSRAM accessed via QMI for high-speed 736KB RAM
- SD Card I/O performed during initialization only (zero runtime overhead)
- PC Speaker uses hardware PWM (no CPU polling required)

**Core utilization:**
- Core0: ~15-20% (VGA handled by DMA, USB HID via TinyUSB, keyboard/mouse input sporadic)
- Core1: ~45% worst case (bus handling at 6 MHz, device emulation)
- Total system headroom: ~35-40% available for future features

**USB HID performance:**
- TinyUSB processes HID reports at ~1ms intervals (1000 Hz polling)
- Keyboard latency: <2ms (USB poll + XT conversion + i8042 injection)
- Mouse latency: <2ms (USB poll + Serial Mouse conversion + COM1 UART)
- Zero impact on i8086 bus performance (all processing on Core0)

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
- PIO program auto-recompiled via `pico_generate_pio_header()` during build
- Output: `src/i8086_bus.pio.h` (auto-generated, do not edit manually)

### Build System Notes

**Important CMakeLists.txt features:**
- **Auto-versioning**: Git branch and commit hash embedded in binary
  - Accessible via `PICO_BUILD_NAME`, `PICO_GIT_BRANCH`, `PICO_GIT_COMMIT` defines
  - Displayed in boot messages and debug output
- **Build numbering**: Post-build script auto-increments `.build_number` file
  - Output format: `RP8086-<branch>-<build_number>.uf2`
  - Useful for tracking firmware versions
- **Conditional USB**: USB HID only compiled in RELEASE mode
  - DEBUG mode: `stdio_usb` enabled for debugging via USB serial
  - RELEASE mode: `tinyusb_host` + `usbhid` enabled for USB keyboard/mouse
- **Memory usage reporting**: Linker prints memory usage summary after build
  - Shows FLASH, RAM, PSRAM usage
  - Helps track size optimizations

**Compile-time configuration:**
- Pin assignments are compile-time constants (cannot be changed at runtime)
- Change pin mappings in `CMakeLists.txt` → rebuild required
- PIO programs depend on specific pin locations → test thoroughly after changes

**Pico SDK libraries used:**
- `pico_runtime` - Core runtime initialization (clocks, PLLs, voltage control)
- `pico_stdlib` - Standard library (GPIO, time, interrupts)
- `pico_stdio` - Standard I/O (USB serial for debugging)
- `pico_multicore` - Dual-core support (Core0/Core1 synchronization)
- `hardware_pio` - PIO state machine API (i8086 bus controller, VGA timing)
- `hardware_pwm` - PWM API (i8086 clock generator, PC Speaker)
- `tinyusb_host` - USB Host stack (keyboard/mouse support, RELEASE mode only)
- `tinyusb_board` - Board-specific USB configuration (RELEASE mode only)

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
- Scancodes can come from two sources:
  1. USB keyboard: USB HID driver converts to XT scancodes and calls `handleScancode()`
  2. Serial terminal: ASCII input converted to scancodes for debugging

### USB HID Integration (TinyUSB)

**Architecture:**
- USB Host stack runs on Core0 via `tuh_task()` in main loop
- HID reports processed in interrupt context by TinyUSB
- Callbacks in `drivers/usbhid/hid_app.c`:
  - `tuh_hid_mount_cb()`: Called when USB HID device connected
  - `tuh_hid_report_received_cb()`: Called on each HID report (keyboard keys, mouse movement/buttons)
  - `tuh_hid_umount_cb()`: Called when USB HID device disconnected

**Keyboard flow:**
```
USB Device → TinyUSB Host Stack → tuh_hid_report_received_cb() →
USB HID Scancode → usb_to_xt_scancodes[] lookup → XT Scancode →
handleScancode() → i8042 controller → Port 0x60
```

**Mouse flow:**
```
USB Device → TinyUSB Host Stack → tuh_hid_report_received_cb() →
HID Mouse Report (X, Y, buttons) → Microsoft Serial Mouse packet (3 bytes) →
uart_write_byte() → COM1 (ports 0x3F8-0x3FF) → DOS mouse driver
```

**Mouse auto-detection:**
1. USB mouse plugged in → `tuh_hid_mount_cb()` sets flag `usb_mouse_connected = true`
2. DOS driver (CTMOUSE, MOUSE.COM) starts and sets DTR/RTS on COM1
3. `uart16550.h` detects DTR/RTS change, checks `is_usb_mouse_connected()`
4. If mouse connected → sends Microsoft Serial Mouse identifier "M" to DOS driver
5. DOS driver recognizes mouse and starts reading movement/button data from COM1

**Supported USB HID devices:**
- Standard USB keyboards (104-key layout, full scancode support)
- Standard USB mice (3-button + scroll wheel)
- USB hubs (up to 4 ports, CFG_TUH_DEVICE_MAX = 4)
- Combined keyboard+mouse devices (e.g., laptops with touchpad)

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
