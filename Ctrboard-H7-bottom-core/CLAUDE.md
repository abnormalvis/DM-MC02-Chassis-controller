# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is bare-metal firmware for the **DM-MC02 chassis motor controller** based on **STM32H723RBTx** (Cortex-M7, 550 MHz). It controls two DC motors using PID current control, receives commands via CRSF (RC receiver) or offboard protocol, and streams telemetry via VOFA over USART10.

No RTOS is used. Timing is driven by the **DWT cycle counter** at 10 kHz.

## Build Commands

```bash
# Configure (from Ctrboard-H7-bottom-core/)
cmake --preset Debug
cmake --preset Release

# Build
cmake --build --preset Debug
cmake --build --preset Release

# Output artifacts (ELF / HEX / BIN for flashing)
build/Debug/Ctrboard-H7-bottom-core.elf
build/Debug/Ctrboard-H7-bottom-core.hex
```

Requires `arm-none-eabi-gcc` and `ninja` in PATH. Toolchain: `cmake/gcc-arm-none-eabi.cmake`.

## Host Unit Tests

Tests in `tests/` compile for the host (native x86_64), not for the MCU:

```bash
mkdir -p tests/build && cd tests/build
cmake ..
cmake --build .

./protocol_smoke_test
./motor_control_task_test
./app_tasks_chain_test
```

Each test binary links only the relevant `Core/Src/app/` file(s) plus stubs from `tests/stubs/`.

## Architecture

### Control Loop (`Core/Src/main.c`)

The `main()` function runs a bare-metal super-loop. Task dispatch uses the DWT cycle counter:
- **10 kHz** — `MotorControlTask_Process()` (100 µs budget, hardest deadline)
- **1 kHz** — `CRSFTask_Process()`, `SafetyTask_Process()`
- **50 Hz** — `VofaDebug_Process()` (USART10 telemetry)
- **10 Hz** — `OledDebug_Process()` (SSD1306 I2C display)

### Task Modules (`Core/Src/app/`)

| File | Role |
|------|------|
| `motor_control_task.c` | Dual-motor PID current control, PWM output |
| `crsf_task.c` | CRSF RC receiver via UART7 DMA + IDLE IRQ |
| `safety_task.c` | Fault bits, e-stop, timeout watchdogs |
| `app_tasks.c` | Command routing between tasks, mode arbitration |
| `vofa_debug.c` | 9-float VOFA frame over USART10 @ 50 Hz |
| `oled_debug.c` | SSD1306 status display |
| `protocol_adapter.c` | Translates legacy v1 ↔ unified v0 offboard protocols |

### Motor Driver Hardware Constraints

These are fixed by the DM-MC02 driver board and must be respected in all firmware:

- **PWM frequency**: 20 kHz (TIM ARR=999 @ 550 MHz with appropriate prescaler)
- **PWM duty cycle**: **0–99% only** — never write ARR (full scale) to the compare register; the driver requires a switching waveform, not a constant level
- **Per-motor topology**: two channels — `FWD` and `REV`; only one active at a time; both at 0 = **active brake** (motor shorted, not free-spin)
- **Current sense**: 0–3.3 V centered at 1.65 V (= 0 A); ADC reads 0–65535 (16-bit), so 32768 = 0 A
- Pin map: TIM1 CH1 = motor A fwd (PA8), TIM1 CH3 = motor A rev (PA9), TIM1 CH4 = motor B fwd (PA0 via TIM2?), TIM2 CH1 = motor B rev — verify against `DM-MC02.ioc`

### Motor Control (`motor_control_task.c`)

**ADC → PID → PWM pipeline at 1 kHz:**
1. ADC1 (Ch19 = motor A, Ch4 = motor B) is triggered by TIM1 TRGO; results land in DMA buffer
2. Raw ADC → current in 0.01 A units: `(raw - 32768) / 64`  (32768 = midpoint = 0 A)
3. RC throttle/steering channels → `target_a = throttle + steering`, `target_b = throttle - steering`, clamped to ±300
4. PI controller (Kp=1.2, Ki=35.0, Kd=0.0, max integral=800, max output=1000)
5. PWM compare value = `abs(drive_cmd) * 989 / 1000` (caps at 98.9% duty, never reaches ARR=999)

### Safety & Fault Model (`safety_task.c`, `app_tasks.h`)

System states: `MODE_IDLE → MODE_RC → MODE_OFFBOARD → MODE_BRAKE_PROTECT`

Fault bits (OR'd into `g_fault_flags`):
- `FAULT_RC_TIMEOUT` — RC link lost >100 ms
- `FAULT_OFFBOARD_TIMEOUT` — PC link lost >100 ms
- `FAULT_OVERCURRENT_A/B` — motor overcurrent
- `FAULT_IMU_FAULT`, `FAULT_PROTOCOL_ERR`, `FAULT_MODE_CONFLICT`

Any active fault downgrades to `MODE_BRAKE_PROTECT` and zeros PWM output.

### Communication Peripherals

| Interface | Peripheral | Baud / Freq | Purpose |
|-----------|-----------|-------------|---------|
| CRSF RC | UART7 | 420000 | RC receiver (DMA circular + IDLE IRQ) |
| VOFA telemetry | USART10 | — | 9-float debug stream |
| Offboard protocol | USART2/3 | — | Host computer control |
| OLED display | I2C1 | — | SSD1306 128×64 debug display |
| IMU | SPI | — | BMI088 (Device/) |
| CAN | FDCAN1 | — | Currently inactive; infrastructure in `protocol_adapter.c` |

### Peripheral Init Ownership

All `MX_*_Init()` functions are CubeMX-generated and live in `Core/Src/{gpio,adc,dma,tim,usart,i2c,spi,octospi}.c`. Do not hand-edit these files — regenerate from `DM-MC02.ioc` via STM32CubeMX instead. Exception: `Core/Src/app/` files are hand-written application code.

## Validation

Hardware test matrix is in `docs/h723_baremetal_test_plan.md` (T00–T06). All of T00, T01, T02, T04, T05, T06 must pass before PID tuning begins.

## Key Constraints

- **DWT timing is safety-critical.** The 10 kHz motor control loop must not be stalled. Never add blocking waits or lengthy printf calls inside `MotorControlTask_Process()`.
- **ADC/DMA circular buffer** is read directly from `g_adc_dma_buf[2]` in `motor_control_task.c`; the buffer is volatile and updated every PWM cycle.
- **CRSFTask** uses a DMA circular buffer with IDLE-line IRQ. The parsed channel data is accessed via `CRSFTask_GetChannels()` — do not access the DMA buffer directly.
- **`app_tasks.c`** is the single arbitration point for control mode transitions; do not call `MotorControlTask_SetTarget()` from outside `app_tasks.c` except in tests.
