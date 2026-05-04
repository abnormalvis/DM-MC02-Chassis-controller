# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Layout

```
DM-MC02-Chassis-controller/
├── 0.cubeMX配置参考/               ← ACTIVE development project (STM32H750VBT6)
│   └── 0.cubeMX配置参考/           ← Actual source root (nested same-name dir)
├── Motor_ Tuning_PID/              ← Reference: RC parsing + cascade PID (STM32F4, CAN motors)
├── comprehensive-裸机调参/          ← Reference: bare-metal PID + VOFA protocol (STM32F1)
├── Ctrboard-H7-bottom-core/        ← Reference: CRSF task, safety task, app architecture (STM32H723)
├── docs/                            ← Hardware manuals, debug guide, protocol spec
└── H743VGT6/                        ← Supplementary H743 files
```

**Active project root**: `0.cubeMX配置参考/0.cubeMX配置参考/`
**Build tool**: Keil MDK-ARM — open `MDK-ARM/STM32H743.uvprojx` in Keil uVision to build and flash.

## Motor Driver Hardware Constraints

These are fixed by the physical motor driver board and must be respected in all firmware:

| Parameter | Value |
|-----------|-------|
| MCU PWM output voltage | 3.3 V logic |
| Driver input voltage range | 3.3 V – 12 V ✓ |
| **Maximum PWM frequency** | **20 kHz** — do not exceed |
| **PWM duty cycle range** | **0 – 99%** — never write ARR (full-scale); the driver needs a switching waveform |
| Current sense output | 0 – 3.3 V, centered at **1.65 V = 0 A** (bidirectional Hall-effect sensor) |
| Brake condition | **Both PWM channels = 0 simultaneously** → active brake (motor shorted, not free-spin) |

**Per-motor dual-channel PWM topology:**
- `PWM_FWD` drives forward; `PWM_REV` drives reverse. Only one is non-zero at a time.
- Setting both non-zero simultaneously is invalid and may damage the driver.
- Both at 0 = **active brake**.
- A constant GPIO level (not PWM) will not spin the motor.

## Active Project: `0.cubeMX配置参考/0.cubeMX配置参考/`

See `0.cubeMX配置参考/0.cubeMX配置参考/CLAUDE.md` for full details.

Quick summary:
- STM32H750VBT6 @ 480 MHz
- Current inner loop (PI, 50 Hz) and VOFA telemetry (USART2, 921.6 k) already implemented in `Core/Src/main.c`
- PWM outputs on TIM1/TIM2 configured but not yet started — motor drive code is the next step
- CRSF RC receiver and speed outer loop not yet implemented

## Reference Projects

### `Motor_ Tuning_PID/` — RC parsing + cascade PID (STM32F4)

Uses CAN-bus DJI motors (not PWM drivers) and SBUS RC protocol (not CRSF), but the patterns are directly portable.

| File | What to reuse |
|------|--------------|
| `application/pid.h/.c` | `pid_type_def` struct with `POSITION_PID`/`DELTA_PID`, error[3] history, Dbuf[3], separate Pout/Iout/Dout fields |
| `application/remote_control.h/.c` | `RC_ctrl_t` structure (5 channels `ch[5]`, 2 switches `s[2]`); DMA circular RX pattern; error detection (any channel >±700 → zero all) |
| `application/enginer_task.h/.c` | Cascade PID pattern: outer speed PID output → inner loop setpoint; cumulative angle tracking across encoder rollovers |
| `application/struct_typedef.h` | `fp32`/`fp64` type aliases |
| `Core/Src/main.c` `map()` | Linear range mapping: `map(x, in_min, in_max, out_min, out_max)` |

**RC channel conventions**: raw value 364–1684, center = 1024, working range ±660. Switch states: UP=1, MID=3, DOWN=2. Safety pattern: `s[0] == 2` → all outputs = 0.

### `comprehensive-裸机调参/` — bare-metal VOFA + PWM motor (STM32F1)

| File | What to reuse |
|------|--------------|
| `Bsp/bsp_uart.c/.h` | VOFA JustFloat frame: N × float32 + tail `00 00 80 7F`; parameter protocol `#P<id>=<value>!` |
| `Hardware/motor.h/.c` | Dual-channel per-motor PWM pattern; duty range −100 to +100 |
| `Hardware/encoder.h/.c` | Speed feedback via timer capture; `Read_Speed()` interface |

### `Ctrboard-H7-bottom-core/` — CRSF + safety (STM32H723)

| File | What to reuse |
|------|--------------|
| `Core/Src/app/crsf_task.c/.h` | CRSF protocol parsing (different from SBUS), UART7 DMA + IDLE-IRQ pattern |
| `Core/Src/app/safety_task.c/.h` | Fault bits, e-stop, 100 ms timeout → brake |
| `Core/Src/app/app_tasks.c/.h` | Mode arbitration: `MODE_IDLE → MODE_RC → MODE_OFFBOARD → MODE_BRAKE_PROTECT` |

## Docs

- Debug pin map & task priorities: `docs/调试手册.md`
- Board user manual: `docs/达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf`
- Protocol spec: `docs/ROS底板协议.pdf`
