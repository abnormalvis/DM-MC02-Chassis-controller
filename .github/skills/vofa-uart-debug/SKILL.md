---
name: vofa-uart-debug
description: "Use when debugging VOFA serial communication, USART1/USART2 routing, UART errors, RX parsing, transmission frequency, or current-feedback display issues in the DM-MC02 STM32H7 project."
---

# VOFA UART Debug

Use this skill when VOFA telemetry or parameter RX behaves strangely on the DM-MC02 chassis controller project.

## Start Here

1. Verify the actual runtime path, not just CubeMX intent.
2. Separate RX, TX, and printf roles before changing code.
3. Reduce the sender to the smallest stable test case.
4. Inspect UART error callbacks before blaming the parser.
5. Confirm whether the displayed value is a control target, a feedback value, or just a sensor offset.

## Practical Debug Order

1. Check which UART is used at runtime by `VOFA_Begin()`, `VOFA_SendFloats()`, and `HAL_UART_RxCpltCallback()`.
2. Confirm the wiring is cross-connected correctly: MCU TX to USB-TTL RX, MCU RX to USB-TTL TX, and shared ground.
3. Keep the VOFA sender independent from the PID loop if the loop timing is slower or noisy.
4. Start with a small frame, such as 2 to 4 floats, and a modest rate like 10 to 100 Hz.
5. Keep other debug prints off the same UART when you are testing VOFA.

## Compare With the Reference Project

The `comprehensive-裸机调参` project uses a simpler pattern:

1. One UART is dedicated to VOFA parameter RX.
2. Waveform sending is periodic and lightweight.
3. The frame count is small.
4. Debug output is kept separate.

When the main project is unstable, first imitate this minimal shape instead of adding more telemetry.

## Error-First Rule

If `HAL_UART_ErrorCallback()` is reached, do not keep chasing the parser first.

1. Record the HAL error code.
2. Record the USART ISR snapshot.
3. Decode whether the issue is `PE`, `NE`, `FE`, or `ORE`.
4. Fix the electrical or timing cause before expanding the parser.

## How To Read The `measure` Value

In this project, the VOFA `measure` channel is the current feedback value, not the PID target.

1. It comes from ADC DMA data via `ADC_GetValues()`.
2. It is converted by `ADC_RawToCurrent_mA()`.
3. The selected feedback source is controlled by `CURRENT_PID_FEEDBACK_CH`.

If the motor clearly reacts to PID and target changes but `measure` stays nearly constant, check these first:

1. Whether the selected ADC channel is the one actually wired to the motor current sensor.
2. Whether the raw ADC values are changing even if the converted current looks flat.
3. Whether the current sensor has a large zero-current offset or the motor is lightly loaded.
4. Whether the sign or channel mapping is wrong.

Do not assume a stable `measure` means VOFA is broken. It can also mean the sensor reading is real but not the quantity you expected.

## Good Defaults For This Repo

1. Keep VOFA transmission at 10 ms or slower until the link is stable.
2. Keep the frame small when validating UART health.
3. Keep printf on a different UART from VOFA.
4. Prefer independent timing for telemetry instead of tying it to the PID execution branch.
5. Use the reference project as a minimal baseline when comparing behaviour.