# H723 Bare-Metal Test Plan

## 1. Purpose
This document defines board-level validation for the migrated single-chip H723 bare-metal control path:
- ADC1 dual-channel sampling (TIM1 TRGO trigger + DMA circular)
- Motor control output (TIM1/TIM2 PWM)
- CRSF input (UART7)
- VOFA telemetry output (USART10)
- Safety behavior under input loss and abnormal state

## 2. Baseline
- Target board: STM32H723
- Build profile: Debug
- Required build task: CMake Configure+Build Debug
- Current expected build state: pass (linker RWX warning may exist)

## 3. Test Environment
- ST-Link debugger
- Logic analyzer or oscilloscope (PWM and trigger timing)
- VOFA host tool on PC (USART10)
- CRSF transmitter/receiver pair (UART7)
- Power supply with current limit

## 4. Preconditions
1. Firmware built and flashed successfully.
2. Board boots without hard fault.
3. Motor output stage disconnected or power-limited for first run.
4. Safety stop condition accessible (switch or power cut).

## 5. Test Matrix

## T00 Build and Flash
- Goal: Confirm software baseline is reproducible.
- Steps:
  1. Run Debug configure+build.
  2. Flash ELF to target.
  3. Reset and attach debugger.
- Expected:
  - Build succeeds.
  - Flash succeeds.
  - Main loop executes.
- Fail signature:
  - Build fail, flash fail, or immediate fault after reset.
- Rollback:
  - Reflash previous known-good ELF.

## T01 PWM Output Presence
- Goal: Verify four PWM channels are active and writable.
- Steps:
  1. Probe PWM pins with scope.
  2. Observe frequency and duty update when control command changes.
- Expected:
  - PWM frequency matches configured target (nominal 10 kHz).
  - Duty changes within limit and no stuck output.
- Fail signature:
  - No waveform, wrong frequency, or duty clipping at fixed value.
- Rollback:
  - Force zero duty and disable motor stage.

## T02 ADC DMA Continuous Update
- Goal: Verify ADC samples are updated continuously by DMA.
- Steps:
  1. Watch ADC DMA buffer in debugger.
  2. Inject analog change on both channels.
  3. Confirm buffer reflects both channels independently.
- Expected:
  - Buffer updates continuously.
  - Channel A and B are not mirrored unless inputs are equal.
- Fail signature:
  - Buffer static, channel swap, or both channels identical unexpectedly.
- Rollback:
  - Stop PWM updates and keep control output neutral.

## T03 Trigger Timing Consistency (TIM1 -> ADC)
- Goal: Confirm ADC trigger is synchronized to TIM1 update event.
- Steps:
  1. Capture TIM1-related reference and ADC activity proxy.
  2. Compare timing stability over time.
- Expected:
  - Stable periodic relationship with no large jitter.
- Fail signature:
  - Drifting phase or random jitter spikes.
- Rollback:
  - Reduce load and disable nonessential telemetry temporarily.

## T04 CRSF Input Decode and Timeout Protection
- Goal: Verify control input path and fail-safe behavior.
- Steps:
  1. Send valid CRSF frames.
  2. Observe control variables update.
  3. Stop CRSF transmission.
- Expected:
  - Valid frames change setpoints.
  - On link loss, safety logic forces safe output in timeout window.
- Fail signature:
  - No setpoint update, stale control, or no fail-safe on link loss.
- Rollback:
  - Apply emergency stop and keep output disabled until fixed.

## T05 VOFA Telemetry Integrity
- Goal: Ensure telemetry can be used for tuning and diagnosis.
- Steps:
  1. Connect VOFA to USART10.
  2. Observe key signals: sampled currents, target command, PWM command, CRSF state.
- Expected:
  - Stable frame stream at configured baud rate.
  - Values change with physical input and control action.
- Fail signature:
  - No stream, parse errors, or frozen values.
- Rollback:
  - Disable high-rate prints and verify UART config.

## T06 Safety and Limit Behavior
- Goal: Verify clamping and protective downgrade paths.
- Steps:
  1. Push command near limit.
  2. Inject abnormal input (out-of-range or disconnect).
  3. Observe clamp and output response.
- Expected:
  - Output remains within configured limits.
  - Fault path sets safe behavior without uncontrolled output.
- Fail signature:
  - Output exceeds clamp or unstable oscillation under fault.
- Rollback:
  - Immediate zero-output command and power stage isolation.

## 6. Data Recording Template
For each case record:
- Test ID
- Date/time
- Firmware hash or build id
- Result (PASS/FAIL)
- Observed values
- Failure signature
- Root cause hypothesis
- Action taken
- Retest result

## 7. Entry Criteria for PID Tuning
Start PID tuning only after all items pass:
- T00, T01, T02, T04, T05, T06

T03 may be accepted with minor bounded jitter if control remains stable and repeatable.

## 8. Open Risks
- Linker RWX segment warning still present in Debug build.
- ADC offset calibration and scaling may need board-specific tuning.
- Final motor-load test not covered by no-load bench verification.

## 9. Next Actions
1. Execute this plan top-down on hardware.
2. Fix first failing case before running new optimization changes.
3. After all pass, create a short acceptance report for regression baseline.
