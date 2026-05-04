---
name: h723-singlechip-baremetal-migration
description: '将H723+C8T6+RTOS分工方案迁移为H723单芯片裸机方案。适用于ADC电流采样、PID电流环、TIM PWM、电机驱动、CRSF串口接收、VOFA调参、OLED分模块调试等重构任务，包含资料检索、实现、构建验证与验收输出。'
argument-hint: '目标模块 + 时序约束 + 验收标准（例如：CRSF+PID+PWM，2ms刷新，构建通过并可在OLED观测）'
user-invocable: true
disable-model-invocation: false
---

# H723 Single-Chip Bare-Metal Migration

## When to Use
- 需要把“C8T6负责采样/反馈 + H723负责部分控制”的旧分工改为“H723独立完成全链路”。
- 需要在无RTOS前提下，建立裸机调度链路：ADC -> PID -> PWM。
- 需要接入或重构 CRSF 串口接收、VOFA 参数调试、OLED 逐模块调试显示。
- 需要先查本地例程与协议资料，再落地代码并验证构建。

## Input Contract
用户输入至少包含：
- 目标模块：例如 `ADC/PID/PWM/CRSF/OLED/VOFA` 的一个或多个。
- 时序约束：例如 `1kHz采样`、`20kHz电流环`、`2ms控制刷新`。
- 验收标准：例如 `CMake Debug构建通过`、`OLED可观测`、`超时保护可触发`。

若输入缺失，先提出最多 3 个关键澄清问题，再继续。

## Procedure
1. 识别现状与边界
- 确认仓库当前模块入口、初始化顺序、主循环与中断分工。
- 标记旧方案中依赖 C8T6/CAN/RTOS 的路径，识别需要替换或废弃的点。

2. 资料检索与证据收集（先本地后网络）
- 先查本仓库资料：`docs` 下协议与历史实现。
- 再查外部本地例程：`F:\stm32_ws\projects\dm-mc02`（尤其 `例程/CtrBoard-H7_*`）。
- 若本地证据不足，再进行网络检索：
  - STM32H723VGT6 外设初始化与采样/PWM实践。
  - CRSF 协议帧格式、校验与开源解析库。
- 对每个关键设计点记录“来源+结论”，避免无依据改动。

3. 设计裸机调度框架
- 定义主循环与周期任务表（例如 1ms/2ms/10ms）。
- 定义中断最小化原则：ISR 仅采样/搬运/置标志，计算放在主循环。
- 给出模块接口：
  - `adc_sample_current()` -> `pid_update_current_loop()` -> `pwm_apply_duty()`
  - `crsf_decode_frame()` -> 控制量更新 -> 目标电流/速度设定

4. 按模块实现（最小改动优先）
- 优先复用现有驱动与命名风格，必要时新增轻量接口层。
- 补充故障路径：串口超时、输入越界、PWM限幅、失控保护。
- OLED用于“逐模块自检”：每次只展示一个模块的关键健康指标。

5. 分支决策逻辑
- 如果已有代码可稳定复用：局部重构，不做大范围重写。
- 如果现有路径强耦合RTOS：先抽象接口，再替换调度。
- 如果CRSF协议实现不完整：先落地最小可用解析（通道+超时），再扩展高级特性。
- 如果构建失败：先修首个编译错误，再逐步清理后续错误，不并行猜改。

6. 验证与完成检查
- 执行构建任务：`CMake: Configure+Build Debug`。
- 检查链路时序是否满足输入约束。
- 检查观测路径：VOFA 或 OLED 至少一个可验证关键变量。
- 按固定格式输出结果。

## Output Format
必须按以下结构回复：

## 变更摘要
- 3到6条，说明实现结果

## 关键文件
- 文件路径 + 改动作用

## 调度与时序
- 模块频率、输入输出、超时/降级策略

## 验证结果
- 构建是否通过
- 若失败：首个错误 + 下一步建议

## 风险与后续
- 最多3条，聚焦稳定性/可维护性

## Quality Gates
- 不引入RTOS依赖。
- 核心控制链路闭环成立：ADC -> PID -> PWM。
- CRSF输入失联时具备保护动作。
- 至少一个可视化观测通道（VOFA或OLED）可用于调参/诊断。
- 输出中给出明确可复验信息。

详细检查项见 [workflow-checklist](./references/workflow-checklist.md)。
