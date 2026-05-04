# Workflow Checklist

## A. Inputs Complete
- [ ] 目标模块明确（ADC/PID/PWM/CRSF/OLED/VOFA）
- [ ] 时序约束明确（频率/周期/延迟）
- [ ] 验收标准明确（构建/观测/保护）

## B. Evidence Collected
- [ ] 已查本仓库 `docs` 相关文件
- [ ] 已查 `F:\stm32_ws\projects\dm-mc02` 的对应例程
- [ ] 必要时已补充网络检索并记录依据

## C. Architecture Decisions
- [ ] 主循环周期任务表已定义
- [ ] ISR与主循环职责边界清晰
- [ ] 模块接口和数据流清晰（ADC->PID->PWM，CRSF->设定值）

## D. Implementation Safety
- [ ] 保留或兼容现有命名风格
- [ ] 有限幅、超时、越界处理
- [ ] 失联/异常时具备安全降级路径

## E. Validation
- [ ] 已执行 `CMake: Configure+Build Debug`
- [ ] 构建结果已记录
- [ ] 至少一个观测通道可用（VOFA/OLED）

## F. Delivery Quality
- [ ] 输出包含：变更摘要、关键文件、调度与时序、验证结果、风险与后续
- [ ] 每个关键结论可追溯到代码或资料来源
