# 嵌入式安全规则

> 本文档是 `hf-embedded-safety` 的静态规则版本，供 `.claude/rules/` 自动注入。
> 适用: RT1064 Cortex-M7 裸机 C (IAR EWARM 9.2)。

## ISR 安全

- ISR 内禁止调用阻塞 API（`delay_ms`、`while(!flag)` 忙等、信号量等待）。
- ISR 修改的全局变量必须用 `volatile` 声明；主循环读取时用 Seq-Lock 或临界区保护。
- ISR 不分配堆内存（`malloc`/`new`），不调用不可重入库函数（`printf`、`sprintf` 等）。
- 中断优先级分组必须显式配置，NVIC 优先级不要随意设为最高。

## Volatile 纪律

- 被多个执行上下文（ISR + 主循环、多任务）读写的变量必须 `volatile`。
- `volatile` 不替代原子操作：多字节变量跨上下文访问须加临界区（`__disable_irq()`/`__enable_irq()`）或内存屏障（`__DMB()`）。
- 编译器优化（`-Oh` 以上）下，非 volatile 的共享变量可能被优化掉读/写——现象是 "debug 正常，release 异常"。

## 数值安全

- 禁止有符号/无符号混算：`uint32` 与 `int32` 运算前显式转换。
- 除法/取模前检查除数非零；浮点运算检查 `isnan()`/`isinf()`。
- 数组索引用 `uint8_t`/`uint32_t`，禁止负数索引。
- 循环计数器上溢/下溢——倒计数 `for (i = N; i >= 0; i--)` 在 `i` 为无符号类型时死循环。
- 定点数缩放统一在注释中标注 Q 格式（如 `Q16.16`）。

## 外设安全

- 外设初始化后必须验证关键寄存器回读值（或至少检查状态标志）。
- PWM 占空比写入前限幅到硬件允许范围（本项目: `[0, CHASSIS_MOTOR_PWM_MAX]`）。
- SPI/I2C/UART 超时处理：不假定外设总是响应，超时后须复位状态机。
- 编码器计数器每周期清零后需检查溢出（本项目用 `int16` pulse delta，高速时可能翻转）。

## 极性 SSOT

- 所有方向系数集中在 `config/configChassis.h` §极性段。不在各模块散落符号修正。
- PID Kp 始终为正；符号由 `*_OUTPUT_DIR` / `ENCODER_*_DIR` / `IMU_YAW_SIGN` 统一处理。
- 修改电机线序/编码器接线/IMU 安装方向后，只改 §极性段，不改 PID 参数或控制逻辑。

## 内存约束

- SRAM 预算 ≤ 512KB (DTCM + OCRAM)
- 新增 BSS 数组上限 10KB
- 大数组优先放 DTCM（零等待），其次 OCRAM
- 禁止递归（栈溢出风险），栈使用通过 IAR Stack Usage 分析确认
