# Lessons index — transport-truck

This index records project-scoped pitfalls, corrections and verified practices.
A lesson is not automatically project policy, a universal contract, or a blocking rule.

| Lesson | Source evidence | Applies when | Does not imply | Status |
|---|---|---|---|---|
| [stale-flash-after-pin-change](stale-flash-after-pin-change.md) | 2026-10-20 C30 不转排查：模式 7 对照、官方例程可用、R41/R42 实测 100 Ω、J1 未焊 | 换舵机/PWM 引脚、切换运行模式、外设不动作时 | 不证明 C30/PWM4 存在硬件故障；不适用于非 PWM 外设 | active |
| [stepper-driver-supply-and-pulse](stepper-driver-supply-and-pulse.md) | 2026-11-11 RMN28 排查：Vref 2.13 V 时 12 V 被拉到 6 V、空载 0.012 A、Vref 1.15 V 时 0.28 A；D12 有脉冲但两相电压不变 | 步进驱动器上电异常、电压被拉低、电机不走步时 | 不证明其他 STEP/DIR 驱动器有同样问题；新驱动器下整机运动未验证 | active |

## Promotion decisions

Promotion requires an explicit destination and authority:

| Lesson | Destination | Decision source | Verification | Review trigger |
|---|---|---|---|---|
| — | — | — | — | — |

Possible destinations include project documentation, an optional check, or a project-owned rule.
Unresolved observations remain lessons with evidence limits recorded.
