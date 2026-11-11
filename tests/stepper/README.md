# 升降步进主机行为测试

直接编译真实的 `project/code/stepper_profile.h`（梯形速度规划）和
`project/code/app_stepper_test.c`（模式 9 状态机），用桩替代按键 GPIO 和
`drv_stepper` 驱动接口。测试不覆盖 GPT1 定时器、脉冲波形和实际电机运动。

覆盖：

- 速度规划：长行程梯形（总步数、速度上下限、每步 v² 变化不超过 2a、总用时）、
  短行程三角形、单步、零步、无加速度恒速、上限低于起停速度时钳位。
- 模式 9：上电等待 1 s、快找开关 → 退出 → 慢碰定零 → 停到就绪位、
  快/慢两段找不到开关进入故障、K1/K2 点动与软件行程钳位、运动中忽略点动、
  K4 任何阶段急停、K3 从停机/就绪重新回原点、上电按住的键须先松开、
  5 ms 毛刺不算按键、`uint32_t` 回绕。

## 运行（Windows／MSVC）

```powershell
cmd /d /c tests\stepper\run_host_tests.bat
```

产物放在 `%TEMP%\transport-stepper-host-tests`。通过时两个程序都输出
`0 failures`，批处理退出码为 0。
