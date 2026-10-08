# 运输车主控基础工程

本工程从本地 RT1064 推箱子工程精简而来，保留四麦克纳姆轮底盘、IMU、编码器、PID、MPC 和串口通信基础。推箱子业务、PC 验证工具、视觉 Python 程序和旧菜单已移除；运输任务逻辑与新菜单待构建。

## 目录结构

```text
transport-truck/
├── project/
│   ├── code/
│   │   ├── config/       # 引脚映射、底盘参数及方向极性
│   │   ├── chassis_*     # 底盘驱动、控制、运动规划及区域判定
│   │   └── app_link.c/h  # 保留的串口协议与地图快照
│   ├── user/            # main.c 和中断入口 isr.c
│   ├── iar/             # IAR 工程
│   └── mdk/             # MDK 工程
├── libraries/           # 逐飞 SDK、外设驱动及许可证
└── docs/                # PINOUT 引脚资料
```

## 当前入口

`project/user/src/main.c` 已恢复为原本地工程的完整文件，当前默认单轮 PID 调试。PIT_CH0 每 5ms 更新姿态，PIT_CH1 每 20ms 执行底盘闭环。

当前版本是删减后的待重建快照。原 `main.c` 仍引用已删除的推箱子和菜单头文件，暂不能直接编译；入口修改将在后续任务中另行进行。

硬件引脚、极性和控制参数仍集中在 `project/code/config/pinMap.h` 与 `configChassis.h`。`app_link` 的既有地图快照保留给底盘软限位使用；新的运输业务和通信初始化后续按任务要求构建。

## 编译与文档

- IAR 工作空间：`project/iar/rt1064.eww`，配置 `nor_sdram_zf_dtcm`。
- MDK 工程：`project/mdk/rt1064.uvprojx`。
- [PROJECT.md](PROJECT.md)：硬件、编译和调试入口。
- [代码编写与变更规范.md](代码编写与变更规范.md)：编码与变更规范。
- [docs/PINOUT.md](docs/PINOUT.md)：引脚资料。
- [底盘运动控制角度环与位置环评审及最优方案.md](底盘运动控制角度环与位置环评审及最优方案.md)：保留的底盘控制评审记录。
