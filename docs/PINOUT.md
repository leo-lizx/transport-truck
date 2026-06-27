# RT1064 引脚冲突矩阵 / PINOUT Matrix

> 引脚单一真相源: `project/code/config/pinMap.h`
> 本矩阵由 pinMap.h + SDK 驱动头自动提取。修改 pinMap.h 后须同步更新本文。
> 矩阵用途: 新增外设时快速检查引脚冲突。

## 占用引脚总表

| Pin | 功能 | 外设块 | 所属模块 | ALT 模式 | 备注 |
|-----|------|--------|----------|----------|------|
| | | | | | |
| **GPIO C 口** | | | | | |
| C0 | ENC1_CH1 | QTIMER1_ENCODER1 | `chassis_encoder` | ALT1 | 编码器 LB A 相 |
| C1 | ENC1_CH2 | QTIMER1_ENCODER1 | `chassis_encoder` | ALT1 | 编码器 LB B 相 |
| C2 | ENC2_CH1 | QTIMER1_ENCODER2 | `chassis_encoder` | ALT1 | 编码器 RB A 相 |
| C3 | ENC3_CH1 | QTIMER2_ENCODER1 | `chassis_encoder` | ALT1 | 编码器 RF A 相 |
| C4 | ENC3_CH2 | QTIMER2_ENCODER1 | `chassis_encoder` | ALT1 | 编码器 RF B 相 |
| C5 | ENC4_CH1 | QTIMER2_ENCODER2 | `chassis_encoder` | ALT1 | 编码器 LF A 相 |
| C6 | PWM_CHA | PWM2_MODULE0 | `chassis_motor` | ALT2 | 电机 LB PWM |
| C7 | GPIO | GPIO | `chassis_motor` | — | 电机 LB DIR (推挽输出) |
| C8 | PWM_CHA | PWM2_MODULE1 | `chassis_motor` | ALT2 | 电机 RB PWM |
| C9 | GPIO | GPIO | `chassis_motor` | — | 电机 RB DIR (推挽输出) |
| C10 | GPIO | GPIO | `chassis_motor` | — | 电机 LF DIR (推挽输出) |
| C11 | PWM_CHB | PWM2_MODULE2 | `chassis_motor` | ALT2 | 电机 LF PWM |
| C12 | GPIO | GPIO | `chassis_menu` | — | 按键 K4 |
| C13 | GPIO | GPIO | `chassis_menu` | — | 按键 K3 |
| C14 | GPIO | GPIO | `chassis_menu` | — | 按键 K2 |
| C15 | GPIO | GPIO | `chassis_menu` | — | 按键 K1 |
| C16 | UART_TX | LPUART4 | `app_link` | ALT2 | OpenART1 TX (115200bps) |
| C17 | UART_RX | LPUART4 | `app_link` | ALT2 | OpenART1 RX (115200bps) |
| C18 | GPIO | GPIO | `chassis_menu` | — | IPS200 背光 BL |
| C19 | GPIO | GPIO | `chassis_menu` | — | IPS200 DC (数据/命令) |
| C20 | GPIO | GPIO | `chassis_imu` | — | IMU660RB CS (片选) |
| C21 | MISO | SPI4 | `chassis_imu` | ALT1 | IMU660RB SDO |
| C22 | MOSI | SPI4 | `chassis_imu` | ALT1 | IMU660RB SDI/SDA |
| C23 | SCK | SPI4 | `chassis_imu` | ALT1 | IMU660RB SPC/SCL |
| C24 | ENC2_CH2 | QTIMER1_ENCODER2 | `chassis_encoder` | ALT1 | 编码器 RB B 相 |
| C25 | ENC4_CH2 | QTIMER2_ENCODER2 | `chassis_encoder` | ALT1 | 编码器 LF B 相 |
| | | | | | |
| **GPIO B 口** | | | | | |
| B0 | SCK | SPI3 | `chassis_menu` | ALT1 | IPS200 SCL |
| B1 | MOSI | SPI3 | `chassis_menu` | ALT1 | IPS200 SDA |
| B2 | GPIO | GPIO | `chassis_menu` | — | IPS200 RST (复位) |
| B3 | GPIO | GPIO | `chassis_menu` | — | IPS200 CS (片选) |
| B12 | UART_TX | LPUART1 | `app_link` | ALT2 | OpenART2 TX (115200bps) |
| B13 | UART_RX | LPUART1 | `app_link` | ALT2 | OpenART2 RX (115200bps) |
| | | | | | |
| **GPIO D 口** | | | | | |
| D2 | GPIO | GPIO | `chassis_motor` | — | 电机 RF DIR (推挽输出) |
| D3 | PWM_CHB | PWM2_MODULE3 | `chassis_motor` | ALT2 | 电机 RF PWM |

## 外设 → 引脚汇总

| 外设 | 占用引脚 | 计数 |
|------|----------|:--:|
| PWM2 (电机) | C6, C8, C11, D3 | 4 |
| QTIMER1 (ENC1/2) | C0, C1, C2, C24 | 4 |
| QTIMER2 (ENC3/4) | C3, C4, C5, C25 | 4 |
| SPI3 (IPS200) | B0, B1, B2, B3, C18, C19 | 6 |
| SPI4 (IMU660RB) | C20, C21, C22, C23 | 4 |
| LPUART4 (OpenART1) | C16, C17 | 2 |
| LPUART1 (OpenART2) | B12, B13 | 2 |
| GPIO (电机 DIR) | C7, C9, C10, D2 | 4 |
| GPIO (按键) | C12, C13, C14, C15 | 4 |
| | | **34** |

## 未占用/空闲引脚 (部分)

> 以下为 RT1064 常用空闲引脚，新增外设时可优先考虑。

| Pin | 备注 |
|-----|------|
| A0~A29 | 大块空闲，适合并行总线或大量 GPIO |
| B4~B11 | B 口中间段空闲 (B6~B11 位于 OpenART2 与电机之间) |
| C26~C31 | C 口高端空闲 |
| D0~D1 | 注意: 部分底板用 D0/D1 做 UART Debug |
| D4~D15 | D 口中段空闲 |

## SDK 管理的外设 (引脚定义在 `libraries/` 中)

| 外设 | 实例 | 备注 |
|------|------|------|
| 摄像头 UART | LPUART5 | `camera_uart_handler()`，引脚依底板而定 |
| 无线模块 UART | LPUART8 | `wireless_module_uart_handler()`，引脚依底板而定 |
| Debug UART | UART8 | 原在 LPUART1 (B12/B13)，已切换 |
| SD 卡 | SDHC | 使用 USDHC 引脚组 |
| 调试器 SWD | SWCLK/SWDIO | 固定引脚，不可占用 |

## 引脚冲突检查清单 (新增外设时)

- [ ] 新引脚不在本表已占用列表中
- [ ] 新增 ALT 模式不与已有 ALT 模式冲突 (同一引脚的不同 ALT 复用)
- [ ] 如使用 SDK 驱动，确认其默认引脚不与本表冲突
- [ ] 修改后同步更新 `pinMap.h` + 本矩阵
