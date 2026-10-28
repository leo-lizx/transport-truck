# 舵机应用主机行为测试

直接编译真实 `project/code/app_servo_test.c`（模式 5）和
`project/code/app_servo_sequence.c`（模式 6），用桩替代
`servo_init()` / `servo_set_angle()`，核对扫动与上电序列的边界、节拍和回绕。
测试不复制应用逻辑，也不证明 PWM 波形、舵机供电或实际转角。

覆盖：

- 模式 5：初始化只调用驱动初始化、首帧只对时、步进间隔边界、上/下限换向、
  合并节拍只走一步、重新初始化回到起始角、`uint32_t` 回绕。
- 模式 6：初始化停在 90°、1.5 s 后去 0°、完整
  90°→0°→180°→90° 周期、完成后不再动作、合并节拍只推进一步、
  回绕、重新初始化重新跑周期。

## Windows／MSVC 运行

需要已安装 Visual Studio C++ 工具链。以下命令把所有产物放在临时目录：

```powershell
$repoRoot = 'E:\car\transport-truck'
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw '未找到 MSVC C++ 工具链' }
$devScript = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
$buildDir = Join-Path ([IO.Path]::GetTempPath()) 'transport-servo-host-tests'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$command = 'call "' + $devScript + '" >nul && cl /nologo /W4 /WX /utf-8 /TC /I"' + $repoRoot + '\project\code" "' + $repoRoot + '\tests\servo\test_app_servo_test.c" "' + $repoRoot + '\project\code\app_servo_test.c" /Fe:test_app_servo_test.exe && test_app_servo_test.exe'
Push-Location $buildDir
try {
    & cmd.exe /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw '编译或行为测试失败' }
} finally {
    Pop-Location
}
```

模式 6 的序列测试用同一个命令，但要把测试源文件换成
`test_app_servo_sequence_test.c`、应用源文件换成
`project/code/app_servo_sequence.c`，exe 名可改为
`test_app_servo_sequence_test.exe`。

通过时输出各用例的 `PASS` 和总计，进程退出码为 0。实机测试仍须空载、
先确认供电、方向与机械限位。

## 模式 6 上电序列

默认 `MAIN_RUN_MODE_SERVO_SEQUENCE`（6），卸载机构后上电：

| 时间（上电算） | 动作 |
|---|---|
| 0 s | 停在 90° |
| +1.5 s | 发 0° |
| +3.0 s | 发 180° |
| +4.5 s | 发 90°，周期结束，不再动作 |

不需要按键；重新上电才会再跑一次。0° 与 180° 对应 0.5/2.5 ms，
在 FS5323M 的 500–2500 µs 额定量程内，但仍是两端极限位置，
首次必须空载确认没有撞限位。PWM 指令测试不证明真实角度、波形、
供电或机械安全，仍须烧录实测。
