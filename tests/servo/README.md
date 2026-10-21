# 舵机应用主机行为测试

直接编译真实 `project/code/app_servo_test.c`，用桩替代 `servo_init()` /
`servo_set_angle()`，核对往返扫动的边界、节拍与回绕。测试不复制应用逻辑，
也不证明 PWM 波形、舵机供电或实际转角。

覆盖：初始化只调用驱动初始化、首帧只对时、步进间隔边界、上/下限换向、
合并节拍只走一步、重新初始化回到起始角、`uint32_t` 回绕。

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

通过时输出各用例的 `PASS` 和总计，进程退出码为 0。实机测试仍须空载、
小范围、先确认供电与方向。
