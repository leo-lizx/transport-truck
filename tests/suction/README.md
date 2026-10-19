# 吸盘应用主机行为测试

直接编译真实 `project/code/app_suction_test.c`，用桩替代 C15 GPIO 和
`suction_init()`／`suction_set()`，核对公开输出行为。测试不复制应用状态机，
不包含 MCU 驱动，也不证明 PWM 波形、电子开关、气路或吸附效果已通过。

覆盖上电按住、稳定松手解锁、20ms 去抖边界和抖动、两秒泄压与重复松手、
泄压中再按下、漏采和重复时间戳、计数回绕、重新初始化，以及只访问 C15。

## Windows／MSVC 运行

需要已安装 Visual Studio C++ 工具链，无需安装额外依赖。以下 PowerShell 命令
把所有编译产物放在临时目录，工程目录不产生 `.obj` 或 `.exe`：

```powershell
$repoRoot = 'E:\car\transport-truck'
$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw '未找到 MSVC C++ 工具链' }
$devScript = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
$buildDir = Join-Path ([IO.Path]::GetTempPath()) 'transport-suction-host-tests'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
$command = 'call "' + $devScript + '" >nul && cl /nologo /W4 /WX /utf-8 /TC /I"' + $repoRoot + '\tests\suction\stubs" /I"' + $repoRoot + '\project\code" "' + $repoRoot + '\tests\suction\test_app_suction_test.c" "' + $repoRoot + '\project\code\app_suction_test.c" /Fe:test_app_suction_test.exe && test_app_suction_test.exe'
Push-Location $buildDir
try {
    & cmd.exe /d /s /c $command
    if ($LASTEXITCODE -ne 0) { throw '编译或行为测试失败' }
} finally {
    Pop-Location
}
```

通过时输出各用例的 `PASS` 和总计，进程退出码为 0。测试没有自行规定连续
吸附的热保护时限；实物测试仍须遵守项目中的手动短按接线与测试说明。

2026-10-17 实际运行结果：MSVC `/W4 /WX /utf-8 /TC` 编译通过，
**19 个用例、205 条断言、0 失败**，退出码 0。产物保存在临时目录。
