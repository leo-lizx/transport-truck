@echo off
rem Stepper host tests: builds real stepper_profile.h and app_stepper_test.c, outputs to %TEMP%.
setlocal
set "REPO=%~dp0..\.."
for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
if not defined VSROOT (echo MSVC not found & exit /b 1)
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
set "OUT=%TEMP%\transport-stepper-host-tests"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"
set INC=/I"%REPO%\tests\stepper\stubs" /I"%REPO%\project\code"
cl /nologo /W4 /WX /utf-8 /TC %INC% "%REPO%\tests\stepper\test_stepper_profile.c" /Fe:test_stepper_profile.exe || (popd & exit /b 2)
test_stepper_profile.exe || (popd & exit /b 3)
cl /nologo /W4 /WX /utf-8 /TC %INC% "%REPO%\tests\stepper\test_app_stepper_test.c" "%REPO%\project\code\app_stepper_test.c" /Fe:test_app_stepper_test.exe || (popd & exit /b 4)
test_app_stepper_test.exe || (popd & exit /b 5)
popd
exit /b 0
