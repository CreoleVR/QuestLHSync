@echo off
rem builds out\qlhs_replay.exe (offline check of the driver's solver on recorded logs) and out\qlhs_nettest.exe
rem (the driver's network link + solver against src\tools\fake_lhsyncd.py)
setlocal
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VS=%%i
if not defined VS (echo Visual Studio with C++ not found & exit /b 1)
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cd /d "%~dp0"
if not exist out mkdir out
if not exist build\replay mkdir build\replay
cl /nologo /O2 /EHsc /std:c++17 /MT /W3 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /Fobuild\replay\ ^
  src\tools\replay.cpp src\driver\sync.cpp /Fe:out\qlhs_replay.exe /link /NOLOGO || exit /b 1
echo built out\qlhs_replay.exe
if not exist build\nettest mkdir build\nettest
cl /nologo /O2 /EHsc /std:c++17 /MT /W3 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /DWIN32_LEAN_AND_MEAN /utf-8 /Fobuild\nettest\ ^
  src\tools\nettest.cpp src\driver\net.cpp src\driver\sync.cpp ws2_32.lib iphlpapi.lib /Fe:out\qlhs_nettest.exe /link /NOLOGO || exit /b 1
echo built out\qlhs_nettest.exe
