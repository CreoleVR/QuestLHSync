@echo off
rem builds the SteamVR driver (driver\questlhsync\bin\win64\driver_questlhsync.dll) and the dashboard app
rem (QuestLHSync.exe next to it). SteamVR must be closed to replace a loaded DLL; "build.bat overlay" builds only
rem the dashboard app (close QuestLHSync.exe first). Needs Visual Studio 2022 (or its Build Tools) with C++.
setlocal
set VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VS=%%i
if not defined VS (echo Visual Studio with C++ not found & exit /b 1)
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cd /d "%~dp0"
set OUT=driver\questlhsync\bin\win64
set MH=third_party\minhook
set OVR=third_party\openvr
if not exist build\driver mkdir build\driver
if not exist build\overlay mkdir build\overlay
if not exist %OUT% mkdir %OUT%
if /i "%1"=="overlay" goto overlay
cl /nologo /LD /O2 /EHsc /std:c++17 /MT /W3 /DNOMINMAX /DWIN32_LEAN_AND_MEAN /D_CRT_SECURE_NO_WARNINGS ^
  /I%OVR%\headers /I%MH%\include /Fobuild\driver\ ^
  src\driver\driver_main.cpp src\driver\sync.cpp src\driver\net.cpp src\driver\gravity.cpp ^
  %MH%\src\buffer.c %MH%\src\hook.c %MH%\src\trampoline.c %MH%\src\hde\hde64.c ^
  /Fe:%OUT%\driver_questlhsync.dll /link /NOLOGO ws2_32.lib iphlpapi.lib shell32.lib ole32.lib setupapi.lib hid.lib || exit /b 1
del %OUT%\driver_questlhsync.exp %OUT%\driver_questlhsync.lib 2>nul
echo built %OUT%\driver_questlhsync.dll
:overlay
cl /nologo /utf-8 /O2 /EHsc /std:c++17 /MT /W3 /DNOMINMAX /DWIN32_LEAN_AND_MEAN /DUNICODE /D_UNICODE /D_CRT_SECURE_NO_WARNINGS ^
  /I%OVR%\headers /Fobuild\overlay\ src\overlay\overlay_main.cpp ^
  /Fe:%OUT%\QuestLHSync.exe /link /NOLOGO /SUBSYSTEM:WINDOWS %OVR%\lib\win64\openvr_api.lib gdi32.lib user32.lib shell32.lib d3d11.lib dxgi.lib || exit /b 1
copy /y %OVR%\bin\win64\openvr_api.dll %OUT%\ >nul
echo built %OUT%\QuestLHSync.exe
