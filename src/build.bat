@echo off
rem Build pathfinder.exe with MSVC (static CRT, no runtime DLL dependencies).
setlocal
where cl >nul 2>nul
if not errorlevel 1 goto build
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR goto novs
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" goto novs
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul

:build
pushd "%~dp0"
cl /nologo /O2 /MT /EHsc /std:c++17 /W3 main.cpp /Fe:pathfinder.exe /link psapi.lib
set ERR=%errorlevel%
del /q main.obj 2>nul
popd
exit /b %ERR%

:novs
echo Visual Studio C++ tools not found. Install "Desktop development with C++" or use build.sh with g++.
exit /b 1
