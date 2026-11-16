@echo off
rem Starts Sifted on Windows. Its fusion service runs inside WSL (set up by install.cmd);
rem the operator view opens in your browser. Keep this window open while you use Sifted.
setlocal EnableExtensions
title Sifted

rem This folder, without the trailing backslash (wsl.exe --cd is picky about it).
set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "DISTRO="
if exist "%ROOT%\build\wsl-distro.txt" set /p DISTRO=<"%ROOT%\build\wsl-distro.txt"

where wsl.exe >nul 2>nul
if errorlevel 1 goto nowsl

echo Starting Sifted from %ROOT% ...
if not defined DISTRO goto default
wsl.exe -d "%DISTRO%" --cd "%ROOT%" -- bash ./sifted %*
set "CODE=%errorlevel%"
rem -1 is a WSL error (e.g. that distribution no longer exists): try the default one.
if not "%CODE%"=="-1" goto after
echo Retrying with your default WSL distribution...
:default
wsl.exe --cd "%ROOT%" -- bash ./sifted %*
set "CODE=%errorlevel%"
:after
if not "%CODE%"=="0" goto failed
if /i "%~1"=="--check" if not defined SIFTED_NO_PAUSE pause
exit /b 0

:failed
echo.
echo Sifted could not start or stopped with an error (exit code %CODE%).
echo The lines above say why. Common fixes:
echo   - "not built" or "No such file": double-click install.cmd again.
echo   - a WSL error: restart Windows, open Ubuntu from the Start menu once, then try again.
echo   - to check the installation, run in this folder:  sifted.cmd --check
echo See Troubleshooting in README.md for more.
echo.
if not defined SIFTED_NO_PAUSE pause
exit /b %CODE%

:nowsl
echo Sifted needs the Windows Subsystem for Linux (WSL). Double-click install.cmd first.
if not defined SIFTED_NO_PAUSE pause
exit /b 1
