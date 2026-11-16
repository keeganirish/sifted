@echo off
rem Sifted installer for Windows: double-click this file.
rem It sets up WSL (if needed), builds Sifted inside it, adds a Sifted shortcut and starts Sifted.
title Sifted installer
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\windows\install.ps1" %*
set "ERR=%errorlevel%"
rem Success: the installer has started Sifted and closes by itself.
if "%ERR%"=="0" exit /b 0
rem Otherwise keep the window open so the message can be read.
echo.
pause
exit /b %ERR%
