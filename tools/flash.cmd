@echo off
rem ???? + ??(?? PowerShell ??????)
rem   tools\flash.cmd                 ?? J-Link
rem   tools\flash.cmd -Interface stlink
rem   tools\flash.cmd -BuildOnly      ??????
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
