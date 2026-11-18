@echo off
setlocal
set "PROJECT_ROOT=%~dp0.."
set "APPLICATION=%PROJECT_ROOT%\QtCenters.exe"

if not exist "%APPLICATION%" (
  call "%~dp0build.cmd"
  if errorlevel 1 exit /b %errorlevel%
)

start "QtCenters" /D "%PROJECT_ROOT%" "%APPLICATION%"

