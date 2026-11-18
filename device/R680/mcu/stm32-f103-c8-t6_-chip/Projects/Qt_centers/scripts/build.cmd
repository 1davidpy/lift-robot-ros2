@echo off
setlocal

set "PROJECT_ROOT=%~dp0.."
set "BUILD_ROOT=%PROJECT_ROOT%\build-release"
set "DIST_ROOT=%PROJECT_ROOT%"
set "QT_ROOT=C:\Qt\6.8.3\mingw_64"
set "MINGW_ROOT=C:\Qt\Tools\mingw1310_64"
set "CMAKE_EXE=C:\Users\xiaofang\AppData\Local\stm32cube\bundles\cmake\4.3.1+st.1\bin\cmake.exe"
set "NINJA_EXE=C:\Users\xiaofang\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe"
set "PATH=%QT_ROOT%\bin;%MINGW_ROOT%\bin;%PATH%"

"%CMAKE_EXE%" -S "%PROJECT_ROOT%" -B "%BUILD_ROOT%" -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="%QT_ROOT%" ^
  -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
  -DCMAKE_C_COMPILER="%MINGW_ROOT%\bin\gcc.exe" ^
  -DCMAKE_CXX_COMPILER="%MINGW_ROOT%\bin\g++.exe"
if errorlevel 1 exit /b %errorlevel%

"%CMAKE_EXE%" --build "%BUILD_ROOT%"
if errorlevel 1 exit /b %errorlevel%

"%CMAKE_EXE%" --build "%BUILD_ROOT%" --target test
if errorlevel 1 exit /b %errorlevel%

if not exist "%DIST_ROOT%" mkdir "%DIST_ROOT%"
copy /Y "%BUILD_ROOT%\bin\QtCenters.exe" "%DIST_ROOT%\QtCenters.exe" >nul
if errorlevel 1 (
  echo ERROR: QtCenters.exe is in use. Close the running upper-computer program and build again.
  exit /b 1
)
if not exist "%DIST_ROOT%\config" mkdir "%DIST_ROOT%\config"
copy /Y "%PROJECT_ROOT%\..\excavator_controlled_servo\config\machines.json" "%DIST_ROOT%\config\machines.json" >nul
if errorlevel 1 exit /b %errorlevel%
copy /Y "%PROJECT_ROOT%\..\excavator_controlled_servo\config\config2.json" "%DIST_ROOT%\config\config2.json" >nul
if errorlevel 1 exit /b %errorlevel%
"%QT_ROOT%\bin\windeployqt.exe" --release --no-translations --dir "%DIST_ROOT%" "%DIST_ROOT%\QtCenters.exe"
if errorlevel 1 exit /b %errorlevel%

echo Built and deployed: %DIST_ROOT%\QtCenters.exe
exit /b 0
