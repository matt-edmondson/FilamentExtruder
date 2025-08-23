@echo off
setlocal enabledelayedexpansion

REM Simple batch file alternative for PowerShell-averse users
echo ==========================================
echo   Pico W Filament Extruder Controller
echo      Simple Build ^& Deploy Script
echo ==========================================
echo.

REM Check if we're in the right directory
if not exist "platformio.ini" (
    echo [ERROR] platformio.ini not found in current directory
    echo Please run this script from the project root directory
    pause
    exit /b 1
)

echo [OK] Found platformio.ini - we're in the right directory
echo.

REM Check if PlatformIO is installed
pio --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [WARNING] PlatformIO not found
    echo.
    echo This script requires PlatformIO to be installed.
    echo Please run the PowerShell script instead for automatic installation:
    echo.
    echo   PowerShell -ExecutionPolicy Bypass -File build_and_deploy.ps1
    echo.
    echo Or install PlatformIO manually:
    echo   1. Install Python 3.7+ from https://python.org
    echo   2. Run: python -m pip install platformio
    echo.
    pause
    exit /b 1
)

echo [OK] PlatformIO found
echo.

REM Build the project
echo [INFO] Building project...
pio run
if %errorlevel% neq 0 (
    echo [ERROR] Build failed
    pause
    exit /b 1
)

echo [OK] Build completed successfully
echo.

REM Ask user if they want to upload
set /p upload="Upload to Pico W? (y/n): "
if /i "%upload%"=="y" goto :upload
if /i "%upload%"=="yes" goto :upload
goto :end

:upload
echo [INFO] Uploading to Pico W...
echo Make sure your Pico W is connected and in bootloader mode
echo (Hold BOOTSEL button while connecting USB)
echo.
pause

pio run --target upload
if %errorlevel% neq 0 (
    echo [ERROR] Upload failed
    echo.
    echo Make sure your Raspberry Pi Pico W is:
    echo   1. Connected via USB cable
    echo   2. In bootloader mode (hold BOOTSEL while connecting)
    echo   3. Recognized as a drive or COM port
    echo.
    pause
    exit /b 1
)

echo [OK] Upload completed successfully
echo.
echo ==========================================
echo    BUILD ^& DEPLOY COMPLETED SUCCESSFULLY
echo ==========================================
echo.
echo Your Filament Extruder Controller is now running!
echo Connect to serial monitor at 115200 baud for debugging
echo.

:end
pause
