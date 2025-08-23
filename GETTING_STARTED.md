# Getting Started - Filament Extruder Controller

## What You Need
- Raspberry Pi Pico W
- SSD1306 OLED Display (128x64)
- 2x I2C ADC modules (like ADS1115)
- 2x 10kΩ analog potentiometers
- Breadboard and jumper wires

## Wiring (Simple)
```
Pico W Pin 11 (GPIO8) → SDA on all I2C devices
Pico W Pin 12 (GPIO9) → SCL on all I2C devices
Pico W 3.3V → VCC on all devices + potentiometer high pins
Pico W GND → GND on all devices + potentiometer low pins

Potentiometer wipers → ADC AIN0 inputs
```

## Software (Super Simple)
1. Download this project
2. Double-click `build_and_deploy.bat` OR run:
   ```
   PowerShell -ExecutionPolicy Bypass -File build_and_deploy.ps1
   ```
3. Follow the prompts
4. Connect your Pico W (hold BOOTSEL while plugging in USB)
5. Done!

## What It Does
- **Speed Control**: First potentiometer controls extruder speed (0-1000 units)
- **Temperature**: Second potentiometer simulates temperature reading (0-300°C)
- **Display**: Shows current speed and temperature on OLED
- **Serial**: Debug info at 115200 baud

## Troubleshooting
- **Script won't run**: Try `Set-ExecutionPolicy RemoteSigned -Scope CurrentUser` in PowerShell
- **No display**: Check wiring, try changing SCREEN_ADDRESS from 0x3D to 0x3C in code
- **Upload fails**: Make sure Pico W is in bootloader mode (hold BOOTSEL)

That's it! Your filament extruder controller should be working.
