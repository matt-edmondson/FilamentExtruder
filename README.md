---
status: draft
---

# Raspberry Pi Pico W Filament Extruder Controller

An Arduino-based filament extruder controller for the Raspberry Pi Pico W featuring custom I2C configuration, SSD1306 display, analog control inputs, and 4-channel relay heater control. Perfect for 3D printing applications requiring precise temperature control and speed monitoring for filament extrusion.

## Features

- **Custom I2C Configuration**: SDA on GPIO8, SCL on GPIO9
- **SSD1306 OLED Display**: 128x64 pixel monochrome display with real-time status
- **Dual Control Inputs**: Speed control and temperature setpoint via I2C ADC modules
- **4-Channel Heater Control**: GPIO18-21 controlling heating element relays
- **Smart Temperature Control**: PWM-based heater pulsing with safety limits
- **Real-time Monitoring**: Live temperature, speed, and heater status display
- **Safety Features**: Emergency shutdown, maximum temperature limits, heater timeouts
- **Serial Debugging**: Comprehensive debug output with heater status
- **Automated Build Scripts**: PowerShell and Batch scripts for easy deployment

## Quick Start

1. **Connect Hardware**: Wire up your Pico W, SSD1306 display, and ADC modules according to the wiring diagram
2. **Run Build Script**: Execute `PowerShell -ExecutionPolicy Bypass -File build_and_deploy.ps1`
3. **Upload to Pico**: Hold BOOTSEL button while connecting USB, then let the script deploy
4. **Monitor Operation**: Connect to serial monitor at 115200 baud or watch the OLED display

The PowerShell script will automatically install Python and PlatformIO if needed!

## Hardware Requirements

### Main Components
- Raspberry Pi Pico W
- SSD1306 128x64 OLED Display (I2C)
- 2x I2C ADC modules (ADS1115, ADS1015, or similar)
- 2x Analog potentiometers (10kΩ recommended)
- 4x Relay modules (for heater control)
- 4x Heating elements (cartridge heaters, resistive heaters, etc.)
- 1x Thermistor (10kΩ NTC) with series resistor for temperature sensing
- Breadboard and jumper wires

### I2C Addresses
- **SSD1306 Display**: 0x3C (default)
- **ADC Module 1**: 0x35 (reading Speed Control Potentiometer)
- **ADC Module 2**: 0x36 (reading Temperature Setpoint Potentiometer)

### GPIO Pin Assignments
- **GPIO8**: I2C SDA
- **GPIO9**: I2C SCL
- **GPIO18**: Heater Relay 1 Control
- **GPIO19**: Heater Relay 2 Control
- **GPIO20**: Heater Relay 3 Control
- **GPIO21**: Heater Relay 4 Control
- **A0**: Thermistor input (if using direct analog reading)

## Wiring Diagram

```
Raspberry Pi Pico W Connections:
┌─────────────────┬──────────────┐
│ Pico W Pin      │ Connection   │
├─────────────────┼──────────────┤
│ GPIO8 (Pin 11)  │ I2C SDA      │
│ GPIO9 (Pin 12)  │ I2C SCL      │
│ 3.3V (Pin 36)   │ VCC (All)    │
│ GND (Pin 38)    │ GND (All)    │
└─────────────────┴──────────────┘

I2C Bus Connections:
┌─────────────────┬─────────┬─────────┬─────────┐
│ Signal          │ Display │ ADC1    │ ADC2    │
├─────────────────┼─────────┼─────────┼─────────┤
│ SDA (GPIO8)     │ SDA     │ SDA     │ SDA     │
│ SCL (GPIO9)     │ SCL     │ SCL     │ SCL     │
│ VCC (3.3V)      │ VCC     │ VCC     │ VCC     │
│ GND             │ GND     │ GND     │ GND     │
└─────────────────┴─────────┴─────────┴─────────┘

Analog Potentiometer Connections:
┌─────────────────┬─────────┬─────────┐
│ Potentiometer   │ ADC1    │ ADC2    │
├─────────────────┼─────────┼─────────┤
│ POT1 Wiper      │ AIN0    │ -       │
│ POT1 +3.3V      │ -       │ -       │
│ POT1 GND        │ -       │ -       │
│ POT2 Wiper      │ -       │ AIN0    │
│ POT2 +3.3V      │ -       │ -       │
│ POT2 GND        │ -       │ -       │
└─────────────────┴─────────┴─────────┘
```

## Software Setup

### Automated Build & Deploy (Recommended)

#### Option 1: PowerShell Script (Full automation)
Run the comprehensive PowerShell script that handles everything:
```powershell
PowerShell -ExecutionPolicy Bypass -File build_and_deploy.ps1
```

**Features:**
- Automatically installs Python if missing
- Automatically installs PlatformIO if missing
- Builds the project
- Deploys to connected Pico W
- Comprehensive error handling and feedback

**Script Options:**
```powershell
# Build only (no deployment)
.\build_and_deploy.ps1 -BuildOnly

# Skip installation checks (if you know PlatformIO is installed)
.\build_and_deploy.ps1 -SkipInstall

# Verbose output for debugging
.\build_and_deploy.ps1 -Verbose
```

#### Option 2: Test Your Setup First
Before building, you can test if everything is properly installed:
```powershell
.\test_setup.ps1
```

#### Option 2b: Fix PlatformIO Issues
If you encounter build errors (like MissingPackageManifestError):
```powershell
# Standard repair (try this first)
.\repair_platformio.ps1

# Nuclear repair (if standard repair fails)
.\nuclear_repair.ps1
```

#### Option 3: Batch Script (Simple)
For users who prefer batch files:
```cmd
build_and_deploy.bat
```

**Note:** This requires PlatformIO to be pre-installed.

### Manual Installation
If you prefer manual setup:

#### Prerequisites
- Python 3.7+ with pip
- PlatformIO Core (`python -m pip install platformio`)
- Arduino framework for Raspberry Pi Pico

#### Manual Build & Deploy
1. Clone or download this project
2. Open terminal in project directory
3. Build: `pio run`
4. Deploy: `pio run --target upload`

### Configuration
The project uses custom I2C pins defined in `platformio.ini`:
```ini
build_flags = 
    -DPIN_WIRE_SDA=8
    -DPIN_WIRE_SCL=9
    -DPICO_W
```

**WiFi Configuration (Optional):**
If you want to add WiFi functionality later, update the WiFi credentials in `platformio.ini`:
```ini
-DWIFI_SSID=\"YourNetworkName\"
-DWIFI_PASSWORD=\"YourPassword\"
```

## Usage

### Display Interface
The OLED display shows:
- **Title**: "Filament Extruder"
- **Speed Control**: Current target speed in RPM
- **Temperature**: Target and current temperature in °C
- **Temperature Progress Bar**: Visual heating progress
- **Heater Status**: 4 indicators showing individual heater on/off state (█ = ON, □ = OFF)
- **Power Level**: Current heater power percentage (0-100%)
- **Safety Status**: "STANDBY", "! HOT !" warnings

### Serial Monitor
Connect at 115200 baud to see:
- Initialization messages
- I2C device scan results
- Real-time speed, temperature, and heater status
- Heater power calculations and safety warnings
- Emergency shutdown notifications

### Control System Details

#### Temperature Control
- **Target Range**: 50-350°C (configurable)
- **Safety Limit**: 350°C maximum (emergency shutdown)
- **Control Method**: Proportional control with PWM heater pulsing
- **Update Rate**: ~20Hz (50ms delay)
- **Pulse Interval**: 1000ms cycles for heater on/off control
- **Safety Timeout**: 5 seconds maximum continuous heater on time

#### Heater Control
- **4 Independent Channels**: GPIO18-21 control separate relay outputs
- **Synchronized Operation**: All heaters follow same power control signal
- **Power Range**: 0-100% via pulse width modulation
- **Dead Zone**: Reduced power when within ±2°C of target

## Code Structure

### Main Functions
- `initializeI2C()`: Configure custom I2C pins
- `initializeDisplay()`: Setup SSD1306 display
- `initializeHeaters()`: Configure GPIO18-21 for heater relay control
- `readAnalogPot(address)`: Read 16-bit ADC value from potentiometer
- `readTemperature()`: Read thermistor and calculate temperature
- `updateTemperatureControl()`: Calculate heater power and safety checks
- `updateHeaterControl()`: Manage PWM heater pulsing
- `setHeaterState(index, state)`: Control individual heater relays
- `calculateHeaterPower(target, current)`: Proportional control algorithm
- `emergencyShutdown()`: Safety shutdown for over-temperature
- `updateDisplay()`: Refresh display with all current values
- `scanI2CDevices()`: Scan and list I2C devices

### Configuration Constants
```cpp
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define POT1_ADDRESS 0x35
#define POT2_ADDRESS 0x36
#define ADC_READ_COMMAND 0x05
#define ADC_MAX_VALUE 1023

// Heater control pins
#define HEATER_1_PIN 18
#define HEATER_2_PIN 19
#define HEATER_3_PIN 20
#define HEATER_4_PIN 21

// Temperature control
#define TARGET_TEMP_DEFAULT 220
#define TEMP_TOLERANCE 2
#define SAFETY_MAX_TEMP 350
#define HEATER_PULSE_INTERVAL 1000
#define MAX_HEATER_ON_TIME 5000
```

## Troubleshooting

### Build Script Issues
1. **PowerShell Execution Policy Error**: Run `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`
2. **Python Installation Fails**: Manually install from [python.org](https://python.org) and retry
3. **PlatformIO Not Found After Install**: Close and reopen terminal/PowerShell to refresh PATH
4. **Upload Fails**: Ensure Pico W is in bootloader mode (hold BOOTSEL while connecting USB)

**For detailed troubleshooting steps, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)**

### Hardware Issues
1. **No Display Output**: Check I2C address (try 0x3C if 0x3D fails)
2. **ADC Not Responding**: Verify I2C addresses (0x35, 0x36) and connections
3. **Potentiometer Readings Erratic**: Check analog connections and power supply
4. **I2C Errors**: Ensure proper pull-up resistors (usually built-in)

### I2C Device Scanning
The code includes an I2C scanner that runs during setup. Check serial output for detected devices.

### Debugging
Enable serial debugging by connecting to the Pico W at 115200 baud. All major operations are logged.

## Customization

### Changing I2C Pins
Modify the pins in both `platformio.ini` and `main.cpp`:
```cpp
Wire.setSDA(your_sda_pin);
Wire.setSCL(your_scl_pin);
```

### Adding More Potentiometers
1. Define new ADC addresses (0x37, 0x38, etc.)
2. Add read calls in main loop
3. Update display function for additional values

### Display Modifications
The Adafruit GFX library provides extensive drawing functions for customizing the display output.

## Libraries Used
- **Adafruit SSD1306**: OLED display control
- **Adafruit GFX**: Graphics library
- **Adafruit BusIO**: I2C communication
- **Wire**: Arduino I2C library

## License
This project is open source. Feel free to modify and distribute.

## Support
For issues or questions, check the serial output first for debugging information.
