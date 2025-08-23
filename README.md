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
- **Automated Build Scripts**: PowerShell scripts for easy deployment

## Quick Start (Arduino IDE)

1. **Install Arduino IDE**: Download from `https://www.arduino.cc/en/software`.
2. **Install RP2040 Support**:
   - File → Preferences → Additional Boards Manager URLs → add `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   - Tools → Board → Boards Manager… → search "rp2040" → install "Raspberry Pi Pico/RP2040 by Earle F. Philhower, III"
3. **Select Board and Port**:
   - Tools → Board: Raspberry Pi Pico W
   - Tools → USB Stack: Pico SDK
   - Tools → Port: your Pico’s COM port
4. **Install Libraries** (Sketch → Include Library → Manage Libraries…):
   - Adafruit SSD1306
   - Adafruit GFX Library
   - Adafruit BusIO
5. **Open the Sketch**: Open `FilamentExtruder.ino` at the project root
6. **Upload**: Click Upload. If needed, hold BOOTSEL while connecting USB to enter bootloader.
7. **Monitor**: Tools → Serial Monitor at 115200 baud.

## Hardware Requirements

### Main Components
- Raspberry Pi Pico W
- SSD1306 128x64 OLED Display (I2C)
- 2x I2C ADC modules (ADS1115, ADS1015, or similar)
- 2x Analog potentiometers (10kΩ recommended)
- 4x Relay modules (for heater control)
- 4x Heating elements (cartridge heaters, resistive heaters, etc.)
- 1x Thermistor (100kΩ NTC) with series resistor for temperature sensing
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

## Build & Deploy (Arduino CLI via PowerShell)

Use the provided PowerShell script to build and deploy with Arduino CLI (auto-installed locally if missing).

### One-time
- Windows PowerShell (7+ recommended)
- Internet access (downloads Arduino CLI/core/libs on first run)

### Commands
```powershell
# Build and wait for RPI-RP2 drive, then copy UF2 automatically
.\build_and_deploy_arduino.ps1

# Build only (no deploy)
.\build_and_deploy_arduino.ps1 -BuildOnly

# Upload via serial (if you prefer COM upload)
.\build_and_deploy_arduino.ps1 -Port COM7

# Customize board or output directory
.\build_and_deploy_arduino.ps1 -Fqbn 'rp2040:rp2040:rpipicow:usbstack=picosdk' -OutputDir .\build
```

### How it works
- Installs a portable `arduino-cli` into `.tools` if not found
- Adds RP2040 board manager URL and installs `rp2040:rp2040`
- Ensures libraries: Adafruit SSD1306, Adafruit GFX Library, Adafruit BusIO
- Compiles `FilamentExtruder.ino` (root-level)
- Deploys by either:
  - Copying the UF2 to the `RPI-RP2` mass storage device (hold BOOTSEL while plugging in USB)
  - Or uploading over the specified `-Port`

### Script options
- `-BuildOnly`: Build but skip deploy
- `-Port <COMx>`: Force serial upload
- `-TimeoutSeconds <n>`: Wait time for `RPI-RP2` drive (default 60)
- `-Fqbn <id>`: Fully Qualified Board Name (default Pico W with Pico SDK stack)
- `-OutputDir <path>`: UF2 output directory (default `.\\build`)
- `-CliPath <path>`: Use a specific `arduino-cli.exe`
- `-Sketch <path>`: Explicit sketch path (not required when `FilamentExtruder.ino` is in repo root)

### Configuration
I2C pins are configured in `FilamentExtruder.ino` using `Wire.setSDA(8)` and `Wire.setSCL(9)` before `Wire.begin()`. If your OLED uses `0x3D`, change `SCREEN_ADDRESS` in the sketch. WiFi is currently disabled; to add later, include `#include <WiFi.h>` and configure credentials in code.

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
Modify the pins in `FilamentExtruder.ino`:
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
This project is licensed under the MIT License. See `LICENSE` for details.

## Support
For issues or questions, check the serial output first for debugging information.
