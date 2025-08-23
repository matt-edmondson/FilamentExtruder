# Project Structure

## File Organization

```
FilamentExtruder/
├── src/
│   └── main.cpp                    # Main Arduino sketch
├── build_and_deploy.ps1           # PowerShell build & deploy script (recommended)
├── build_and_deploy.bat           # Batch file alternative
├── test_setup.ps1                 # Setup verification script
├── test_device_detection.ps1      # Device detection test
├── repair_platformio.ps1          # PlatformIO package repair script
├── nuclear_repair.ps1             # Nuclear PlatformIO reset script
├── platformio.ini                 # PlatformIO configuration
├── README.md                      # Complete project documentation
├── GETTING_STARTED.md             # Quick start guide
├── TROUBLESHOOTING.md             # Detailed troubleshooting guide
├── wiring_schematic.txt           # Detailed wiring diagrams
├── PROJECT_STRUCTURE.md           # This file
└── .gitignore                     # Git ignore rules
```

## Key Files

### Core Files
- **`src/main.cpp`** - The main Arduino code for the filament extruder controller
- **`platformio.ini`** - Project configuration with Pico W settings and dependencies

### Build Scripts
- **`build_and_deploy.ps1`** - Full-featured PowerShell script with automatic dependency installation
- **`build_and_deploy.bat`** - Simple batch file for manual PlatformIO setups  
- **`test_setup.ps1`** - Quick verification script to test Python/PlatformIO installation
- **`test_device_detection.ps1`** - Test script to verify Pico W detection
- **`repair_platformio.ps1`** - Repair script for PlatformIO package corruption issues
- **`nuclear_repair.ps1`** - Nuclear option to completely reset PlatformIO when standard repair fails

### Documentation
- **`README.md`** - Complete project documentation with features, wiring, and troubleshooting
- **`GETTING_STARTED.md`** - Quick start guide for beginners
- **`TROUBLESHOOTING.md`** - Comprehensive troubleshooting guide for common issues
- **`wiring_schematic.txt`** - ASCII art wiring diagrams and component details

## Generated Files (Created during build)

```
FilamentExtruder/
├── .pio/                          # PlatformIO build directory
│   ├── build/                     # Compiled binaries
│   └── libdeps/                   # Downloaded libraries
└── .vscode/                       # VS Code settings (if using VS Code)
```

These files are automatically created and managed by PlatformIO during the build process.
