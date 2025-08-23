# Troubleshooting Guide

## PowerShell Script Issues

### "PlatformIO installation failed - command not found after install"

This is the most common issue. Try these solutions in order:

#### Solution 1: Restart PowerShell
1. Close PowerShell completely
2. Open a new PowerShell window
3. Run the script again: `.\build_and_deploy.ps1`

#### Solution 2: Use the -SkipInstall flag
If you know PlatformIO is installed but the script can't find it:
```powershell
.\build_and_deploy.ps1 -SkipInstall
```

#### Solution 3: Manual PlatformIO Installation
Install PlatformIO manually, then use the script:
```powershell
# Install manually
python -m pip install platformio

# Then run the script
.\build_and_deploy.ps1 -SkipInstall
```

#### Solution 4: Check PATH
Verify PlatformIO is in your PATH:
```powershell
# Check if PlatformIO is accessible
pio --version

# If not found, try the Python module directly
python -m platformio --version
```

### "Execution Policy" Errors

If you get execution policy errors:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### "winget not found" Errors

If Python installation fails:
1. Install Python manually from [python.org](https://python.org)
2. Make sure to check "Add to PATH" during installation
3. Restart PowerShell and try again

## Hardware Issues

### Display Not Working
- **Try different I2C address**: Change `SCREEN_ADDRESS` from `0x3D` to `0x3C` (or vice versa)
- **Check connections**: Verify SDA/SCL wiring to GPIO8/GPIO9
- **Check power**: Ensure 3.3V and GND connections are solid

### ADC Not Responding
- **Verify I2C addresses**: Use `pio device list` or serial monitor to see detected devices
- **Check ADC connections**: Ensure proper wiring between ADC and Pico W
- **Test with I2C scanner**: The built-in I2C scanner will show detected addresses

### Upload Issues

#### False Success Reports
**Problem**: Script reports "Upload completed successfully" but device wasn't connected.
**Solution**: This has been fixed in the latest script version. The new script:
- Detects when only Bluetooth devices are present
- Warns you before attempting upload
- Analyzes upload output for actual errors
- Checks exit codes properly

#### Real Upload Failures
- **Enter bootloader mode**: Hold BOOTSEL button while plugging in USB
- **Check USB cable**: Try a different USB cable (some are power-only)
- **Try different port**: Use a different USB port
- **Check device manager**: Look for "RPI RP2" device in Windows Device Manager

#### Confirming Successful Upload
A real successful upload should show:
- Pico W appears as "RPI-RP2" drive in Windows Explorer (bootloader mode)
- OR Pico W appears as USB Serial device in Device Manager (after code upload)
- No "Error", "Failed", or "Could not find" messages in output
- Exit code 0 from PlatformIO

## Build Issues

### MissingPackageManifestError

**Problem**: Build fails with "MissingPackageManifestError: Could not find one of 'package.json' manifest files in the package"

This is a **PlatformIO package corruption issue**, not a code problem.

**Quick Fix**: Run the repair script:
```powershell
.\repair_platformio.ps1
```

**Manual Fix**:
```powershell
# Clean and reinstall platform
python -m platformio run --target clean
python -m platformio platform uninstall raspberrypi
python -m platformio platform install raspberrypi
python -m platformio lib update
```

**Nuclear Option** (if repair script fails):
```powershell
.\nuclear_repair.ps1
```

**Manual Nuclear Option**:
1. Delete the `.pio` folder in your project directory
2. Delete your entire PlatformIO user directory:
   - Windows: `%USERPROFILE%\.platformio`
3. Restart PowerShell completely
4. Reinstall: `python -m pip install platformio --force-reinstall`
5. Run: `python -m platformio platform install raspberrypi --force`
6. Try building again

### Library Download Fails
```powershell
# Clean and rebuild
pio run --target clean
pio run
```

### Compilation Errors
- Check that you're using the correct board settings in `platformio.ini`
- Verify all include files are present
- Try cleaning and rebuilding

## Alternative Commands

If the script doesn't work, try these manual commands:

### Manual Build
```bash
# Navigate to project directory
cd path\to\FilamentExtruder

# Build
pio run

# Upload (with Pico W in bootloader mode)  
pio run --target upload
```

### Manual Python Module Commands
```bash
# If pio command doesn't work, use Python module directly
python -m platformio run
python -m platformio run --target upload
```

## Getting Help

### Check Serial Output
Connect to your Pico W at 115200 baud to see debug information:
- Speed and temperature values
- I2C device scan results  
- Error messages

### Enable Verbose Mode
For more detailed output:
```powershell
.\build_and_deploy.ps1 -Verbose
```

### Check I2C Devices
The code includes an I2C scanner that runs at startup. Check serial output for:
```
I2C device found at address 0x3C  # Display
I2C device found at address 0x35  # ADC 1  
I2C device found at address 0x36  # ADC 2
```

If devices aren't found, check wiring and power connections.
