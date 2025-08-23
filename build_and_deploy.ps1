#Requires -Version 5.1

<#
.SYNOPSIS
    Build and Deploy Script for Raspberry Pi Pico W Filament Extruder Controller

.DESCRIPTION
    This script checks for PlatformIO installation, installs it if missing,
    builds the project, and deploys it to the connected Raspberry Pi Pico W.

.PARAMETER SkipInstall
    Skip the PlatformIO installation check and proceed directly to build

.PARAMETER BuildOnly
    Only build the project without deploying

.PARAMETER UF2Only
    Force UF2 upload method (requires Pico W in bootloader mode)

.PARAMETER Verbose
    Enable verbose output for debugging

.EXAMPLE
    .\build_and_deploy.ps1
    
.EXAMPLE
    .\build_and_deploy.ps1 -BuildOnly -Verbose

.EXAMPLE
    .\build_and_deploy.ps1 -UF2Only
#>

[CmdletBinding()]
param(
    [switch]$SkipInstall,
    [switch]$BuildOnly,
    [switch]$UF2Only
)

# Set error action preference
$ErrorActionPreference = "Stop"

# Script-scoped variables
$script:pioCommand = $null
$script:pythonCommand = $null

# Colors for output
function Write-ColoredOutput {
    param(
        [string]$Message,
        [string]$Color = "White"
    )
    Write-Host $Message -ForegroundColor $Color
}

function Write-Success { param([string]$Message) Write-ColoredOutput $Message "Green" }
function Write-Warning { param([string]$Message) Write-ColoredOutput $Message "Yellow" }
function Write-Error { param([string]$Message) Write-ColoredOutput $Message "Red" }
function Write-Info { param([string]$Message) Write-ColoredOutput $Message "Cyan" }

# Header
Write-Info "=========================================="
Write-Info "  Pico W Filament Extruder Controller"
Write-Info "     Build & Deploy Script"
Write-Info "=========================================="
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "platformio.ini")) {
    Write-Error "Error: platformio.ini not found in current directory"
    Write-Error "Please run this script from the project root directory"
    exit 1
}

Write-Success "✓ Found platformio.ini - we're in the right directory"

# Function to check if a command exists
function Test-Command {
    param([string]$CommandName)
    try {
        $null = Get-Command $CommandName -ErrorAction Stop
        return $true
    }
    catch {
        return $false
    }
}

# Function to check Python installation
function Test-PythonInstallation {
    Write-Info "Checking Python installation..."
    
    $pythonCommands = @("python", "python3", "py")
    $pythonCmd = $null
    
    foreach ($cmd in $pythonCommands) {
        if (Test-Command $cmd) {
            try {
                $version = & $cmd --version 2>&1
                if ($version -match "Python (\d+)\.(\d+)") {
                    $major = [int]$matches[1]
                    $minor = [int]$matches[2]
                    if ($major -gt 3 -or ($major -eq 3 -and $minor -ge 7)) {
                        Write-Success "✓ Found Python: $version"
                        return $cmd
                    }
                }
            }
            catch {
                # Continue to next command
            }
        }
    }
    
    return $null
}

# Function to install Python if missing
function Install-Python {
    Write-Warning "Python 3.7+ is required for PlatformIO"
    Write-Info "Attempting to install Python using winget..."
    
    if (Test-Command "winget") {
        try {
            Write-Info "Installing Python..."
            winget install Python.Python.3.12
            Write-Success "✓ Python installed successfully"
            
            # Refresh PATH
            $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "User")
            
            return Test-PythonInstallation
        }
        catch {
            Write-Error "Failed to install Python using winget: $_"
            return $null
        }
    }
    else {
        Write-Error "winget not available. Please install Python 3.7+ manually from https://python.org"
        return $null
    }
}

# Function to check PlatformIO installation
function Test-PlatformIOInstallation {
    Write-Info "Checking PlatformIO installation..."
    
    # Try direct pio command first
    if (Test-Command "pio") {
        try {
            $version = pio --version 2>&1
            Write-Success "✓ PlatformIO found: $version"
            $script:pioCommand = "pio"
            return $true
        }
        catch {
            # Fall through to Python module check
        }
    }
    
    # Try Python module approach
    $pythonCommands = @("python", "python3", "py")
    foreach ($pythonCmd in $pythonCommands) {
        if (Test-Command $pythonCmd) {
            try {
                $version = & $pythonCmd -m platformio --version 2>&1
                if ($version -and $version -notmatch "No module named") {
                    Write-Success "✓ PlatformIO found via Python module: $version"
                    $script:pioCommand = "$pythonCmd -m platformio"
                    return $true
                }
            }
            catch {
                # Continue to next Python command
            }
        }
    }
    
    return $false
}

# Function to install PlatformIO
function Install-PlatformIO {
    param([string]$PythonCommand)
    
    Write-Info "Installing PlatformIO..."
    
    try {
        # Upgrade pip first
        Write-Info "Upgrading pip..."
        & $PythonCommand -m pip install --upgrade pip
        
        # Install PlatformIO
        Write-Info "Installing PlatformIO Core..."
        & $PythonCommand -m pip install platformio
        
        # Get the Scripts directory where PlatformIO is installed
        Write-Info "Locating PlatformIO installation..."
        $userBase = & $PythonCommand -c "import site; print(site.USER_BASE)"
        $pioPath = Join-Path $userBase "Scripts"
        $pioExe = Join-Path $pioPath "pio.exe"
        
        # Also try Python's Scripts directory
        $pythonScripts = Split-Path (& $PythonCommand -c "import sys; print(sys.executable)") -Parent
        $pythonScripts = Join-Path $pythonScripts "Scripts"
        $pioExeAlt = Join-Path $pythonScripts "pio.exe"
        
        # Update session PATH immediately
        if (Test-Path $pioExe) {
            Write-Info "Found PlatformIO at: $pioPath"
            $env:PATH = $env:PATH + ";" + $pioPath
            $script:pioCommand = $pioExe
        }
        elseif (Test-Path $pioExeAlt) {
            Write-Info "Found PlatformIO at: $pythonScripts"
            $env:PATH = $env:PATH + ";" + $pythonScripts
            $script:pioCommand = $pioExeAlt
        }
        else {
            # Try to find pio.exe anywhere in Python installation
            Write-Info "Searching for PlatformIO executable..."
            $pythonDir = Split-Path (& $PythonCommand -c "import sys; print(sys.executable)") -Parent
            $pioFiles = Get-ChildItem -Path $pythonDir -Name "pio.exe" -Recurse -ErrorAction SilentlyContinue
            if ($pioFiles) {
                $foundPio = Join-Path $pythonDir $pioFiles[0]
                $foundDir = Split-Path $foundPio -Parent
                Write-Info "Found PlatformIO at: $foundDir"
                $env:PATH = $env:PATH + ";" + $foundDir
                $script:pioCommand = $foundPio
            }
        }
        
        # Update user PATH permanently
        $currentUserPath = [Environment]::GetEnvironmentVariable("PATH", "User")
        if ($currentUserPath -notlike "*$pioPath*" -and (Test-Path $pioPath)) {
            Write-Info "Adding PlatformIO to permanent PATH..."
            [Environment]::SetEnvironmentVariable("PATH", $currentUserPath + ";" + $pioPath, "User")
        }
        
        # Verify installation - try direct command first, then fallback to python module
        Start-Sleep 1
        $pioWorking = $false
        
        try {
            if ($script:pioCommand -and (Test-Path $script:pioCommand)) {
                $version = & $script:pioCommand --version 2>&1
                Write-Success "✓ PlatformIO installed successfully: $version"
                $pioWorking = $true
            }
        }
        catch {
            Write-Info "Direct pio command failed, trying Python module..."
        }
        
        if (-not $pioWorking) {
            try {
                $version = & $PythonCommand -m platformio --version 2>&1
                Write-Success "✓ PlatformIO installed successfully (via Python module): $version"
                $script:pioCommand = "$PythonCommand -m platformio"
                $pioWorking = $true
            }
            catch {
                Write-Error "PlatformIO installation verification failed"
            }
        }
        
        return $pioWorking
    }
    catch {
        Write-Error "Failed to install PlatformIO: $_"
        return $false
    }
}

# Function to build the project
function Build-Project {
    Write-Info "Building project..."
    
    # Use the detected PlatformIO command or fallback to direct command
    $pioCmd = if ($script:pioCommand) { $script:pioCommand } else { "pio" }
    
    Write-Info "Using PlatformIO command: $pioCmd"
    
    $buildOutput = ""
    $buildExitCode = 0
    
    try {
        if ($script:pioCommand -and $script:pioCommand.Contains("-m platformio")) {
            # Using Python module approach
            $parts = $script:pioCommand -split " "
            $pythonExe = $parts[0]
            $moduleArgs = $parts[1..($parts.Length-1)]
            $buildOutput = & $pythonExe @moduleArgs run 2>&1
            $buildExitCode = $LASTEXITCODE
        }
        else {
            # Using direct PlatformIO executable
            $buildOutput = & $pioCmd run 2>&1
            $buildExitCode = $LASTEXITCODE
        }
        
        # Display build output
        Write-Host $buildOutput
        
        # Analyze build result
        $hasErrors = $buildOutput -match "Error|Failed|error:|failed:"
        $hasCriticalErrors = $buildOutput -match "MissingPackageManifestError|PackageInstallError|PlatformInstallError"
        
        if ($buildExitCode -eq 0 -and -not $hasErrors -and -not $hasCriticalErrors) {
            Write-Success "✓ Build completed successfully"
            return $true
        }
        elseif ($buildExitCode -eq 0 -and $hasCriticalErrors -and -not $hasErrors) {
            Write-Warning "⚠️  Build completed with warnings (package manifest issues)"
            Write-Warning "This may affect some PlatformIO features but code should still work"
            return $true
        }
        else {
            Write-Error "Build failed!"
            Write-Error "Exit code: $buildExitCode"
            if ($hasErrors) {
                Write-Error "Detected compilation errors in output"
            }
            if ($hasCriticalErrors) {
                Write-Error "Detected critical package/platform errors"
                Write-Warning ""
                Write-Warning "This appears to be a PlatformIO package/platform issue."
                Write-Warning "Try these solutions:"
                Write-Warning "  1. Clean and reinstall packages:"
                Write-Warning "     python -m platformio run --target clean"
                Write-Warning "     python -m platformio platform uninstall raspberrypi"
                Write-Warning "     python -m platformio platform install raspberrypi"
                Write-Warning "  2. Update PlatformIO:"
                Write-Warning "     python -m pip install --upgrade platformio"
                Write-Warning "  3. Clear PlatformIO cache:"
                Write-Warning "     python -m platformio system prune --dry-run"
                Write-Warning ""
                
                $tryFix = Read-Host "Would you like me to try cleaning and reinstalling the platform? (y/n)"
                if ($tryFix -eq "y" -or $tryFix -eq "Y") {
                    return Repair-PlatformIOPackages
                }
            }
            return $false
        }
    }
    catch {
        Write-Error "Build execution failed: $_"
        return $false
    }
}

# Function to repair PlatformIO packages
function Repair-PlatformIOPackages {
    Write-Info "Attempting to repair PlatformIO packages..."
    
    $pythonCmd = if ($script:pythonCommand) { $script:pythonCommand } else { "python" }
    
    try {
        # Clean current build
        Write-Info "1. Cleaning current build..."
        & $pythonCmd -m platformio run --target clean
        
        # Remove raspberrypi platform
        Write-Info "2. Uninstalling raspberrypi platform..."
        & $pythonCmd -m platformio platform uninstall raspberrypi 2>&1 | Out-Null
        
        # Reinstall raspberrypi platform
        Write-Info "3. Reinstalling raspberrypi platform..."
        & $pythonCmd -m platformio platform install raspberrypi
        
        # Update libraries
        Write-Info "4. Updating libraries..."
        & $pythonCmd -m platformio lib update
        
        Write-Success "✓ Package repair completed"
        
        # Try building again
        Write-Info "5. Attempting rebuild..."
        return Build-Project
        
    }
    catch {
        Write-Error "Package repair failed: $_"
        Write-Warning "Manual recovery options:"
        Write-Warning "  1. Delete .pio folder in project directory"
        Write-Warning "  2. Run: python -m platformio platform install raspberrypi --force"
        Write-Warning "  3. Restart PowerShell and try again"
        return $false
    }
}

# Function to check for RPI-RP2 bootloader drive
function Find-PicoBootloaderDrive {
    Write-Info "Checking for Pico W in bootloader mode..."
    
    # Get all drives and look for one with very small capacity (typical of bootloader)
    $drives = Get-PSDrive -PSProvider FileSystem | Where-Object { 
        $_.Free -lt 1GB -and $_.Free -gt 0 -and $_.Used -lt 100MB
    }
    
    foreach ($drive in $drives) {
        $drivePath = $drive.Name + ":\"
        
        # Check if drive contains typical bootloader files or is named RPI-RP2
        $hasInfoFile = Test-Path (Join-Path $drivePath "INFO_UF2.TXT")
        $hasIndexFile = Test-Path (Join-Path $drivePath "INDEX.HTM")
        
        if ($hasInfoFile -or $hasIndexFile -or $drive.Description -like "*RPI*") {
            Write-Success "✓ Found Pico W bootloader drive: $($drive.Name):\"
            return $drive.Name + ":"
        }
    }
    
    return $null
}

# Function to copy UF2 firmware directly to bootloader drive
function Copy-UF2Firmware {
    param([string]$BootloaderDrive)
    
    $firmwarePath = ".pio\build\picow\firmware.uf2"
    
    if (-not (Test-Path $firmwarePath)) {
        Write-Error "Firmware file not found: $firmwarePath"
        Write-Error "Build may have failed or target path incorrect"
        return $false
    }
    
    # Check firmware file size (should be reasonable for a Pico project)
    $firmwareFile = Get-Item $firmwarePath
    $firmwareSize = $firmwareFile.Length
    
    Write-Info "Firmware file size: $([math]::Round($firmwareSize/1024, 1)) KB"
    
    if ($firmwareSize -lt 1024) {
        Write-Warning "⚠️  Firmware file seems very small ($firmwareSize bytes) - build may be incomplete"
        $continue = Read-Host "Continue with upload anyway? (y/n)"
        if ($continue -ne "y" -and $continue -ne "Y") {
            Write-Info "Upload cancelled by user"
            return $false
        }
    }
    elseif ($firmwareSize -gt 2MB) {
        Write-Warning "⚠️  Firmware file is very large ($([math]::Round($firmwareSize/1024/1024, 1)) MB) - may not fit on Pico"
        $continue = Read-Host "Continue with upload anyway? (y/n)"
        if ($continue -ne "y" -and $continue -ne "Y") {
            Write-Info "Upload cancelled by user"  
            return $false
        }
    }
    
    try {
        Write-Info "Copying firmware to Pico W bootloader drive..."
        $destination = Join-Path $BootloaderDrive "firmware.uf2"
        
        Copy-Item $firmwarePath $destination -Verbose
        
        # Wait a moment for the copy to complete
        Start-Sleep -Seconds 2
        
        # Check if the drive still exists (it should disappear after successful upload)
        if (Test-Path $BootloaderDrive) {
            Write-Warning "⚠️  Bootloader drive still present - checking upload status..."
            Start-Sleep -Seconds 3
            if (-not (Test-Path $BootloaderDrive)) {
                Write-Success "✓ Firmware uploaded successfully via UF2 copy!"
                Write-Success "✓ Pico W has rebooted and is running your code!"
                return $true
            }
            else {
                Write-Warning "Upload may have failed - bootloader drive still present"
                return $false
            }
        }
        else {
            Write-Success "✓ Firmware uploaded successfully via UF2 copy!"
            Write-Success "✓ Pico W has rebooted and is running your code!"
            Write-Info ""
            Write-Info "🎯 Your Filament Extruder Controller is now active:"
            Write-Info "  • Monitoring temperature via thermistor on pin A0"
            Write-Info "  • Controlling speed via potentiometer (I2C address 0x35)"
            Write-Info "  • Controlling temperature via potentiometer (I2C address 0x36)"
            Write-Info "  • Driving heater relays on GPIO pins 18-21"
            Write-Info "  • Displaying status on SSD1306 OLED (I2C pins 8/9)"
            Write-Info "  • Serial debug output at 115200 baud"
            return $true
        }
    }
    catch {
        Write-Error "Failed to copy firmware: $_"
        return $false
    }
}

# Function to deploy the project
function Deploy-Project {
    Write-Info "Checking for connected Pico W..."
    
    # If UF2Only mode is requested, only try UF2 method
    if ($UF2Only) {
        Write-Info "🔧 UF2-only mode requested - looking for bootloader drive..."
        $bootloaderDrive = Find-PicoBootloaderDrive
        if ($bootloaderDrive) {
            Write-Info "🎯 Pico W detected in bootloader mode - using direct UF2 upload"
            return Copy-UF2Firmware $bootloaderDrive
        }
        else {
            Write-Error "❌ UF2-only mode: Pico W not found in bootloader mode"
            Write-Warning ""
            Write-Info "To use UF2-only mode:"
            Write-Info "  1. Disconnect USB cable from Pico W"
            Write-Info "  2. Hold BOOTSEL button on Pico W"
            Write-Info "  3. Connect USB cable while holding BOOTSEL"
            Write-Info "  4. Release BOOTSEL - should see RPI-RP2 drive"
            Write-Info "  5. Run script again with -UF2Only"
            return $false
        }
    }
    
    # First, check if Pico W is in bootloader mode (most reliable method)
    $bootloaderDrive = Find-PicoBootloaderDrive
    if ($bootloaderDrive) {
        Write-Info "🎯 Pico W detected in bootloader mode - using direct UF2 upload"
        return Copy-UF2Firmware $bootloaderDrive
    }
    
    # Use the detected PlatformIO command or fallback to direct command
    $pioCmd = if ($script:pioCommand) { $script:pioCommand } else { "pio" }
    
    # List connected devices for serial upload
    try {
        Write-Info "Scanning for serial devices..."
        
        if ($script:pioCommand -and $script:pioCommand.Contains("-m platformio")) {
            # Using Python module approach
            $parts = $script:pioCommand -split " "
            $pythonExe = $parts[0]
            $moduleArgs = $parts[1..($parts.Length-1)]
            $devices = & $pythonExe @moduleArgs device list 2>&1
        }
        else {
            # Using direct PlatformIO executable
            $devices = & $pioCmd device list 2>&1
        }
        
        Write-Info "Connected devices:"
        Write-Host $devices
        
        # Check if we have a Pico device or other USB serial devices
        $hasUSBDevice = $false
        $hasPicoDevice = $false
        if ($devices -and $devices -is [string]) {
            $deviceLines = $devices -split "`n"
            foreach ($line in $deviceLines) {
                if ($line -match "COM\d+") {
                    # Check for Raspberry Pi Pico (VID:PID=2E8A:*)
                    if ($line -match "2E8A:" -or $line -match "VID:PID=2E8A") {
                        $hasUSBDevice = $true
                        $hasPicoDevice = $true
                        Write-Info "🎯 Raspberry Pi Pico detected on $(($line -split ' ')[0])"
                        break
                    }
                    # Check for other USB devices (not Bluetooth)
                    elseif ($line -notmatch "Bluetooth" -and $line -notmatch "BTHENUM" -and ($line -match "USB VID:PID=" -or $line -match "Serial Device")) {
                        $hasUSBDevice = $true
                        Write-Info "🔌 USB Serial Device detected: $(($line -split ' ')[0])"
                    }
                }
            }
        }
        
        if ($hasPicoDevice) {
            Write-Success "✓ Raspberry Pi Pico detected - trying serial upload..."
        }
        elseif ($hasUSBDevice) {
            Write-Info "ℹ️  USB Serial device detected (may be compatible)"
        }
        else {
            Write-Warning "⚠️  No USB devices detected!"
            Write-Warning "All detected devices appear to be Bluetooth connections."
            Write-Warning ""
            Write-Info "📋 To upload to Pico W, you have two options:"
            Write-Info ""
            Write-Success "Option 1: Bootloader Mode (RECOMMENDED)"
            Write-Info "  1. Disconnect USB cable from Pico W"
            Write-Info "  2. Hold down the BOOTSEL button on Pico W"
            Write-Info "  3. While holding BOOTSEL, connect USB cable"
            Write-Info "  4. Release BOOTSEL - Pico W appears as RPI-RP2 drive"
            Write-Info "  5. Run this script again"
            Write-Info ""
            Write-Success "Option 2: Serial Upload"
            Write-Info "  1. Connect Pico W via USB (normal connection)"
            Write-Info "  2. Ensure it appears as a COM port in Device Manager"
            Write-Info "  3. Continue with this script"
            Write-Warning ""
            $continue = Read-Host "Continue with serial upload attempt? (y/n)"
            if ($continue -ne "y" -and $continue -ne "Y") {
                Write-Info "Upload cancelled by user"
                Write-Info ""
                Write-Info "💡 TIP: For most reliable upload, use bootloader mode (Option 1)"
                return $false
            }
        }
        
        Write-Info "Attempting serial upload to Pico W..."
        
        $uploadOutput = ""
        $uploadExitCode = 0
        
        if ($script:pioCommand -and $script:pioCommand.Contains("-m platformio")) {
            # Using Python module approach
            try {
                $uploadOutput = & $pythonExe @moduleArgs run --target upload 2>&1
                $uploadExitCode = $LASTEXITCODE
            }
            catch {
                $uploadOutput = $_.Exception.Message
                $uploadExitCode = 1
            }
        }
        else {
            # Using direct PlatformIO executable  
            try {
                $uploadOutput = & $pioCmd run --target upload 2>&1
                $uploadExitCode = $LASTEXITCODE
            }
            catch {
                $uploadOutput = $_.Exception.Message
                $uploadExitCode = 1
            }
        }
        
        # Analyze upload output to determine actual success
        Write-Host $uploadOutput
        
        if ($uploadExitCode -eq 0 -and 
            $uploadOutput -notmatch "Error" -and 
            $uploadOutput -notmatch "Failed" -and
            $uploadOutput -notmatch "Could not find" -and
            $uploadOutput -notmatch "No device found") {
            Write-Success "✓ Serial upload completed successfully"
            Write-Success "✓ Your Filament Extruder Controller is ready!"
            Write-Info ""
            Write-Info "🎯 Your Filament Extruder Controller is now active:"
            Write-Info "  • Monitoring temperature via thermistor on pin A0"
            Write-Info "  • Controlling speed via potentiometer (I2C address 0x35)"
            Write-Info "  • Controlling temperature via potentiometer (I2C address 0x36)"
            Write-Info "  • Driving heater relays on GPIO pins 18-21"
            Write-Info "  • Displaying status on SSD1306 OLED (I2C pins 8/9)"
            Write-Info "  • Serial debug output at 115200 baud"
            return $true
        }
        else {
            Write-Warning "⚠️  Serial upload failed - trying bootloader mode fallback..."
            Write-Warning ""
            Write-Info "Checking again for bootloader mode..."
            
            # Give user a chance to put device in bootloader mode
            Write-Info "Please put your Pico W in bootloader mode now:"
            Write-Info "  1. Disconnect USB cable"
            Write-Info "  2. Hold BOOTSEL button" 
            Write-Info "  3. Connect USB while holding BOOTSEL"
            Write-Info "  4. Release BOOTSEL"
            Write-Info ""
            $bootloadAttempt = Read-Host "Press Enter when Pico W is in bootloader mode (or 'q' to quit)"
            
            if ($bootloadAttempt -eq "q" -or $bootloadAttempt -eq "Q") {
                Write-Info "Upload cancelled by user"
                return $false
            }
            
            # Check for bootloader drive again
            $bootloaderDrive = Find-PicoBootloaderDrive
            if ($bootloaderDrive) {
                Write-Success "✓ Bootloader drive found! Uploading firmware..."
                return Copy-UF2Firmware $bootloaderDrive
            }
            else {
                Write-Error "❌ Upload failed - could not detect Pico W"
                Write-Error "Exit code: $uploadExitCode"
                Write-Warning ""
                Write-Warning "🔧 Troubleshooting steps:"
                Write-Warning "  1. Ensure Pico W is connected via USB DATA cable (not power-only)"
                Write-Warning "  2. Try different USB port or cable"
                Write-Warning "  3. Check Windows Device Manager:"
                Write-Warning "     - In bootloader mode: Should see 'RPI RP2' device"
                Write-Warning "     - In normal mode: Should see 'USB Serial Device' on COMx"
                Write-Warning "  4. Verify BOOTSEL button press sequence"
                Write-Warning "  5. Try restarting your computer if drivers are problematic"
                Write-Warning ""
                Write-Info "🎯 Manual upload option:"
                Write-Info "  1. Put Pico W in bootloader mode"
                Write-Info "  2. Copy .pio\\build\\picow\\firmware.uf2 to RPI-RP2 drive"
                return $false
            }
        }
    }
    catch {
        Write-Error "Device detection failed: $_"
        Write-Warning "Make sure your Raspberry Pi Pico W is:"
        Write-Warning "  1. Connected via USB cable (not just power)"
        Write-Warning "  2. Either in bootloader mode OR running compatible firmware"
        Write-Warning "  3. Recognized by Windows (check Device Manager)"
        return $false
    }
}

# Main execution
try {
    if (-not $SkipInstall) {
        # Check Python installation
        $pythonCmd = Test-PythonInstallation
        if (-not $pythonCmd) {
            $pythonCmd = Install-Python
            if (-not $pythonCmd) {
                Write-Error "Cannot proceed without Python. Please install Python 3.7+ and try again."
                exit 1
            }
        }
        
        # Store Python command for use by repair functions
        $script:pythonCommand = $pythonCmd
        Write-Debug "Python command available: $pythonCmd"
        
        # Check PlatformIO installation
        if (-not (Test-PlatformIOInstallation)) {
            if (-not (Install-PlatformIO $pythonCmd)) {
                Write-Error "Cannot proceed without PlatformIO. Installation failed."
                exit 1
            }
        }
    }
    
    Write-Host ""
    Write-Info "Starting build process..."
    Write-Host ""
    
    # Build the project
    if (-not (Build-Project)) {
        exit 1
    }
    
    # Deploy if requested
    if (-not $BuildOnly) {
        Write-Host ""
        Write-Info "Starting deployment..."
        Write-Host ""
        
        if (-not (Deploy-Project)) {
            Write-Warning "Build completed but deployment failed"
            Write-Info "Manual upload options:"
            Write-Info "  1. Connect Pico W in bootloader mode (hold BOOTSEL)"
            Write-Info "  2. Run: python -m platformio run --target upload"
            Write-Info "  3. Or try: pio run --target upload"
            exit 1
        }
    }
    else {
        Write-Info "Build-only mode: Skipping deployment"
    }
    
    Write-Host ""
    Write-Success "=========================================="
    if ($BuildOnly) {
        Write-Success "      BUILD COMPLETED SUCCESSFULLY"
    }
    else {
        Write-Success "   BUILD & DEPLOY COMPLETED SUCCESSFULLY"
    }
    Write-Success "=========================================="
    Write-Host ""
    
    if (-not $BuildOnly) {
        Write-Info "Your Filament Extruder Controller is now running with:"
        Write-Info "  • Speed control via Potentiometer 1"
        Write-Info "  • Temperature monitoring via Potentiometer 2"
        Write-Info "  • Real-time display on SSD1306 OLED"
        Write-Info "  • I2C communication on GPIO8 (SDA) & GPIO9 (SCL)"
        Write-Host ""
        Write-Info "Connect to serial monitor at 115200 baud for debugging"
    }
}
catch {
    Write-Error "Script execution failed: $_"
    exit 1
}

Write-Host ""
Write-Info "Script completed successfully!"
