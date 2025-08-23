Param(
  [switch]$BuildOnly,
  [string]$Port,
  [int]$TimeoutSeconds = 60,
  [int]$MonitorTimeoutSeconds = 60,
  [int]$Baud = 115200,
  [switch]$NoMonitor,
  [string]$CliPath,
  [string]$Fqbn = 'rp2040:rp2040:rpipicow:usbstack=picosdk',
  [string]$OutputDir = '.\\build',
  [string]$Sketch
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

#region Utility Functions

function Write-Info([string]$Message) { 
    Write-Host "[INFO] $Message" -ForegroundColor Cyan 
}

function Write-Warn([string]$Message) { 
    Write-Host "[WARN] $Message" -ForegroundColor Yellow 
}

function Write-Err([string]$Message) { 
    Write-Host "[ERROR] $Message" -ForegroundColor Red 
}

#endregion

#region Arduino CLI Setup

function Get-ArduinoCliPath {
    <# .SYNOPSIS
       Resolves the path to arduino-cli, downloading if necessary
    #>
    if ($CliPath) {
        if (Test-Path $CliPath) { 
            return (Resolve-Path $CliPath).Path 
        }
        throw "Provided CliPath not found: $CliPath"
    }

    # Check if arduino-cli is in PATH
    $found = (Get-Command arduino-cli -ErrorAction SilentlyContinue)?.Source
    if ($found) { return $found }

    # Check local installation
    $toolsDir = Join-Path $PSScriptRoot '.tools'
    $localCli = Join-Path $toolsDir 'arduino-cli\\arduino-cli.exe'
    if (Test-Path $localCli) { return $localCli }

    # Download arduino-cli
    Write-Info 'arduino-cli not found. Downloading portable arduino-cli...'
    New-Item -ItemType Directory -Force -Path (Split-Path $localCli) | Out-Null

    $zipUrl = 'https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip'
    $zipPath = Join-Path $toolsDir 'arduino-cli_latest_Windows_64bit.zip'
    
    Invoke-WebRequest -Uri $zipUrl -OutFile $zipPath
    Expand-Archive -Path $zipPath -DestinationPath (Split-Path $localCli) -Force
    Remove-Item $zipPath -Force

    if (!(Test-Path $localCli)) { 
        throw 'Failed to install arduino-cli' 
    }
    return $localCli
}

function Install-RequiredComponents([string]$CliPath) {
    <# .SYNOPSIS
       Installs the RP2040 core and required libraries
    #>
    $packageUrl = 'https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json'
    
    Write-Info 'Initializing Arduino CLI configuration'
    & $CliPath config init | Out-Null
    & $CliPath config set board_manager.additional_urls $packageUrl | Out-Null
    
    Write-Info 'Updating core index'
    & $CliPath core update-index | Out-Null
    
    Write-Info 'Installing RP2040 core'
    & $CliPath core install rp2040:rp2040 | Out-Null

    $requiredLibraries = @(
        'Adafruit SSD1306',
        'Adafruit GFX Library',
        'Adafruit BusIO'
    )
    
    foreach ($library in $requiredLibraries) {
        Write-Info "Installing library: $library"
        & $CliPath lib install "$library" | Out-Null
    }
}

#endregion

#region Sketch Management

function Get-SketchPath {
    <# .SYNOPSIS
       Finds the Arduino sketch file (.ino) to compile
    #>
    if ($Sketch) {
        if (Test-Path $Sketch) { 
            return (Resolve-Path $Sketch).Path 
        }
        throw "Sketch not found: $Sketch"
    }

    # Find all .ino files in the project
    $allSketches = Get-ChildItem -Path $PSScriptRoot -Filter '*.ino' -File -Recurse
    if (-not $allSketches) { 
        throw 'No .ino sketch found in project.' 
    }

    # Filter out vendor/build/example directories
    $validSketches = $allSketches | Where-Object {
        $path = $_.FullName
        return ($path -notmatch "\\\.pio\\") -and 
               ($path -notmatch "\\\.tools\\") -and 
               ($path -notmatch "\\examples\\") -and 
               ($path -notmatch "\\libraries\\") -and 
               ($path -notmatch "\\\.git\\")
    }
    
    if (-not $validSketches) { 
        $validSketches = $allSketches 
    }

    # Prefer FilamentExtruder.ino if it exists
    $preferredSketch = $validSketches | Where-Object { 
        $_.Name -ieq 'FilamentExtruder.ino' 
    } | Sort-Object { 
        $_.DirectoryName -eq $PSScriptRoot 
    } -Descending | Select-Object -First 1
    
    if ($preferredSketch) { 
        return $preferredSketch.FullName 
    }

    # Prefer root-level sketches
    $rootSketch = $validSketches | Where-Object { 
        $_.DirectoryName -eq $PSScriptRoot 
    } | Select-Object -First 1
    
    if ($rootSketch) { 
        return $rootSketch.FullName 
    }

    # Choose the sketch with the shortest path
    Push-Location $PSScriptRoot
    try {
        $rankedSketches = $validSketches | ForEach-Object {
            $relativePath = Resolve-Path $_.FullName -Relative
            [PSCustomObject]@{ 
                Sketch = $_
                Depth = ($relativePath -split '\\\\').Length 
            }
        } | Sort-Object Depth, { $_.Sketch.Name }
        
        return $rankedSketches[0].Sketch.FullName
    } finally { 
        Pop-Location 
    }
}

#endregion

#region Build Functions

function Invoke-SketchCompilation([string]$CliPath, [string]$SketchPath, [string]$BoardFqbn, [string]$OutputDirectory) {
    <# .SYNOPSIS
       Compiles the Arduino sketch to UF2 firmware
    #>
    Write-Info "Compiling $([IO.Path]::GetFileName($SketchPath)) for $BoardFqbn"
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    
    & $CliPath compile --fqbn $BoardFqbn --output-dir $OutputDirectory --warnings all --verbose $SketchPath
    if ($LASTEXITCODE -ne 0) { 
        throw "Compilation failed with exit code $LASTEXITCODE" 
    }
}

function Get-CompiledFirmware([string]$OutputDirectory) {
    <# .SYNOPSIS
       Finds the compiled UF2 firmware file
    #>
    $firmwareFile = Get-ChildItem -Path $OutputDirectory -Filter '*.uf2' -File -Recurse | 
                   Sort-Object LastWriteTime -Descending | 
                   Select-Object -First 1
    
    if (-not $firmwareFile) { 
        throw "No UF2 firmware found in '$OutputDirectory'" 
    }
    return $firmwareFile.FullName
}

#endregion

#region Serial Port Detection



function Get-ArduinoDetectedPorts([string]$CliPath, [string]$BoardFqbn) {
    <# .SYNOPSIS
       Gets serial ports detected by arduino-cli with board matching
    #>
    $preferredPorts = @()
    $otherPorts = @()
    
    try {
        # Get JSON output from arduino-cli
        $jsonOutput = & $CliPath board list --format json 2>$null | Out-String
        Write-Info 'arduino-cli board list --format json output:'
        if ($jsonOutput) { 
            Write-Host $jsonOutput 
        } else { 
            Write-Host '[empty]'
            return @(), @()
        }
        
        $boardData = $jsonOutput | ConvertFrom-Json
        
        # Only process detected_ports array (no ports property exists)
        if ($null -eq $boardData.detected_ports) {
            Write-Warn "No detected_ports found in JSON output"
            return @(), @()
        }
        
        $detectedEntries = $boardData.detected_ports
        
        foreach ($entry in $detectedEntries) {
            # First check if this entry has matching_boards property
            if (-not $entry.PSObject.Properties['matching_boards']) {
                Write-Info "Skipping entry - no matching_boards property"
                continue
            }
            
            # Read detected_ports[i].port.address
            $portInfo = $entry.port
            if (-not $portInfo) { 
                Write-Info "Skipping entry - no port info"
                continue 
            }
            
            # Only process serial ports
            if ($portInfo.protocol -ne 'serial') { 
                Write-Info "Skipping port $($portInfo.address) - not serial protocol"
                continue 
            }
            
            $portAddress = $portInfo.address
            if (-not $portAddress) { 
                Write-Info "Skipping entry - no port address"
                continue 
            }
            
            $matchingBoards = $entry.matching_boards
            if (-not $matchingBoards) {
                Write-Info "Skipping $portAddress - no matching_boards"
                continue
            }
            
            # Handle both single object and array cases
            if ($matchingBoards -is [Array]) {
                $boardCount = $matchingBoards.Count
                if ($boardCount -eq 0) {
                    Write-Info "Skipping $portAddress - empty matching_boards array"
                    continue
                }
            } else {
                $boardCount = 1
                $matchingBoards = @($matchingBoards)  # Ensure it's an array
            }
            
            Write-Info "Found serial port with board detection: $portAddress"
            Write-Info "  Checking $boardCount matching boards..."
            
            # Extract target base FQBN once (rp2040:rp2040:rpipicow)
            $targetBaseFqbn = ($BoardFqbn -split ':')[0..2] -join ':'
            Write-Info "  Target base FQBN: $targetBaseFqbn"
            
            $hasTargetBoard = $false
            foreach ($board in $matchingBoards) {
                $boardFqbn = $board.fqbn
                $boardName = $board.name
                
                Write-Info "    Board: $boardName (FQBN: $boardFqbn)"
                
                # Only match if this specific board FQBN matches our target
                if ($boardFqbn -eq $targetBaseFqbn) {
                    $hasTargetBoard = $true
                    Write-Info "    -> FQBN match! ($boardFqbn matches target $targetBaseFqbn)"
                    break
                } else {
                    Write-Info "    -> No match ($boardFqbn != $targetBaseFqbn)"
                }
            }
            
            if ($hasTargetBoard) {
                $preferredPorts += $portAddress
                Write-Info "  -> Added $portAddress to preferred ports"
            } else {
                $otherPorts += $portAddress
                Write-Info "  -> Added $portAddress to other ports"
            }
        }
        
        
    } catch {
        Write-Warn "Failed to parse arduino-cli JSON output: $($_.Exception.Message)"
    }
    
    return @($preferredPorts), @($otherPorts)
}

function Get-SerialPortCandidates([string]$CliPath, [string]$BoardFqbn) {
    <# .SYNOPSIS
       Gets ordered list of serial port candidates for upload attempts
    #>
    # Get ports detected by arduino-cli
    $preferredPorts, $otherPorts = Get-ArduinoDetectedPorts -CliPath $CliPath -BoardFqbn $BoardFqbn
    
    $candidates = @($preferredPorts) + @($otherPorts)
    
    # Log final candidate list
    if ($candidates -and $candidates.Length -gt 0) { 
        Write-Info ("Serial port candidates: " + ($candidates -join ', '))
    } else { 
        Write-Warn 'No serial port candidates found'
    }
    
    return ,$candidates  # Force return as array
}

#endregion

#region Upload Functions

function Copy-FirmwareViaSerial([string]$CliPath, [string]$SketchPath, [string]$BoardFqbn, [string]$Port) {
    <# .SYNOPSIS
       Uploads firmware via serial connection
    #>
    Write-Info "Uploading via serial on $Port"
    & $CliPath upload --fqbn $BoardFqbn -p $Port $SketchPath
    if ($LASTEXITCODE -ne 0) { 
        throw "Serial upload failed with exit code $LASTEXITCODE" 
    }
}

function Wait-ForMassStorageDevice([int]$TimeoutSeconds) {
    <# .SYNOPSIS
       Waits for RPI-RP2 mass storage device to appear
    #>
    $stopwatch = [Diagnostics.Stopwatch]::StartNew()
    
    while ($stopwatch.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        # Try CIM first for broader compatibility
        try {
            $drive = Get-CimInstance Win32_LogicalDisk | 
                    Where-Object { $_.DriveType -eq 2 -or $_.DriveType -eq 3 } | 
                    Where-Object { $_.VolumeName -eq 'RPI-RP2' } | 
                    Select-Object -First 1
            
            if ($drive) { 
                return ($drive.DeviceID + '\\') 
            }
        } catch {
            Write-Warn "CIM drive detection failed: $($_.Exception.Message)"
        }

        # Fallback: look for INFO_UF2.TXT on any drive
        try {
            foreach ($drive in [System.IO.DriveInfo]::GetDrives()) {
                if ($drive.DriveType -in @([IO.DriveType]::Removable, [IO.DriveType]::Fixed) -and $drive.IsReady) {
                    $infoFile = Join-Path $drive.RootDirectory.FullName 'INFO_UF2.TXT'
                    if (Test-Path $infoFile) { 
                        return $drive.RootDirectory.FullName 
                    }
                }
            }
        } catch {
            Write-Warn "DriveInfo detection failed: $($_.Exception.Message)"
        }
        
        Start-Sleep -Milliseconds 300
    }
    
    throw 'Timed out waiting for RPI-RP2 mass storage device'
}

function Copy-FirmwareViaMassStorage([string]$FirmwarePath, [int]$TimeoutSeconds) {
    <# .SYNOPSIS
       Copies firmware to RPI-RP2 mass storage device
    #>
    Write-Info 'Waiting for RPI-RP2 mass storage (hold BOOTSEL and plug in USB if needed)...'
    
    $driveRoot = Wait-ForMassStorageDevice -TimeoutSeconds $TimeoutSeconds
    Write-Info "Found RPI-RP2 at $driveRoot"
    
    $destinationPath = Join-Path $driveRoot ([IO.Path]::GetFileName($FirmwarePath))
    Copy-Item -Path $FirmwarePath -Destination $destinationPath -Force
    Write-Info 'Firmware copy complete'
}

#endregion

#region Serial Monitoring

function Wait-ForSerialPort([string]$CliPath, [string]$BoardFqbn, [string]$PreferredPort, [int]$TimeoutSeconds) {
    <# .SYNOPSIS
       Waits for a serial port to become available
    #>
    if (-not $PreferredPort) {
        Write-Warn "No preferred port specified for monitoring"
        return $null
    }
    
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    
    while ((Get-Date) -lt $deadline) {
        # Use arduino-cli to check if the preferred port is available
        try {
            $preferredPorts, $otherPorts = Get-ArduinoDetectedPorts -CliPath $CliPath -BoardFqbn $BoardFqbn
            $allDetectedPorts = $preferredPorts + $otherPorts
            
            if ($allDetectedPorts -contains $PreferredPort) {
                return $PreferredPort
            }
        } catch {
            Write-Warn "Failed to check port availability: $($_.Exception.Message)"
        }
        
        Start-Sleep -Milliseconds 500
    }
    
    Write-Warn "Timeout waiting for port $PreferredPort to become available"
    return $null
}

function Start-SerialMonitor([string]$CliPath, [string]$Port, [int]$BaudRate) {
    <# .SYNOPSIS
       Starts the Arduino CLI serial monitor
    #>
    Write-Info "Starting serial monitor on $Port @ ${BaudRate} baud (Ctrl+C to exit)"
    & $CliPath monitor -p $Port -c "baudrate=$BaudRate"
}

#endregion

#region Main Script Logic

try {
    # Initialize Arduino CLI
    $arduinoCliPath = Get-ArduinoCliPath
    Write-Info "Using arduino-cli: $arduinoCliPath"
    
    Install-RequiredComponents -CliPath $arduinoCliPath

    # Build firmware
    $sketchPath = Get-SketchPath
    Invoke-SketchCompilation -CliPath $arduinoCliPath -SketchPath $sketchPath -BoardFqbn $Fqbn -OutputDirectory $OutputDir
    
    $firmwarePath = Get-CompiledFirmware -OutputDirectory $OutputDir
    Write-Info "Firmware ready: $firmwarePath"

    # Exit early if build-only mode
    if ($BuildOnly) { 
        Write-Info 'Build-only mode specified. Skipping deployment.'
        exit 0 
    }

    # Try user-specified port first
    if ($Port) {
        try {
            Copy-FirmwareViaSerial -CliPath $arduinoCliPath -SketchPath $sketchPath -BoardFqbn $Fqbn -Port $Port
            
            if (-not $NoMonitor) {
                $monitorPort = Wait-ForSerialPort -CliPath $arduinoCliPath -BoardFqbn $Fqbn -PreferredPort $Port -TimeoutSeconds $MonitorTimeoutSeconds
                if (-not $monitorPort) { 
                    Write-Warn "Could not detect serial port for monitoring; using specified port $Port"
                    $monitorPort = $Port 
                }
                Write-Info "Waiting 2 seconds for device to initialize..."
                Start-Sleep -Seconds 2
                Start-SerialMonitor -CliPath $arduinoCliPath -Port $monitorPort -BaudRate $Baud
            }
            exit 0
        } catch {
            Write-Warn "Serial upload on specified port failed: $($_.Exception.Message). Falling back to auto-detection."
        }
    }

    # Auto-detect and try available serial ports
    $portCandidates = Get-SerialPortCandidates -CliPath $arduinoCliPath -BoardFqbn $Fqbn
    
    if (-not $portCandidates -or $portCandidates.Length -eq 0) {
        Write-Warn 'No serial port candidates found; skipping to mass storage.'
    } else {
        foreach ($candidate in $portCandidates) {
            Write-Info "Attempting serial upload on $candidate"
            try {
                Copy-FirmwareViaSerial -CliPath $arduinoCliPath -SketchPath $sketchPath -BoardFqbn $Fqbn -Port $candidate
                
                if (-not $NoMonitor) {
                    Write-Info "Waiting 2 seconds for device to initialize..."
                    Start-Sleep -Seconds 2
                    Start-SerialMonitor -CliPath $arduinoCliPath -Port $candidate -BaudRate $Baud
                }
                exit 0
            } catch {
                Write-Warn "Serial upload on $candidate failed: $($_.Exception.Message)"
            }
        }
    }

    # Fall back to mass storage upload
    Write-Warn 'All serial upload attempts failed. Falling back to mass storage.'
    
    Copy-FirmwareViaMassStorage -FirmwarePath $firmwarePath -TimeoutSeconds $TimeoutSeconds
    
    if (-not $NoMonitor) {
        Write-Info 'Checking for available serial ports after mass storage deployment...'
        $preferredPorts, $otherPorts = Get-ArduinoDetectedPorts -CliPath $arduinoCliPath -BoardFqbn $Fqbn
        $availablePorts = @($preferredPorts) + @($otherPorts)
        
        if ($availablePorts -and $availablePorts.Length -gt 0) {
            $monitorPort = $availablePorts[0]  # Use first available port
            Write-Info "Waiting 3 seconds for device to initialize after mass storage deployment..."
            Start-Sleep -Seconds 3
            Start-SerialMonitor -CliPath $arduinoCliPath -Port $monitorPort -BaudRate $Baud
        } else {
            Write-Warn 'No serial ports detected after deployment. Connect manually if needed.'
        }
    }
    
    Write-Info 'Deployment complete.'
    exit 0
    
} catch {
    Write-Err $_.Exception.Message
    exit 1
}

#endregion