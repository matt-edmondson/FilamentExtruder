# Quick Device Detection Test
# This script tests if we can properly detect a connected Pico W

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "    Pico W Device Detection Test" -ForegroundColor Cyan  
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

# Test device detection
try {
    Write-Host "Scanning for devices..." -ForegroundColor Yellow
    
    # Try both python module and direct pio command
    $devices = $null
    
    if (Get-Command "python" -ErrorAction SilentlyContinue) {
        try {
            $devices = python -m platformio device list 2>&1
            Write-Host "Using: python -m platformio device list" -ForegroundColor Gray
        }
        catch {
            Write-Host "Python module approach failed" -ForegroundColor Red
        }
    }
    
    if (-not $devices -and (Get-Command "pio" -ErrorAction SilentlyContinue)) {
        try {
            $devices = pio device list 2>&1
            Write-Host "Using: pio device list" -ForegroundColor Gray
        }
        catch {
            Write-Host "Direct pio command failed" -ForegroundColor Red
        }
    }
    
    if ($devices) {
        Write-Host ""
        Write-Host "Raw device list output:" -ForegroundColor Green
        Write-Host "------------------------" -ForegroundColor Gray
        Write-Host $devices
        Write-Host "------------------------" -ForegroundColor Gray
        Write-Host ""
        
        # Analyze devices
        Write-Host "Device Analysis:" -ForegroundColor Yellow
        $hasUSBDevice = $false
        $hasPicoDevice = $false
        $bluetoothCount = 0
        $usbCount = 0
        $picoCount = 0
        
        if ($devices -is [string]) {
            $deviceLines = $devices -split "`n"
            foreach ($line in $deviceLines) {
                $line = $line.Trim()
                if ($line -and $line -match "COM\d+") {
                    if ($line -match "Bluetooth" -or $line -match "BTHENUM") {
                        $bluetoothCount++
                        Write-Host "  🔵 Bluetooth: $line" -ForegroundColor Blue
                    }
                    elseif ($line -match "2E8A:" -or $line -match "VID:PID=2E8A") {
                        $hasUSBDevice = $true
                        $hasPicoDevice = $true
                        $picoCount++
                        $usbCount++
                        Write-Host "  🎯 Raspberry Pi Pico: $line" -ForegroundColor Green
                    }
                    elseif ($line -notmatch "Bluetooth" -and $line -notmatch "BTHENUM" -and ($line -match "USB VID:PID=" -or $line -match "Serial Device")) {
                        $hasUSBDevice = $true
                        $usbCount++
                        Write-Host "  🟢 USB/Serial: $line" -ForegroundColor Green
                    }
                }
            }
        }
        
        Write-Host ""
        Write-Host "Summary:" -ForegroundColor Cyan
        Write-Host "  Raspberry Pi Picos: $picoCount" -ForegroundColor $(if ($picoCount -gt 0) { "Green" } else { "Gray" })
        Write-Host "  Other USB/Serial devices: $(($usbCount - $picoCount))" -ForegroundColor $(if (($usbCount - $picoCount) -gt 0) { "Green" } else { "Gray" })
        Write-Host "  Bluetooth devices: $bluetoothCount" -ForegroundColor Blue
        
        if ($hasPicoDevice) {
            Write-Host ""
            Write-Host "🎯 Raspberry Pi Pico detected - perfect for upload!" -ForegroundColor Green
            Write-Host "Your Pico W is ready to receive the filament extruder firmware." -ForegroundColor Green
        }
        elseif ($hasUSBDevice) {
            Write-Host ""
            Write-Host "✓ USB Serial device detected - may be compatible for upload" -ForegroundColor Yellow
        }
        else {
            Write-Host ""
            Write-Host "⚠️  No USB devices detected" -ForegroundColor Yellow
            Write-Host "To connect Pico W:" -ForegroundColor Yellow
            Write-Host "  1. Hold BOOTSEL button on Pico W" -ForegroundColor White
            Write-Host "  2. Connect USB cable while holding button" -ForegroundColor White
            Write-Host "  3. Release BOOTSEL button" -ForegroundColor White
            Write-Host "  4. Pico W should appear as 'RPI-RP2' drive or USB Serial Device" -ForegroundColor White
        }
    }
    else {
        Write-Host "❌ Could not get device list" -ForegroundColor Red
        Write-Host "Make sure PlatformIO is installed" -ForegroundColor Yellow
    }
}
catch {
    Write-Host "❌ Error during device detection: $_" -ForegroundColor Red
}

Write-Host ""
Write-Host "=====================================" -ForegroundColor Cyan
