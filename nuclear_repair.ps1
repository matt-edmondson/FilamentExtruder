# Nuclear PlatformIO Repair Script
# Use this when standard repair fails due to deep package corruption

Write-Host "=====================================" -ForegroundColor Red
Write-Host "     NUCLEAR PLATFORMIO REPAIR" -ForegroundColor Red  
Write-Host "=====================================" -ForegroundColor Red
Write-Host ""

Write-Host "⚠️  WARNING: This will completely reset PlatformIO!" -ForegroundColor Yellow
Write-Host "This script will:" -ForegroundColor Yellow
Write-Host "  1. Delete entire .pio folder (all cached packages)" -ForegroundColor White
Write-Host "  2. Delete PlatformIO user directory" -ForegroundColor White
Write-Host "  3. Reinstall PlatformIO completely" -ForegroundColor White
Write-Host "  4. Force reinstall all packages" -ForegroundColor White
Write-Host ""

$continue = Read-Host "Continue with nuclear repair? (type 'NUCLEAR' to proceed)"
if ($continue -ne "NUCLEAR") {
    Write-Host "Nuclear repair cancelled" -ForegroundColor Yellow
    pause
    exit 0
}

# Find Python command
$pythonCmd = $null
$pythonCommands = @("python", "python3", "py")

foreach ($cmd in $pythonCommands) {
    try {
        $version = & $cmd --version 2>&1
        if ($version -match "Python \d+\.\d+") {
            $pythonCmd = $cmd
            Write-Host "✓ Found Python: $version" -ForegroundColor Green
            break
        }
    }
    catch {
        continue
    }
}

if (-not $pythonCmd) {
    Write-Host "❌ Python not found" -ForegroundColor Red
    pause
    exit 1
}

Write-Host ""
Write-Host "🚀 Starting nuclear repair..." -ForegroundColor Red

try {
    # Step 1: Delete local .pio folder
    Write-Host ""
    Write-Host "1. Deleting local .pio folder..." -ForegroundColor Yellow
    if (Test-Path ".pio") {
        Remove-Item ".pio" -Recurse -Force
        Write-Host "   ✓ Local .pio folder deleted" -ForegroundColor Green
    } else {
        Write-Host "   ✓ No local .pio folder found" -ForegroundColor Green
    }
    
    # Step 2: Delete PlatformIO user directory
    Write-Host ""
    Write-Host "2. Finding PlatformIO user directory..." -ForegroundColor Yellow
    
    $pioHome = $env:PLATFORMIO_HOME
    if (-not $pioHome) {
        # Default locations
        $possibleHomes = @(
            "$env:USERPROFILE\.platformio",
            "$env:USERPROFILE\Documents\PlatformIO"
        )
        
        foreach ($path in $possibleHomes) {
            if (Test-Path $path) {
                $pioHome = $path
                break
            }
        }
    }
    
    if ($pioHome -and (Test-Path $pioHome)) {
        Write-Host "   Found PlatformIO directory: $pioHome" -ForegroundColor Gray
        Write-Host "   Deleting PlatformIO user directory..." -ForegroundColor Yellow
        Remove-Item $pioHome -Recurse -Force
        Write-Host "   ✓ PlatformIO user directory deleted" -ForegroundColor Green
    } else {
        Write-Host "   ✓ No PlatformIO user directory found" -ForegroundColor Green
    }
    
    # Step 3: Clear pip cache
    Write-Host ""
    Write-Host "3. Clearing pip cache..." -ForegroundColor Yellow
    & $pythonCmd -m pip cache purge 2>&1 | Out-Null
    Write-Host "   ✓ Pip cache cleared" -ForegroundColor Green
    
    # Step 4: Reinstall PlatformIO
    Write-Host ""
    Write-Host "4. Reinstalling PlatformIO..." -ForegroundColor Yellow
    & $pythonCmd -m pip uninstall platformio -y 2>&1 | Out-Null
    & $pythonCmd -m pip install platformio
    Write-Host "   ✓ PlatformIO reinstalled" -ForegroundColor Green
    
    # Step 5: Force install raspberrypi platform
    Write-Host ""
    Write-Host "5. Force installing raspberrypi platform..." -ForegroundColor Yellow
    & $pythonCmd -m platformio platform install raspberrypi --force
    Write-Host "   ✓ Raspberrypi platform installed" -ForegroundColor Green
    
    # Step 6: Install project dependencies
    Write-Host ""
    Write-Host "6. Installing project dependencies..." -ForegroundColor Yellow
    & $pythonCmd -m platformio lib install
    Write-Host "   ✓ Dependencies installed" -ForegroundColor Green
    
    # Step 7: Test build
    Write-Host ""
    Write-Host "7. Testing build..." -ForegroundColor Yellow
    $buildOutput = & $pythonCmd -m platformio run 2>&1
    $buildExitCode = $LASTEXITCODE
    
    Write-Host ""
    if ($buildExitCode -eq 0) {
        Write-Host "🎉 NUCLEAR REPAIR SUCCESSFUL!" -ForegroundColor Green
        Write-Host "Your project is now ready to build and deploy!" -ForegroundColor Green
        Write-Host ""
        Write-Host "You can now run:" -ForegroundColor Cyan
        Write-Host "  .\build_and_deploy.ps1" -ForegroundColor White
    } else {
        Write-Host "⚠️  Nuclear repair completed but build still fails" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "This may indicate a deeper system issue." -ForegroundColor Yellow
        Write-Host "Build output:" -ForegroundColor Gray
        Write-Host $buildOutput -ForegroundColor Gray
        Write-Host ""
        Write-Host "Final troubleshooting steps:" -ForegroundColor Red
        Write-Host "  1. Restart your computer" -ForegroundColor White
        Write-Host "  2. Try a different Python installation" -ForegroundColor White
        Write-Host "  3. Install PlatformIO IDE as alternative" -ForegroundColor White
    }
}
catch {
    Write-Host ""
    Write-Host "❌ Nuclear repair failed: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "This suggests a deeper system issue." -ForegroundColor Red
    Write-Host "Consider:" -ForegroundColor Yellow
    Write-Host "  1. Restarting your computer" -ForegroundColor White
    Write-Host "  2. Running PowerShell as Administrator" -ForegroundColor White
    Write-Host "  3. Using PlatformIO IDE instead of command line" -ForegroundColor White
}

Write-Host ""
Write-Host "=====================================" -ForegroundColor Red
pause
