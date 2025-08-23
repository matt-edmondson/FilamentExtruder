# PlatformIO Package Repair Script
# Use this if you're experiencing MissingPackageManifestError or other package issues

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "     PlatformIO Package Repair" -ForegroundColor Cyan  
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "platformio.ini")) {
    Write-Host "❌ Error: platformio.ini not found in current directory" -ForegroundColor Red
    Write-Host "Please run this script from the project root directory" -ForegroundColor Red
    pause
    exit 1
}

Write-Host "✓ Found platformio.ini - we're in the right directory" -ForegroundColor Green
Write-Host ""

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
Write-Host "This script will:" -ForegroundColor Yellow
Write-Host "  1. Clean current build files" -ForegroundColor White
Write-Host "  2. Uninstall raspberrypi platform" -ForegroundColor White
Write-Host "  3. Reinstall raspberrypi platform" -ForegroundColor White
Write-Host "  4. Update libraries" -ForegroundColor White
Write-Host "  5. Attempt a test build" -ForegroundColor White
Write-Host ""

$continue = Read-Host "Continue with repair? (y/n)"
if ($continue -ne "y" -and $continue -ne "Y") {
    Write-Host "Repair cancelled" -ForegroundColor Yellow
    pause
    exit 0
}

Write-Host ""
Write-Host "Starting repair process..." -ForegroundColor Cyan

try {
    # Step 1: Clean
    Write-Host ""
    Write-Host "1. Cleaning build files..." -ForegroundColor Yellow
    & $pythonCmd -m platformio run --target clean
    
    # Step 2: Uninstall platform
    Write-Host ""
    Write-Host "2. Uninstalling raspberrypi platform..." -ForegroundColor Yellow
    & $pythonCmd -m platformio platform uninstall raspberrypi 2>&1 | Out-Null
    Write-Host "   Platform removed" -ForegroundColor Gray
    
    # Step 3: Reinstall platform
    Write-Host ""
    Write-Host "3. Reinstalling raspberrypi platform..." -ForegroundColor Yellow
    & $pythonCmd -m platformio platform install raspberrypi
    
    # Step 4: Update libraries
    Write-Host ""
    Write-Host "4. Updating libraries..." -ForegroundColor Yellow
    & $pythonCmd -m platformio lib update
    
    # Step 5: Test build
    Write-Host ""
    Write-Host "5. Testing build..." -ForegroundColor Yellow
    $buildOutput = & $pythonCmd -m platformio run 2>&1
    $buildExitCode = $LASTEXITCODE
    
    Write-Host ""
    if ($buildExitCode -eq 0) {
        Write-Host "🎉 Repair successful!" -ForegroundColor Green
        Write-Host "Your project should now build correctly." -ForegroundColor Green
        Write-Host ""
        Write-Host "You can now run:" -ForegroundColor Cyan
        Write-Host "  .\build_and_deploy.ps1" -ForegroundColor White
    }
    else {
        Write-Host "⚠️  Repair completed but build still has issues" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "Build output:" -ForegroundColor Gray
        Write-Host $buildOutput -ForegroundColor Gray
        Write-Host ""
        Write-Host "Additional steps to try:" -ForegroundColor Yellow
        Write-Host "  1. Delete the .pio folder completely" -ForegroundColor White
        Write-Host "  2. Restart PowerShell" -ForegroundColor White
        Write-Host "  3. Run: python -m platformio platform install raspberrypi --force" -ForegroundColor White
    }
}
catch {
    Write-Host ""
    Write-Host "❌ Repair failed: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Manual recovery steps:" -ForegroundColor Yellow
    Write-Host "  1. Close PowerShell completely" -ForegroundColor White
    Write-Host "  2. Delete the .pio folder in your project directory" -ForegroundColor White
    Write-Host "  3. Open new PowerShell window" -ForegroundColor White
    Write-Host "  4. Run: python -m pip install --upgrade platformio" -ForegroundColor White
    Write-Host "  5. Try building again" -ForegroundColor White
}

Write-Host ""
Write-Host "=====================================" -ForegroundColor Cyan
pause
