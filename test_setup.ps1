# Quick Setup Test Script
# This script verifies that all dependencies are properly installed

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host "  FilamentExtruder Setup Test" -ForegroundColor Cyan  
Write-Host "=====================================" -ForegroundColor Cyan
Write-Host ""

$allGood = $true

# Test 1: Check Python
Write-Host "Testing Python installation..." -ForegroundColor Yellow
$pythonFound = $false
$pythonCommands = @("python", "python3", "py")

foreach ($cmd in $pythonCommands) {
    try {
        $version = & $cmd --version 2>&1
        if ($version -match "Python (\d+)\.(\d+)") {
            $major = [int]$matches[1]
            $minor = [int]$matches[2]
            if ($major -gt 3 -or ($major -eq 3 -and $minor -ge 7)) {
                Write-Host "✓ Python found: $version" -ForegroundColor Green
                $pythonCmd = $cmd
                $pythonFound = $true
                break
            }
        }
    }
    catch {
        # Continue
    }
}

if (-not $pythonFound) {
    Write-Host "✗ Python 3.7+ not found" -ForegroundColor Red
    $allGood = $false
}

# Test 2: Check PlatformIO
Write-Host "Testing PlatformIO installation..." -ForegroundColor Yellow
$pioFound = $false

# Try direct pio command
try {
    $version = pio --version 2>&1
    Write-Host "✓ PlatformIO found: $version" -ForegroundColor Green
    $pioFound = $true
}
catch {
    # Try Python module
    if ($pythonFound) {
        try {
            $version = & $pythonCmd -m platformio --version 2>&1
            if ($version -and $version -notmatch "No module named") {
                Write-Host "✓ PlatformIO found via Python module: $version" -ForegroundColor Green
                $pioFound = $true
            }
        }
        catch {
            # Not found
        }
    }
}

if (-not $pioFound) {
    Write-Host "✗ PlatformIO not found" -ForegroundColor Red
    $allGood = $false
}

# Test 3: Check project structure
Write-Host "Testing project structure..." -ForegroundColor Yellow
if (Test-Path "platformio.ini") {
    Write-Host "✓ platformio.ini found" -ForegroundColor Green
} else {
    Write-Host "✗ platformio.ini missing" -ForegroundColor Red
    $allGood = $false
}

if (Test-Path "src\main.cpp") {
    Write-Host "✓ src\main.cpp found" -ForegroundColor Green  
} else {
    Write-Host "✗ src\main.cpp missing" -ForegroundColor Red
    $allGood = $false
}

# Test 4: Try a basic PlatformIO command
if ($pioFound) {
    Write-Host "Testing PlatformIO functionality..." -ForegroundColor Yellow
    try {
        if ($pioFound -and (Get-Command "pio" -ErrorAction SilentlyContinue)) {
            $null = pio platform list 2>&1
        } else {
            $null = & $pythonCmd -m platformio platform list 2>&1
        }
        Write-Host "✓ PlatformIO is functional" -ForegroundColor Green
    }
    catch {
        Write-Host "✗ PlatformIO command failed: $_" -ForegroundColor Red
        $allGood = $false
    }
}

# Summary
Write-Host ""
Write-Host "=====================================" -ForegroundColor Cyan
if ($allGood) {
    Write-Host "  ALL TESTS PASSED!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Your system is ready to build and deploy." -ForegroundColor Green
    Write-Host "Run: .\build_and_deploy.ps1" -ForegroundColor Cyan
} else {
    Write-Host "  SOME TESTS FAILED" -ForegroundColor Red  
    Write-Host ""
    Write-Host "Please fix the issues above before proceeding." -ForegroundColor Yellow
    Write-Host "You can try running: .\build_and_deploy.ps1" -ForegroundColor Cyan
    Write-Host "The build script will attempt to fix missing dependencies." -ForegroundColor Yellow
}
Write-Host "=====================================" -ForegroundColor Cyan
