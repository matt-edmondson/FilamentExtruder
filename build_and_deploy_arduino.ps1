Param(
  [switch]$BuildOnly,
  [string]$Port,
  [int]$TimeoutSeconds = 60,
  [string]$CliPath,
  [string]$Fqbn = 'rp2040:rp2040:rpipicow:usbstack=picosdk',
  [string]$OutputDir = '.\\build',
  [string]$Sketch
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Write-Info([string]$msg) { Write-Host "[INFO] $msg" -ForegroundColor Cyan }
function Write-Warn([string]$msg) { Write-Host "[WARN] $msg" -ForegroundColor Yellow }
function Write-Err([string]$msg)  { Write-Host "[ERROR] $msg" -ForegroundColor Red }

function Resolve-ArduinoCliPath {
  if ($CliPath) {
    if (Test-Path $CliPath) { return (Resolve-Path $CliPath).Path }
    throw "Provided CliPath not found: $CliPath"
  }

  $found = (Get-Command arduino-cli -ErrorAction SilentlyContinue)?.Source
  if ($found) { return $found }

  $toolsDir = Join-Path $PSScriptRoot '.tools'
  $localCli = Join-Path $toolsDir 'arduino-cli\\arduino-cli.exe'
  if (Test-Path $localCli) { return $localCli }

  Write-Info 'arduino-cli not found. Downloading portable arduino-cli...'
  New-Item -ItemType Directory -Force -Path (Split-Path $localCli) | Out-Null

  $zipUrl = 'https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip'
  $zipPath = Join-Path $toolsDir 'arduino-cli_latest_Windows_64bit.zip'
  Invoke-WebRequest -Uri $zipUrl -OutFile $zipPath
  Expand-Archive -Path $zipPath -DestinationPath (Split-Path $localCli) -Force
  Remove-Item $zipPath -Force

  if (!(Test-Path $localCli)) { throw 'Failed to install arduino-cli' }
  return $localCli
}

function Ensure-Core-And-Libs([string]$cli) {
  $url = 'https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json'
  Write-Info 'Initializing Arduino CLI config'
  & $cli config init | Out-Null
  & $cli config set board_manager.additional_urls $url | Out-Null
  Write-Info 'Updating core index'
  & $cli core update-index | Out-Null
  Write-Info 'Installing RP2040 core if missing'
  & $cli core install rp2040:rp2040 | Out-Null

  $libs = @('Adafruit SSD1306','Adafruit GFX Library','Adafruit BusIO')
  foreach ($lib in $libs) {
    Write-Info "Ensuring library installed: $lib"
    & $cli lib install "$lib" | Out-Null
  }
}

function Get-SketchPath {
  if ($Sketch) {
    if (Test-Path $Sketch) { return (Resolve-Path $Sketch).Path }
    throw "Sketch not found: $Sketch"
  }

  $all = Get-ChildItem -Path $PSScriptRoot -Filter '*.ino' -File -Recurse
  if (-not $all) { throw 'No .ino sketch found in project.' }

  # Filter out vendor/build/example/library folders
  $filtered = $all | Where-Object {
    $p = $_.FullName
    return ($p -notmatch "\\\.pio\\") -and ($p -notmatch "\\\.tools\\") -and ($p -notmatch "\\examples\\") -and ($p -notmatch "\\libraries\\") -and ($p -notmatch "\\\.git\\")
  }
  if (-not $filtered) { $filtered = $all }

  # Strongly prefer FilamentExtruder.ino if present
  $named = $filtered | Where-Object { $_.Name -ieq 'FilamentExtruder.ino' } | Sort-Object { $_.DirectoryName -eq $PSScriptRoot } -Descending | Select-Object -First 1
  if ($named) { return $named.FullName }

  # Prefer root-level .ino
  $rootSketch = $filtered | Where-Object { $_.DirectoryName -eq $PSScriptRoot } | Select-Object -First 1
  if ($rootSketch) { return $rootSketch.FullName }

  # Otherwise prefer the shallowest path
  Push-Location $PSScriptRoot
  try {
    $ranked = $filtered | ForEach-Object {
      $rel = Resolve-Path $_.FullName -Relative
      [PSCustomObject]@{ Item = $_; Depth = ($rel -split '\\\\').Length }
    } | Sort-Object Depth, { $_.Item.Name }
    return $ranked[0].Item.FullName
  } finally { Pop-Location }
}

function Compile([string]$cli, [string]$sketch, [string]$fqbn, [string]$outDir) {
  Write-Info "Compiling $([IO.Path]::GetFileName($sketch)) for $fqbn"
  New-Item -ItemType Directory -Force -Path $outDir | Out-Null
  & $cli compile --fqbn $fqbn --output-dir $outDir --warnings all --verbose $sketch
}

function Find-UF2([string]$outDir) {
  $uf2 = Get-ChildItem -Path $outDir -Filter '*.uf2' -File -Recurse | Sort-Object LastWriteTime -Descending | Select-Object -First 1
  if (-not $uf2) { throw "No UF2 produced in '$outDir'" }
  return $uf2.FullName
}

function Find-RP2Drive {
  try {
    # Prefer CIM for broad compatibility
    $drv = Get-CimInstance Win32_LogicalDisk | Where-Object { $_.DriveType -eq 2 -or $_.DriveType -eq 3 } | Where-Object { $_.VolumeName -eq 'RPI-RP2' } | Select-Object -First 1
    if ($drv) { return ($drv.DeviceID + '\\') }
  } catch {}

  # Fallback: look for INFO_UF2.TXT on any removable drive
  foreach ($d in [System.IO.DriveInfo]::GetDrives()) {
    if ($d.DriveType -in @([IO.DriveType]::Removable,[IO.DriveType]::Fixed) -and $d.IsReady) {
      $info = Join-Path $d.RootDirectory.FullName 'INFO_UF2.TXT'
      if (Test-Path $info) { return $d.RootDirectory.FullName }
    }
  }
  return $null
}

function Upload-ViaMassStorage([string]$uf2, [int]$timeoutSec) {
  Write-Info 'Waiting for RPI-RP2 mass storage (hold BOOTSEL and plug in USB if needed)...'
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt $timeoutSec) {
    $root = Find-RP2Drive
    if ($root) {
      Write-Info "Found RPI-RP2 at $root"
      Copy-Item -Path $uf2 -Destination (Join-Path $root ([IO.Path]::GetFileName($uf2))) -Force
      Write-Info 'Copy complete.'
      return
    }
    Start-Sleep -Milliseconds 300
  }
  throw 'Timed out waiting for RPI-RP2 drive.'
}

function Upload-ViaSerial([string]$cli, [string]$sketch, [string]$fqbn, [string]$port) {
  Write-Info "Uploading via serial on $port"
  & $cli upload --fqbn $fqbn -p $port $sketch
}

try {
  $cli = Resolve-ArduinoCliPath
  Write-Info "Using arduino-cli: $cli"
  Ensure-Core-And-Libs -cli $cli

  $sketchPath = Get-SketchPath
  Compile -cli $cli -sketch $sketchPath -fqbn $Fqbn -outDir $OutputDir
  $uf2 = Find-UF2 -outDir $OutputDir
  Write-Info "UF2 ready: $uf2"

  if ($BuildOnly) { Write-Info 'BuildOnly specified. Skipping deploy.'; exit 0 }

  if ($Port) {
    try {
      Upload-ViaSerial -cli $cli -sketch $sketchPath -fqbn $Fqbn -port $Port
      exit 0
    } catch {
      Write-Warn "Serial upload failed: $($_.Exception.Message). Falling back to UF2 mass-storage copy."
    }
  }

  Upload-ViaMassStorage -uf2 $uf2 -timeoutSec $TimeoutSeconds
  Write-Info 'Done.'
  exit 0
} catch {
  Write-Err $_.Exception.Message
  exit 1
}


