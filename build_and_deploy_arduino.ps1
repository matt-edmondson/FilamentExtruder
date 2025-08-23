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
  Write-Info 'Ensuring RP2040 core is installed'
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

function Get-SerialPortList {
  $ports = @()
  try {
    $cim = Get-CimInstance Win32_SerialPort -ErrorAction Stop | ForEach-Object { $_.DeviceID }
    if ($cim) { $ports += $cim }
  } catch {}
  try {
    $dotnet = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($dotnet) { $ports += $dotnet }
  } catch {}
  return ($ports | Sort-Object -Unique)
}

function Wait-ForSerialPort([string[]]$baseline, [string]$preferredPort, [int]$timeoutSec) {
  $deadline = (Get-Date).AddSeconds($timeoutSec)
  while ((Get-Date) -lt $deadline) {
    $now = Get-SerialPortList
    if ($preferredPort) {
      if ($now -contains $preferredPort) { return $preferredPort }
    } else {
      $newPorts = $now | Where-Object { $baseline -notcontains $_ }
      if ($newPorts) { return ($newPorts | Select-Object -First 1) }
    }
    Start-Sleep -Milliseconds 300
  }
  return $null
}

function Start-SerialMonitor([string]$cli, [string]$port, [int]$baud) {
  Write-Info "Starting serial monitor on $port @ ${baud} baud (Ctrl+C to exit)"
  & $cli monitor -p $port -c "baud=$baud"
}

function Find-SerialPortViaCli([string]$cli, [string]$fqbn) {
  try {
    $out = & $cli board list --format json 2>$null | Out-String
    if (-not $out) { return $null }
    $data = $out | ConvertFrom-Json
    $ports = @()
    if ($data -and $data.ports) { $ports = $data.ports } elseif ($data) { $ports = $data }
    foreach ($p in $ports) {
      if ($p.protocol -ne 'serial') { continue }
      $addr = $p.address
      if (-not $addr) { $addr = $p.Address }
      $boards = $p.boards
      if (-not $boards) { $boards = $p.Boards }
      if ($boards) {
        foreach ($b in $boards) {
          $bfqbn = $b.fqbn; if (-not $bfqbn) { $bfqbn = $b.FQBN }
          $bname = $b.name; if (-not $bname) { $bname = $b.Name }
          if ($bfqbn -eq $fqbn -or ($bname -and ($bname -match 'Pico'))) {
            if ($addr) { return $addr }
          }
        }
      }
    }
  } catch {}
  return $null
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

  $baselinePorts = Get-SerialPortList

  if ($Port) {
    try {
      Upload-ViaSerial -cli $cli -sketch $sketchPath -fqbn $Fqbn -port $Port
      if (-not $NoMonitor) {
        $monitorPort = Wait-ForSerialPort -baseline $baselinePorts -preferredPort $Port -timeoutSec $MonitorTimeoutSeconds
        if (-not $monitorPort) { Write-Warn "Could not detect serial port for monitor; trying provided port $Port"; $monitorPort = $Port }
        Start-SerialMonitor -cli $cli -port $monitorPort -baud $Baud
      }
      exit 0
    } catch {
      Write-Warn "Serial upload failed: $($_.Exception.Message). Falling back to UF2 mass-storage copy."
    }
  }

  # Auto-detect serial port for Pico W and prefer uploading over serial
  $autoPort = Find-SerialPortViaCli -cli $cli -fqbn $Fqbn
  if (-not $autoPort) {
    $portsNow = Get-SerialPortList
    if ($portsNow.Count -eq 1) { $autoPort = $portsNow[0] }
  }
  if ($autoPort) {
    Write-Info "Detected Pico serial port: $autoPort (attempting serial upload)"
    try {
      Upload-ViaSerial -cli $cli -sketch $sketchPath -fqbn $Fqbn -port $autoPort
      if (-not $NoMonitor) {
        $monitorPort = Wait-ForSerialPort -baseline $baselinePorts -preferredPort $autoPort -timeoutSec $MonitorTimeoutSeconds
        if (-not $monitorPort) { $monitorPort = $autoPort }
        Start-SerialMonitor -cli $cli -port $monitorPort -baud $Baud
      }
      exit 0
    } catch {
      Write-Warn "Auto serial upload failed: $($_.Exception.Message). Falling back to UF2 mass-storage copy."
    }
  }

  Upload-ViaMassStorage -uf2 $uf2 -timeoutSec $TimeoutSeconds
  if (-not $NoMonitor) {
    Write-Info 'Waiting for device to enumerate as a serial port...'
    $monitorPort = Wait-ForSerialPort -baseline $baselinePorts -preferredPort $null -timeoutSec $MonitorTimeoutSeconds
    if ($monitorPort) {
      Start-SerialMonitor -cli $cli -port $monitorPort -baud $Baud
    } else {
      Write-Warn 'No new serial port detected after deployment. Connect manually if needed.'
    }
  }
  Write-Info 'Done.'
  exit 0
} catch {
  Write-Err $_.Exception.Message
  exit 1
}


