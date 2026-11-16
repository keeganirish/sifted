# Sifted installer for Windows. Run it through install.cmd (double-click).
#
# Sifted's fusion service uses POSIX networking, so on Windows it runs inside
# the Windows Subsystem for Linux (WSL). This script:
#   1. checks that WSL and a Linux distribution are present (and offers to
#      install them if not),
#   2. runs install.sh inside that distribution, which installs the build
#      tools, builds Sifted and checks it,
#   3. creates "Sifted" shortcuts on the desktop and in the Start menu.
# Safe to run again. Options: -Yes, -WithTests, -Distro <name>, -NoShortcut.
#Requires -Version 5.1
param(
    [switch]$Yes,
    [switch]$WithTests,
    [switch]$NoShortcut,
    [string]$Distro = ""
)
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

function Step($n, $text) { Write-Host ""; Write-Host "[$n/4] $text" -ForegroundColor Cyan }
function Ok($text)   { Write-Host "  + $text" -ForegroundColor Green }
function Info($text) { Write-Host "  - $text" -ForegroundColor DarkGray }
function Warn($text) { Write-Host "  ! $text" -ForegroundColor Yellow }
function Fail($text, $hint) {
    Write-Host ""
    Write-Host "  x $text" -ForegroundColor Red
    if ($hint) { Write-Host "    $hint" }
    Write-Host ""
    Write-Host "  See the Troubleshooting section of README.md, then run install.cmd again."
    exit 1
}
function Confirm($question) {
    if ($Yes) { return $true }
    $r = Read-Host "  $question [Y/n]"
    return ($r -eq "" -or $r -match "^[Yy]")
}
# wsl.exe writes UTF-16 unless WSL_UTF8 is set (newer WSL); handle both.
function Invoke-WslText([string[]]$WslArgs) {
    $ErrorActionPreference = "Continue"   # PowerShell 5.1 turns native stderr into errors
    $env:WSL_UTF8 = "1"
    $out = & wsl.exe @WslArgs 2>&1 | Out-String
    return ($out -replace "`0", "")
}

Write-Host "Sifted installer for Windows" -ForegroundColor White
Write-Host "  Folder: $Root" -ForegroundColor DarkGray

# --------------------------------------------------------------------------- 1. WSL
Step 1 "Checking the Windows Subsystem for Linux"
if ([Environment]::OSVersion.Platform -ne "Win32NT") {
    Fail "This installer is for Windows." "On Linux or macOS run ./install.sh instead."
}
$build = [Environment]::OSVersion.Version.Build
if ($build -lt 19041) {
    Fail "Windows 10 version 2004 (build 19041) or newer is needed for WSL; this is build $build." "Update Windows, then run install.cmd again."
}
Ok "Windows build $build"

$wsl = Get-Command wsl.exe -ErrorAction SilentlyContinue
$wslReady = $false
if ($wsl) {
    # "--status" succeeds once WSL is enabled (older builds: "--list" does if a distro exists).
    $null = Invoke-WslText @("--status")
    $wslReady = ($LASTEXITCODE -eq 0)
    if (-not $wslReady) { $null = Invoke-WslText @("--list", "--quiet"); $wslReady = ($LASTEXITCODE -eq 0) }
}
if (-not $wsl -or -not $wslReady) {
    Warn "WSL is not installed."
    if (-not (Confirm "Install WSL with Ubuntu now? (needs administrator rights and a restart)")) {
        Fail "WSL is required." "Install it yourself with 'wsl --install' in an administrator PowerShell, restart, and run install.cmd again."
    }
    try {
        Start-Process -FilePath "wsl.exe" -ArgumentList "--install", "-d", "Ubuntu" -Verb RunAs -Wait
    } catch {
        Fail "Could not start the WSL installer: $($_.Exception.Message)" "Run 'wsl --install' in an administrator PowerShell instead."
    }
    Write-Host ""
    Write-Host "  WSL is being installed. RESTART your computer, then:" -ForegroundColor Yellow
    Write-Host "    1. open 'Ubuntu' from the Start menu once and choose a Linux user name and password,"
    Write-Host "    2. run install.cmd again."
    exit 3   # action needed: install.cmd keeps the window open
}
Ok "WSL is available"

# Asks a distribution for its own name ($WSL_DISTRO_NAME), so what we store is
# exactly what "wsl.exe -d" accepts, whatever encoding "wsl --list" printed in.
function Get-DistroName([string]$Name) {
    $a = @()
    if ($Name -ne "") { $a += @("-d", $Name) }
    $a += @("-e", "printenv", "WSL_DISTRO_NAME")
    $out = (Invoke-WslText $a).Trim()
    if ($LASTEXITCODE -eq 0 -and $out -match "^[A-Za-z0-9._-]+$") { return $out }
    return $null
}
function Test-Usable([string]$Name) { return ($Name -and $Name -notmatch "^(docker|rancher)-desktop") }

$listed = @((Invoke-WslText @("--list", "--quiet")) -split "`r?`n" |
    ForEach-Object { $_ -replace "[^A-Za-z0-9._-]", "" } | Where-Object { $_ -ne "" })
$resolved = $null
if ($Distro -ne "") {
    $resolved = Get-DistroName $Distro
    if (-not $resolved) { Fail "WSL has no working distribution called '$Distro'." "Installed: $($listed -join ', ')" }
} else {
    $default = Get-DistroName ""
    if (Test-Usable $default) {
        $resolved = $default
    } else {
        foreach ($n in $listed) {
            if (Test-Usable $n) {
                $r = Get-DistroName $n
                if ($r) { $resolved = $r; break }
            }
        }
    }
}
if (-not $resolved) {
    if ($listed.Count -gt 0) {
        Warn "WSL has a distribution ($($listed -join ', ')), but it didn't start."
        Fail "Open Ubuntu from the Start menu once to finish its setup (it asks for a user name and password)." "Then run install.cmd again."
    }
    Warn "WSL has no Linux distribution yet."
    if (-not (Confirm "Install Ubuntu now?")) { Fail "A Linux distribution is required." "Run 'wsl --install -d Ubuntu', then install.cmd again." }
    & wsl.exe --install -d Ubuntu
    Write-Host ""
    Write-Host "  When Ubuntu has finished setting up (it asks for a user name and password)," -ForegroundColor Yellow
    Write-Host "  close its window and run install.cmd again."
    exit 3   # action needed: install.cmd keeps the window open
}
$Distro = $resolved
Ok "Using the '$Distro' distribution"

# --------------------------------------------------------------------------- 2. files
Step 2 "Preparing the project files"
# Shell scripts must have Unix line endings. A clone made with Git's
# core.autocrlf=true can convert them; .gitattributes prevents that for new
# clones, and this repairs older ones.
$fixed = 0
foreach ($rel in @("install.sh", "sifted", "scripts\demo.sh")) {
    $path = Join-Path $Root $rel
    if (-not (Test-Path $path)) { continue }
    $bytes = [IO.File]::ReadAllBytes($path)
    $text = [Text.Encoding]::UTF8.GetString($bytes)
    if ($text.Contains("`r`n")) {
        [IO.File]::WriteAllText($path, $text.Replace("`r`n", "`n"), (New-Object Text.UTF8Encoding($false)))
        $fixed++
    }
}
if ($fixed -gt 0) { Ok "Converted $fixed script(s) to Unix line endings" } else { Ok "Scripts are ready" }
New-Item -ItemType Directory -Force -Path (Join-Path $Root "build") | Out-Null
Set-Content -Path (Join-Path $Root "build\wsl-distro.txt") -Value $Distro -NoNewline -Encoding ASCII

# --------------------------------------------------------------------------- 3. install inside WSL
Step 3 "Installing and building inside Linux ($Distro)"
Info "This runs install.sh in WSL. Linux may ask for the password you chose for '$Distro'."
# Double-clicking the installer is the consent: no questions inside Linux.
$shArgs = @("-d", $Distro, "--cd", $Root, "--", "bash", "./install.sh", "--from-windows", "--yes")
if ($WithTests) { $shArgs += "--with-tests" }
Remove-Item Env:WSL_UTF8 -ErrorAction SilentlyContinue
& wsl.exe @shArgs
if ($LASTEXITCODE -ne 0) {
    Fail "The Linux part of the installation failed (exit code $LASTEXITCODE)." "The messages above say why; the full log is build\install.log."
}

# Start Sifted exactly the way the shortcut does (sifted.cmd), in check mode.
$env:SIFTED_NO_PAUSE = "1"
$p = Start-Process -FilePath (Join-Path $Root "sifted.cmd") -ArgumentList "--check" -WorkingDirectory $Root -NoNewWindow -Wait -PassThru
Remove-Item Env:SIFTED_NO_PAUSE -ErrorAction SilentlyContinue
if ($p.ExitCode -ne 0) {
    Fail "Sifted was built, but starting it from Windows failed (exit code $($p.ExitCode))." "The lines above show why."
}
Ok "Sifted starts from Windows"

# --------------------------------------------------------------------------- 4. shortcuts
Step 4 "Adding shortcuts"
if ($NoShortcut) {
    Info "Skipped"
} else {
    $shell = New-Object -ComObject WScript.Shell
    $targets = @(
        (Join-Path ([Environment]::GetFolderPath("Desktop")) "Sifted.lnk"),
        (Join-Path ([Environment]::GetFolderPath("Programs")) "Sifted.lnk")
    )
    foreach ($t in $targets) {
        $lnk = $shell.CreateShortcut($t)
        # PowerShell, hidden, running the starter: no terminal window stays open.
        $lnk.TargetPath = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"
        $lnk.Arguments = "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$Root\scripts\windows\start.ps1`""
        $lnk.WindowStyle = 7   # minimised, so nothing flashes while it hides itself
        $lnk.WorkingDirectory = $Root
        $lnk.Description = "Sifted - multi-sensor track fusion"
        $icon = Join-Path $Root "app\assets\sifted.ico"
        if (Test-Path $icon) { $lnk.IconLocation = $icon }
        $lnk.Save()
    }
    Ok "Desktop and Start menu shortcuts: Sifted"
}

Write-Host ""
Write-Host "Sifted is installed." -ForegroundColor Green
Write-Host "  Next time, start it from the Sifted shortcut on your desktop."
Write-Host "  Starting Sifted now; it opens in its own window. This window closes in 5 seconds."
Start-Process -FilePath "powershell.exe" -WindowStyle Hidden -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-WindowStyle", "Hidden", "-File", "`"$Root\scripts\windows\start.ps1`"")
Start-Sleep -Seconds 5
exit 0
