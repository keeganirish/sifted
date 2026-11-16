# Starts Sifted without a terminal window (what the Sifted shortcut runs).
#
# The launcher runs inside WSL, hidden; it opens the operator view in its own
# app window (Chrome or Edge "app mode": no tabs or address bar) and stops when
# that window is closed. If it can't start, this shows a message box with the
# reason instead of failing silently. For a visible terminal, run sifted.cmd.
#Requires -Version 5.1
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Build = Join-Path $Root "build"
$State = Join-Path $Build "launcher.state"
$Log = Join-Path $Build "launcher.log"

function Show-Problem([string]$Text) {
    Add-Type -AssemblyName PresentationFramework
    [System.Windows.MessageBox]::Show($Text, "Sifted", "OK", "Error") | Out-Null
}

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    Show-Problem "Sifted needs the Windows Subsystem for Linux.`n`nDouble-click install.cmd in the Sifted folder first."
    exit 1
}

New-Item -ItemType Directory -Force -Path $Build | Out-Null
Remove-Item $State -ErrorAction SilentlyContinue

# Start-Process joins arguments with spaces, so quote the folder ourselves.
$wslArgs = @()
$distroFile = Join-Path $Build "wsl-distro.txt"
if (Test-Path $distroFile) {
    $distro = (Get-Content $distroFile -Raw).Trim()
    if ($distro -match "^[A-Za-z0-9._-]+$") { $wslArgs += @("-d", $distro) }
}
$wslArgs += @("--cd", "`"$Root`"", "--", "bash", "./sifted", "--background")
$proc = Start-Process -FilePath "wsl.exe" -ArgumentList $wslArgs -WindowStyle Hidden -PassThru

# Wait until the launcher says it is up (it writes build\launcher.state), or fails.
$deadline = (Get-Date).AddSeconds(45)
while ((Get-Date) -lt $deadline) {
    if (Test-Path $State) { exit 0 }
    if ($proc.HasExited) {
        Start-Sleep -Milliseconds 500
        if (Test-Path $State) { exit 0 }
        break
    }
    Start-Sleep -Milliseconds 300
}

$details = ""
if (Test-Path $Log) { $details = ((Get-Content $Log -Tail 12) -join "`n").Trim() }
if (-not $details -and $proc.HasExited) { $details = "WSL exited with code $($proc.ExitCode)." }
if (-not $details) { $details = "It did not report back within 45 seconds." }
Show-Problem ("Sifted could not start.`n`n$details`n`n" +
    "To see everything, run sifted.cmd in the Sifted folder. If it says Sifted isn't built, run install.cmd.")
if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -ErrorAction SilentlyContinue }
exit 1
