# Starts Sifted without a terminal window (what the Sifted shortcut runs).
#
# 1. Starts the launcher inside WSL, hidden (build\launcher.log has its output).
# 2. Shows the operator view in a native window: WebView2 (the Edge engine that
#    ships with Windows) hosted in a WinForms window with Sifted's own icon, so
#    the taskbar icon is sharp at any display scaling. install.cmd downloads the
#    WebView2 SDK into build\webview2.
# 3. If that isn't available, falls back to an Edge/Chrome app window.
# Closing the window stops Sifted. Problems are shown in a message box.
#Requires -Version 5.1
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Build = Join-Path $Root "build"
$State = Join-Path $Build "launcher.state"
$Log = Join-Path $Build "launcher.log"
$Wv2 = Join-Path $Build "webview2"
$Icon = Join-Path $Root "app\assets\sifted.ico"
$AppId = "Sifted.OperatorView"   # also set on the shortcuts, so the taskbar groups them

function Show-Problem([string]$Text) {
    Add-Type -AssemblyName PresentationFramework
    [System.Windows.MessageBox]::Show($Text, "Sifted", "OK", "Error") | Out-Null
}

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    Show-Problem "Sifted needs the Windows Subsystem for Linux.`n`nDouble-click install.cmd in the Sifted folder first."
    exit 1
}

$useHost = (Test-Path (Join-Path $Wv2 "Microsoft.Web.WebView2.WinForms.dll")) -and
           (Test-Path (Join-Path $Wv2 "Microsoft.Web.WebView2.Core.dll")) -and
           (Test-Path (Join-Path $Wv2 "WebView2Loader.dll"))

# ------------------------------------------------------------------ 1. launcher
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
if ($useHost) { $wslArgs += "--no-browser" }   # this script opens the window
$proc = Start-Process -FilePath "wsl.exe" -ArgumentList $wslArgs -WindowStyle Hidden -PassThru

$url = $null
$deadline = (Get-Date).AddSeconds(45)
while ((Get-Date) -lt $deadline) {
    if (Test-Path $State) {
        $line = (Get-Content $State -Raw).Trim()
        if ($line -match "^running (http://\S+)$") { $url = $Matches[1]; break }
    }
    if ($proc.HasExited -and -not (Test-Path $State)) { Start-Sleep -Milliseconds 500; if (-not (Test-Path $State)) { break } }
    Start-Sleep -Milliseconds 300
}
if (-not $url) {
    $details = ""
    if (Test-Path $Log) { $details = ((Get-Content $Log -Tail 12) -join "`n").Trim() }
    if (-not $details -and $proc.HasExited) { $details = "WSL exited with code $($proc.ExitCode)." }
    if (-not $details) { $details = "It did not report back within 45 seconds." }
    Show-Problem ("Sifted could not start.`n`n$details`n`n" +
        "To see everything, run sifted.cmd in the Sifted folder. If it says Sifted isn't built, run install.cmd.")
    if (-not $proc.HasExited) { Stop-Process -Id $proc.Id -ErrorAction SilentlyContinue }
    exit 1
}
if (-not $useHost) { exit 0 }   # the launcher opened an Edge/Chrome app window itself

# ------------------------------------------------------------------ 2. native window
function Open-AppWindow([string]$Address) {
    # Fallback: Edge (always present on Windows 10/11) in app mode.
    try { Start-Process -FilePath "msedge.exe" -ArgumentList "--app=$Address", "--start-maximized" } catch { Start-Process $Address }
}

function Send-Bye([string]$Address) {
    # Tells the launcher this window closed; it stops a few seconds later unless
    # another Sifted window is still open.
    try {
        $wc = New-Object System.Net.WebClient
        $wc.Proxy = $null
        $wc.UploadString($Address + "api/bye", "POST", "") | Out-Null
    } catch { }
}

$script:hostFailed = $false
try {
    Add-Type -AssemblyName System.Windows.Forms, System.Drawing
    Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class SiftedNative {
    [DllImport("shell32.dll", CharSet = CharSet.Unicode)]
    public static extern int SetCurrentProcessExplicitAppUserModelID(string appId);
    [DllImport("user32.dll")]
    public static extern bool SetProcessDpiAwarenessContext(IntPtr value);
    [DllImport("dwmapi.dll")]
    public static extern int DwmSetWindowAttribute(IntPtr hwnd, int attribute, ref int value, int size);
}
"@
    # Sharp text and icons at any scaling (per-monitor DPI aware, v2 = -4); our own
    # taskbar identity instead of PowerShell's.
    [void][SiftedNative]::SetProcessDpiAwarenessContext([IntPtr](-4))
    [void][SiftedNative]::SetCurrentProcessExplicitAppUserModelID($AppId)

    $env:PATH = "$Wv2;$env:PATH"   # WebView2Loader.dll
    Add-Type -Path (Join-Path $Wv2 "Microsoft.Web.WebView2.Core.dll")
    Add-Type -Path (Join-Path $Wv2 "Microsoft.Web.WebView2.WinForms.dll")
    try { [Microsoft.Web.WebView2.Core.CoreWebView2Environment]::SetLoaderDllFolderPath($Wv2) } catch { }

    [System.Windows.Forms.Application]::EnableVisualStyles()
    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Sifted"
    if (Test-Path $Icon) { $form.Icon = New-Object System.Drawing.Icon($Icon) }
    $form.BackColor = [System.Drawing.Color]::FromArgb(11, 16, 23)
    $area = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
    $form.StartPosition = "Manual"
    $form.Width = [int]($area.Width * 0.85)
    $form.Height = [int]($area.Height * 0.88)
    $form.Left = $area.Left + [int](($area.Width - $form.Width) / 2)
    $form.Top = $area.Top + [int](($area.Height - $form.Height) / 2)
    # Open maximized (windowed fullscreen); the size above is what Restore returns to.
    $form.WindowState = [System.Windows.Forms.FormWindowState]::Maximized
    $form.Add_HandleCreated({
        $dark = 1   # dark title bar (Windows 10 20H1+ / 11)
        [void][SiftedNative]::DwmSetWindowAttribute($form.Handle, 20, [ref]$dark, 4)
    })

    $view = New-Object Microsoft.Web.WebView2.WinForms.WebView2
    $view.Dock = [System.Windows.Forms.DockStyle]::Fill
    try { $view.DefaultBackgroundColor = $form.BackColor } catch { }
    $props = New-Object Microsoft.Web.WebView2.WinForms.CoreWebView2CreationProperties
    $props.UserDataFolder = Join-Path $env:LOCALAPPDATA "Sifted\WebView2"
    $view.CreationProperties = $props
    $view.Add_CoreWebView2InitializationCompleted({
        param($sender, $e)
        if (-not $e.IsSuccess) {
            # Usually: the WebView2 runtime is missing. Fall back after the window closes.
            $script:hostFailed = $true
            $form.Close()
            return
        }
        # The in-app Quit button calls window.close(): close this window too.
        $view.CoreWebView2.add_WindowCloseRequested({ $form.Close() })
        $view.CoreWebView2.Settings.IsStatusBarEnabled = $false
    })
    $form.Controls.Add($view)
    $view.Source = [Uri]$url
    [System.Windows.Forms.Application]::Run($form)
} catch {
    $script:hostFailed = $true
}

if ($script:hostFailed) { Open-AppWindow $url } else { Send-Bye $url }
exit 0
