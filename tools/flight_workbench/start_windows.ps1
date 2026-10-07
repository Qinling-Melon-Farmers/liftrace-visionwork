param(
    [ValidateSet('ssh', 'local')][string]$Transport = 'ssh',
    [ValidateRange(1, 65535)][int]$Port = 8771,
    [string]$ProfileDir = '',
    [string]$Python = '',
    [switch]$LogsOnly,
    [switch]$Open
)
$ErrorActionPreference = 'Stop'
# Native Python only; no installation, SSH connection or device startup here.
if (-not $Python) { $Python = $env:WORKBENCH_PYTHON }
if (-not $Python) {
    $candidate = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($candidate -and $candidate.Source -notlike '*WindowsApps*') { $Python = $candidate.Source }
    elseif (Get-Command py.exe -ErrorAction SilentlyContinue) { $Python = 'py.exe' }
    else { throw 'Python 3.9+ required. Set WORKBENCH_PYTHON or pass -Python.' }
}
& $Python -c "import sys,yaml,pexpect,paramiko; assert sys.version_info >= (3,9); print('Native Python:', sys.executable)"
if ($LASTEXITCODE -ne 0) { throw 'Use an existing Python 3.9+ environment with PyYAML, pexpect and Paramiko. Nothing was installed.' }
$backendArgs = @((Join-Path $PSScriptRoot 'server.py'), '--host', '127.0.0.1', '--port', [string]$Port, '--transport', $Transport)
if ($ProfileDir) { $backendArgs += @('--profile-dir', $ProfileDir) }
if ($LogsOnly) { $backendArgs += '--logs-only' }
if ($Open) { $backendArgs += '--open' }
Write-Host 'Use the actual URL printed by the backend (port may change if occupied). Ctrl+C to stop.'
& $Python @backendArgs
exit $LASTEXITCODE
