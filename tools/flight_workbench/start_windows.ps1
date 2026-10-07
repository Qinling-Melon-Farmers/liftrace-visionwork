param(
    [ValidateSet('ssh', 'local')][string]$Transport = 'ssh',
    [ValidateRange(1024, 65535)][int]$Port = 8791,
    [string]$ProfileDir = ''
)
$ErrorActionPreference = 'Stop'
if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw 'WSL is required. See README_FIRST.md; native Windows Python is not supported.'
}
# Encode the script as UTF-8 before crossing Windows -> WSL argv (paths may be Chinese).
$quotedPath = "'" + $PSScriptRoot.Replace("'", "'\''") + "'"
$quotedProfile = "'" + $ProfileDir.Replace("'", "'\''") + "'"
$script = @'
set -euo pipefail
package_windows_path=__PACKAGE_PATH__
package_dir=$(wslpath -a "$package_windows_path")
cd "$package_dir"
if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  conda activate rl_drone
fi
profile_dir=__PROFILE_DIR__
profile_args=()
if [[ -n "$profile_dir" ]]; then
  profile_args=(--profile-dir "$profile_dir")
fi
exec bash ./start_workbench.sh --host 127.0.0.1 --port __PORT__ --transport __TRANSPORT__ "${profile_args[@]}"
'@
$script = $script.Replace('__PACKAGE_PATH__', $quotedPath).Replace('__PORT__', [string]$Port).Replace('__TRANSPORT__', $Transport).Replace('__PROFILE_DIR__', $quotedProfile)
$encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($script))
Write-Host "Workbench: http://127.0.0.1:$Port/  (keep this window open; Ctrl+C to stop)"
# No user text is interpolated as shell code in this ASCII-only command.
wsl -e bash -c "printf %s $encoded | base64 -d | bash"
exit $LASTEXITCODE
