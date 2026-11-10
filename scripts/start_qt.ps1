$ErrorActionPreference = 'Stop'
try {
    $usbipd = 'C:\Program Files\usbipd-win\usbipd.exe'
    $devices = & $usbipd list
    $adapter = @($devices | Where-Object { $_ -match '0c72:000c' })
    if ($adapter.Count -ne 1) {
        throw 'Connect one XCAN-USB adapter, then try again.'
    }
    if ($adapter[0] -notmatch 'Attached') {
        $busId = ($adapter[0].Trim() -split '\s+')[0]
        & $usbipd attach --wsl --busid $busId
        if ($LASTEXITCODE -ne 0) { throw 'USB attachment failed. Check usbipd list and device sharing.' }
    }
    $projectPath = (Split-Path -Parent $PSScriptRoot).Replace('\', '/')
    $linuxPath = '/mnt/' + $projectPath.Substring(0, 1).ToLower() + $projectPath.Substring(2)
    & wsl.exe -d Ubuntu-22.04 -u root --cd $linuxPath bash scripts/start_qt.sh
    if ($LASTEXITCODE -ne 0) { throw 'Qt exited with an error. See the output above.' }
} catch {
    Write-Host $_ -ForegroundColor Red
    Read-Host 'Press Enter to close'
}
