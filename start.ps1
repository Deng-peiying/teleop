param([switch]$Headless,[double]$Duration=0,
      [ValidateSet('thumb_track_b4')][string]$Method='thumb_track_b4')
$ErrorActionPreference='Stop'
$options=@('--method',$Method,'--display','command','--duration',"$Duration")
if($Headless){$options+='--headless'}
Push-Location -LiteralPath $PSScriptRoot
try {
    & wsl -d Ubuntu-22.04 --exec env PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u -m teleop.hand2_live @options
    $result=$LASTEXITCODE
} finally {
    Pop-Location
}
exit $result
