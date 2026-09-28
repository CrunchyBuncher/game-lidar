<#
.SYNOPSIS
End-to-end tests: the ReShade addon capturing the fake game, checked with lidar_verify.

.DESCRIPTION
There is one test flow ($Cases below). A graphics API is only an environment ($Environments in
e2e_common.ps1, shared with run_viewer_e2e.ps1): which rig folder ReShade is installed in, which
--api the fake game renders with, what the reference needs to match it and how close its depth
can get. Every case runs in every selected environment
and depth mode. With more than one environment, the frames each one captured of the same frozen
scene are then compared with each other, so the APIs must also agree among themselves.

Needs a Release build (lidar_verify, lidar_capture, fake_game) and ReShade in sandbox\fake_game\
as dxgi.dll. The D3D9 rig gets the same DLL as d3d9.dll if it has none. Close any game with the
addon first: everything here shares the frame ring.

The fake games open behind other windows (--background), so they don't take the focus.

.EXAMPLE
tests\e2e\run_e2e.ps1                          # every API, then compare them
.EXAMPLE
tests\e2e\run_e2e.ps1 -Depth all               # every case in every depth mode
.EXAMPLE
tests\e2e\run_e2e.ps1 -Api d3d12 -Case pose -Depth reversed
#>
param(
    [string[]]$Api = 'all',       # all | d3d11 | d3d12 | d3d9, several comma-separated
    [string[]]$Case = '*',        # names, wildcards allowed
    [string[]]$Depth = 'default', # default (each case's Depths) | all | standard, reversed, reversed-infinite
    [int]$Frames = 60,
    [string]$BuildDir = 'build'
)
$ErrorActionPreference = 'Stop'
# Lists arrive as one "a,b" string through powershell -File.
$Api, $Case, $Depth = ($Api, $Case, $Depth) | ForEach-Object { , @($_ -split ',' | ForEach-Object Trim | Where-Object { $_ }) }
. (Join-Path $PSScriptRoot 'e2e_common.ps1')  # $Environments, $Tolerance, the helpers

# ---- The test flow: written once, run in every environment ---------------------------------------
# Check: 'ring'  - the addon's frames carry its pose and land on the true scene (lidar_verify ring).
#        'addon' - frozen camera, pixel for pixel against an un-injected D3D11 reference
#                  (lidar_verify addon). The frame is kept for the cross-API comparison.
# Profile: the camera profile the addon loads (next to the rig's exe). Args: extra fake_game flags.
# Apis: only for the environments listed (a feature one API has); others report SKIP.
# Depths: the depth modes it runs in by default (all three if not given); -Depth all runs every mode.
# The API-only cases exercise a binding or draw path, which doesn't depend on the depth mode.
$AllDepths = @('standard', 'reversed', 'reversed-infinite')
$Cases = @(
    @{ Name = 'pose';       Check = 'ring';  Profile = 'lidar_profile.toml' }
    @{ Name = 'exact';      Check = 'addon'; Profile = 'lidar_profile.toml' }
    @{ Name = 'viewproj';   Check = 'ring';  Profile = 'fake_game_viewproj.toml' }
    @{ Name = 'cbv-tables'; Check = 'ring';  Profile = 'lidar_profile.toml'; Args = @('--cbv-tables'); Apis = @('d3d12')
       Depths = @('reversed') }
    @{ Name = 'own-depth';  Check = 'ring';  Profile = 'lidar_profile.toml'; Args = @('--own-depth'); Apis = @('d3d9')
       Depths = @('reversed') }
    @{ Name = 'draw-up';    Check = 'ring';  Profile = 'lidar_profile.toml'; Args = @('--up'); Apis = @('d3d9')
       Depths = @('reversed') }
    @{ Name = 'modelview';  Check = 'ring';  Profile = 'fake_game_modelview.toml'
       Args = @('--camera-layout', 'modelview'); Apis = @('d3d9'); Depths = @('reversed') }
)

$FreezeAt = 3  # path time for the frozen cases: every environment captures this same view

# ---- Runner --------------------------------------------------------------------------------------
$logs = Join-Path $work 'logs'
$dumps = Join-Path $work 'frames'
Initialize-Suite $logs @($verify, (Join-Path $root 'sandbox\fake_game\fake_game.exe'))
Remove-Item -Recurse -Force $dumps -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $dumps | Out-Null

# The reference runs un-injected, so it must live outside sandbox\ (ReShade would hook it there).
$refDir = Join-Path $work 'reference'
New-Item -ItemType Directory -Force $refDir | Out-Null
Copy-Item (Join-Path $root 'sandbox\fake_game\fake_game.exe') $refDir -Force
$refExe = Join-Path $refDir 'fake_game.exe'

foreach ($d in $Depth) { if ($d -notin @('default', 'all') + $AllDepths) { throw "unknown -Depth $d" } }
function Get-CaseDepths($c) {
    if ($Depth -contains 'all') { return $AllDepths }
    if ($Depth -contains 'default') { if ($c.Depths) { return $c.Depths } else { return $AllDepths } }
    return $Depth
}
$apis = Select-Environments $Api
$cases = @($Cases | Where-Object { $n = $_.Name; @($Case | Where-Object { $n -like $_ }).Count -gt 0 })
if ($cases.Count -eq 0) { throw "no case matches '$Case' (cases: $($Cases.Name -join ', '))" }

try {
    foreach ($apiName in $apis) {
        $e = $Environments[$apiName]
        $rigProblem = Test-Rig $apiName $e
        $rig = Join-Path $root $e.Rig
        foreach ($c in $cases) {
            if ($c.Apis -and $c.Apis -notcontains $apiName) { Add-Result $apiName $c.Name '-' 'SKIP' "only on $($c.Apis -join ', ')"; continue }
            if ($rigProblem) { Add-Result $apiName $c.Name '-' 'SKIP' $rigProblem; continue }
            foreach ($d in Get-CaseDepths $c) {

                Set-RigProfile $rig $c.Profile
                $frozen = $c.Check -eq 'addon'
                $gameArgs = @($e.Args) + @($c.Args | Where-Object { $_ }) + @('--no-npc', '--no-publish', '--depth', $d, '--duration', '120')
                if ($frozen) { $gameArgs += @('--freeze', "$FreezeAt") }
                $verifyArgs = @($c.Check, '--frames', "$Frames", '--tol', "$Tolerance")
                if ($frozen) {
                    $dump = Join-Path $dumps "$($c.Name)-$d-$apiName.frame"
                    $verifyArgs += @('--depth-tol', "$($e.DepthTol)", '--dump', $dump)
                }

                $log = Join-Path $logs "$apiName-$($c.Name)-$d.log"
                $procs = @()
                try {
                    if ($frozen) {
                        $refArgs = @('--no-npc', '--depth', $d, '--freeze', "$FreezeAt", '--ring', 'Local\game_lidar_ref', '--duration', '120') + $e.Reference
                        $procs += Start-Game $refExe $refArgs
                    }
                    $procs += Start-Game (Join-Path $rig 'fake_game.exe') $gameArgs
                    "fake_game $($gameArgs -join ' ')`nlidar_verify $($verifyArgs -join ' ')`n" | Set-Content $log
                    & $verify @verifyArgs | Add-Content $log
                    $code = $LASTEXITCODE
                } finally {
                    Stop-Games $procs
                }
                Add-Result $apiName $c.Name $d ($(if ($code -eq 0) { 'PASS' } else { 'FAIL' })) (Get-Verdict $log)
            }
        }
    }

    # Cross-API: every frozen case each environment passed, compared against the first environment.
    if ($apis.Count -gt 1) {
        foreach ($c in $cases | Where-Object { $_.Check -eq 'addon' }) {
            foreach ($d in Get-CaseDepths $c) {
                $base = $apis[0]
                foreach ($other in $apis | Select-Object -Skip 1) {
                    $pair = "$base~$other"
                    $a = Join-Path $dumps "$($c.Name)-$d-$base.frame"
                    $b = Join-Path $dumps "$($c.Name)-$d-$other.frame"
                    if (-not (Test-Path $a) -or -not (Test-Path $b)) { Add-Result $pair $c.Name $d 'SKIP' 'a frame is missing'; continue }
                    $tol = [Math]::Max([double]$Environments[$base].DepthTol, [double]$Environments[$other].DepthTol)
                    $log = Join-Path $logs "compare-$pair-$($c.Name)-$d.log"
                    & $verify compare $a $b --depth-tol "$tol" | Set-Content $log
                    Add-Result $pair $c.Name $d ($(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' })) (Get-Verdict $log)
                }
            }
        }
    }
} finally {
    Restore-RigProfiles
}

exit (Write-Summary $logs)
