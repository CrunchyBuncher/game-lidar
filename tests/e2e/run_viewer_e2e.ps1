<#
.SYNOPSIS
End-to-end tests of the viewer: the addon captures the fake game, lidar_viewer builds its point
cloud on the GPU, and lidar_verify checks the saved scan against the true scene.

.DESCRIPTION
Where run_e2e.ps1 checks the addon's frames (with lidar_verify's CPU unprojection), this checks
what the viewer makes of them: GPU unprojection, voxel dedupe and carving. There is one flow
($Scenarios below), run in every graphics API environment ($Environments in e2e_common.ps1, shared
with run_e2e.ps1). Per run: the ReShade-injected fake_game walks its camera path at 4x speed
(--no-publish: the addon publishes), lidar_viewer saves a .ply after -SaveAfter seconds and exits,
and `lidar_verify ply` needs the scenario's MinWithin of the points within $Tolerance of the scene
(99.9% static, 99.5% with the NPC).

Needs a Release build (lidar_viewer, lidar_verify, lidar_capture, fake_game) and ReShade in
sandbox\fake_game\ as dxgi.dll. Close any game with the addon first: everything here shares the
frame ring.

The game and the viewer open behind other windows (--background, no console), so they don't take
the focus.

.EXAMPLE
tests\e2e\run_viewer_e2e.ps1                     # both scenarios in every API: 6 runs
.EXAMPLE
tests\e2e\run_viewer_e2e.ps1 -Api d3d9 -Scenario static
#>
param(
    [string[]]$Api = 'all',        # all | d3d11 | d3d12 | d3d9, several comma-separated
    [string[]]$Scenario = '*',     # names, wildcards allowed
    [double]$SaveAfter = 8,        # viewer seconds before it saves the scan (about half a lap)
    [string]$BuildDir = 'build'
)
$ErrorActionPreference = 'Stop'
# Lists arrive as one "a,b" string through powershell -File.
$Api, $Scenario = ($Api, $Scenario) | ForEach-Object { , @($_ -split ',' | ForEach-Object Trim | Where-Object { $_ }) }
. (Join-Path $PSScriptRoot 'e2e_common.ps1')  # $Environments, $Tolerance, the helpers

# ---- The test flow: written once, run in every environment ---------------------------------------
# Game: extra fake_game flags. Viewer: extra lidar_viewer flags. MinWithin: the fraction of points
# that must lie within $Tolerance of the scene.
# npc-carve: the NPC walks through the level and carving removes where it was. Its points where it
# stands at save time aren't in the static true scene (up to 1.8 m off), so they count as errors:
# measured 0.05-0.13% of the scan (4x speed, saves across a lap). With carving broken its whole
# trail would stay: 0.8-1.2% (measured with --no-carve), which 99.5% catches.
# static: neither, so the whole scan must match the scene (measured: no point off by 5 mm).
$Scenarios = @(
    @{ Name = 'npc-carve'; Game = @();          Viewer = @();             MinWithin = 0.995 }
    @{ Name = 'static';    Game = @('--no-npc'); Viewer = @('--no-carve'); MinWithin = 0.999 }
)
$ProfileName = 'lidar_profile.toml'  # the addon's camera profile, next to the rig's exe
$Depth = 'reversed'
$Speed = 4  # the camera walks its path 4x as fast: a harder load, and a shorter run

# ---- Runner --------------------------------------------------------------------------------------
$viewer = Join-Path $bin 'lidar_viewer.exe'
$logs = Join-Path $work 'viewer-logs'
Initialize-Suite $logs @($verify, $viewer, (Join-Path $root 'sandbox\fake_game\fake_game.exe'))

$apis = Select-Environments $Api
$scenarios = @($Scenarios | Where-Object { $n = $_.Name; @($Scenario | Where-Object { $n -like $_ }).Count -gt 0 })
if ($scenarios.Count -eq 0) { throw "no scenario matches '$Scenario' (scenarios: $($Scenarios.Name -join ', '))" }
$exitAfter = $SaveAfter + 1

try {
    foreach ($apiName in $apis) {
        $e = $Environments[$apiName]
        $rigProblem = Test-Rig $apiName $e
        $rig = Join-Path $root $e.Rig
        foreach ($s in $scenarios) {
            if ($rigProblem) { Add-Result $apiName $s.Name '-' 'SKIP' $rigProblem; continue }

            Set-RigProfile $rig $ProfileName
            $ply = Join-Path $logs "$apiName-$($s.Name).ply"
            $log = Join-Path $logs "$apiName-$($s.Name).log"
            $gameArgs = @($e.Args) + $s.Game + @('--no-publish', '--depth', $Depth, '--speed', "$Speed",
                                                  '--duration', "$($exitAfter + 15)")
            $viewerArgs = $s.Viewer + @('--out', $ply, '--save-after', "$SaveAfter", '--exit-after', "$exitAfter")
            $verifyArgs = @('ply', $ply, '--tol', "$Tolerance", '--min-within', "$($s.MinWithin)")
            "fake_game $($gameArgs -join ' ')`nlidar_viewer $($viewerArgs -join ' ')`nlidar_verify $($verifyArgs -join ' ')`n" | Set-Content $log

            $procs = @()
            $problem = $null
            try {
                $procs += Start-Game (Join-Path $rig 'fake_game.exe') $gameArgs
                $procs += Start-Game $viewer $viewerArgs
                if (-not $procs[1].WaitForExit([int](($exitAfter + 15) * 1000))) { $problem = 'the viewer did not exit' }
            } finally {
                Stop-Games $procs
            }
            if (-not $problem -and -not (Test-Path $ply)) { $problem = 'the viewer saved no scan' }
            if ($problem) {
                $problem | Add-Content $log
                Add-Result $apiName $s.Name $Depth 'FAIL' $problem
                continue
            }
            & $verify @verifyArgs | Add-Content $log
            Add-Result $apiName $s.Name $Depth ($(if ($LASTEXITCODE -eq 0) { 'PASS' } else { 'FAIL' })) (Get-Verdict $log)
        }
    }
} finally {
    Restore-RigProfiles
}

exit (Write-Summary $logs)
