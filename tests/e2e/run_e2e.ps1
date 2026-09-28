<#
.SYNOPSIS
End-to-end tests: the ReShade addon capturing the fake game, checked with lidar_verify.

.DESCRIPTION
There is one test flow ($Cases below). A graphics API is only an environment ($Environments): which
rig folder ReShade is installed in, which --api the fake game renders with, what the reference
needs to match it and how close its depth can get. Every case runs in every selected environment
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
$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')

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

# ---- The environments: the only per-API part ---------------------------------------------------
# Rig: where ReShade is installed as ReShadeDll. Reference: flags for the D3D11 reference to match
# this API's depth format. DepthTol: allowed per-pixel depth difference against that reference, and
# against the other APIs' frames.
$Environments = [ordered]@{
    d3d11 = @{ Rig = 'sandbox\fake_game'; ReShadeDll = 'dxgi.dll'; Args = @('--api', 'd3d11')
               Reference = @(); DepthTol = 0 }
    d3d12 = @{ Rig = 'sandbox\fake_game'; ReShadeDll = 'dxgi.dll'; Args = @('--api', 'd3d12')
               Reference = @(); DepthTol = 0 }
    # 24-bit depth: in reversed modes ~5% of pixels are one step (6e-8) off the --d24 reference, and
    # under a step off the other APIs' float depth.
    d3d9  = @{ Rig = 'sandbox\fake_game_d3d9'; ReShadeDll = 'd3d9.dll'; Args = @('--api', 'd3d9')
               Reference = @('--d24'); DepthTol = 6e-8 }
}
$FreezeAt = 3  # path time for the frozen cases: every environment captures this same view
# 99% of points must lie within this many meters of the true scene. Measured worst case: standard
# depth, p99 ~3 mm (its precision falls off with distance); every other mode stays under 0.5 mm.
$Tolerance = 0.005

# ---- Runner --------------------------------------------------------------------------------------
$bin = Join-Path $root "$BuildDir\bin\Release"
$verify = Join-Path $bin 'lidar_verify.exe'
$work = Join-Path $root "$BuildDir\e2e"
$logs = Join-Path $work 'logs'
$dumps = Join-Path $work 'frames'
foreach ($p in $verify, (Join-Path $root 'sandbox\fake_game\fake_game.exe')) {
    if (-not (Test-Path $p)) { throw "missing $($p): build lidar_verify and fake_game (Release) first" }
}
if (Get-Process fake_game -ErrorAction SilentlyContinue) { throw 'a fake_game is already running: close it first' }
Remove-Item -Recurse -Force $logs, $dumps -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $logs, $dumps | Out-Null

# The reference runs un-injected, so it must live outside sandbox\ (ReShade would hook it there).
$refDir = Join-Path $work 'reference'
New-Item -ItemType Directory -Force $refDir | Out-Null
Copy-Item (Join-Path $root 'sandbox\fake_game\fake_game.exe') $refDir -Force
$refExe = Join-Path $refDir 'fake_game.exe'

foreach ($a in $Api) { if ($a -ne 'all' -and -not $Environments.Contains($a)) { throw "unknown -Api $a (all, $($Environments.Keys -join ', '))" } }
foreach ($d in $Depth) { if ($d -notin @('default', 'all') + $AllDepths) { throw "unknown -Depth $d" } }
function Get-CaseDepths($c) {
    if ($Depth -contains 'all') { return $AllDepths }
    if ($Depth -contains 'default') { if ($c.Depths) { return $c.Depths } else { return $AllDepths } }
    return $Depth
}
$apis = if ($Api -contains 'all') { @($Environments.Keys) } else { @($Environments.Keys | Where-Object { $Api -contains $_ }) }
$cases = @($Cases | Where-Object { $n = $_.Name; @($Case | Where-Object { $n -like $_ }).Count -gt 0 })
if ($cases.Count -eq 0) { throw "no case matches '$Case' (cases: $($Cases.Name -join ', '))" }

# Set the rig's [LIDAR] Profile (the game must be closed); the original ReShade.ini comes back at the end.
$iniBackups = @{}
function Set-RigProfile([string]$rig, [string]$profileName) {
    $ini = Join-Path $rig 'ReShade.ini'
    if (-not $iniBackups.ContainsKey($ini)) {
        $iniBackups[$ini] = if (Test-Path $ini) { [IO.File]::ReadAllText($ini) } else { $null }
    }
    $lines = if (Test-Path $ini) { @([IO.File]::ReadAllLines($ini)) } else { @() }
    $out = [Collections.Generic.List[string]]::new()
    $section = ''; $done = $false
    foreach ($l in $lines) {
        if ($l -match '^\[(.+)\]$') {
            if ($section -eq 'LIDAR' -and -not $done) { $out.Add("Profile=$profileName"); $done = $true }
            $section = $Matches[1]
        }
        if ($section -eq 'LIDAR' -and $l -match '^Profile=') {
            if (-not $done) { $out.Add("Profile=$profileName"); $done = $true }
            continue
        }
        $out.Add($l)
    }
    if ($section -eq 'LIDAR' -and -not $done) { $out.Add("Profile=$profileName"); $done = $true }
    if (-not $done) { $out.Add('[LIDAR]'); $out.Add("Profile=$profileName") }
    [IO.File]::WriteAllLines($ini, $out)
}

# ReShade in the rig: the D3D9 rig takes the same DLL as the D3D11 one, renamed (docs/development.md).
function Test-Rig($envName, $e) {
    $rig = Join-Path $root $e.Rig
    if (-not (Test-Path (Join-Path $rig 'fake_game.exe'))) { return "no fake_game.exe in $($e.Rig) (build fake_game)" }
    $dll = Join-Path $rig $e.ReShadeDll
    if (-not (Test-Path $dll)) {
        $src = Join-Path $root 'sandbox\fake_game\dxgi.dll'
        if (-not (Test-Path $src)) { return "ReShade isn't installed in sandbox\fake_game" }
        Copy-Item $src $dll
        Write-Host "installed ReShade in $($e.Rig) as $($e.ReShadeDll)"
    }
    return $null
}

# fake_game is a console program: started directly (not through the shell) with no console window,
# which would otherwise pop up and take the focus. --background keeps its game window from doing so.
function Start-Game([string]$exe, [string[]]$gameArgs) {
    $si = [Diagnostics.ProcessStartInfo]::new($exe)
    $si.Arguments = (($gameArgs + '--background') | ForEach-Object { if ($_ -match '\s') { "`"$_`"" } else { $_ } }) -join ' '
    $si.WorkingDirectory = Split-Path $exe
    $si.UseShellExecute = $false
    $si.CreateNoWindow = $true
    [Diagnostics.Process]::Start($si)
}
function Stop-Games($procs) {
    foreach ($p in $procs) {
        if ($p -and -not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
    }
    foreach ($p in $procs) { if ($p) { $p.WaitForExit(10000) | Out-Null } }
}

$results = [Collections.Generic.List[object]]::new()
function Add-Result($api, $case, $depth, $result, $detail) {
    $results.Add([pscustomobject]@{ Api = $api; Case = $case; Depth = $depth; Result = $result; Detail = $detail })
    $color = @{ PASS = 'Green'; FAIL = 'Red'; SKIP = 'DarkGray' }[$result]
    Write-Host ("{0,-4} {1,-6} {2,-11} {3,-18} {4}" -f $result, $api, $case, $depth, $detail) -ForegroundColor $color
}

# The last line lidar_verify printed that says how it went, for the summary.
function Get-Verdict([string]$log) {
    $lines = @(Get-Content $log | Where-Object { $_ -match 'within|FAIL|no .*frame|mismatch|->' })
    if ($lines.Count) { $lines[-1].Trim() } else { '' }
}

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
    foreach ($ini in $iniBackups.Keys) {
        if ($null -eq $iniBackups[$ini]) { Remove-Item $ini -ErrorAction SilentlyContinue }
        else { [IO.File]::WriteAllText($ini, $iniBackups[$ini]) }
    }
}

$failed = @($results | Where-Object Result -eq 'FAIL').Count
$passed = @($results | Where-Object Result -eq 'PASS').Count
$skipped = @($results | Where-Object Result -eq 'SKIP').Count
Write-Host ''
Write-Host "$passed passed, $failed failed, $skipped skipped. Logs: $logs" -ForegroundColor ($(if ($failed) { 'Red' } else { 'Green' }))
exit ($(if ($failed) { 1 } else { 0 }))
