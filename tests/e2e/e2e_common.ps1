<#
.SYNOPSIS
What the end-to-end suites share: the graphics API environments, the tolerance and the helpers.

.DESCRIPTION
Dot-sourced by run_e2e.ps1 (the addon's frames) and run_viewer_e2e.ps1 (the viewer's point cloud),
after they set $BuildDir. Defines $root, $bin, $verify, $work, $Environments and $Tolerance.
#>

$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$bin = Join-Path $root "$BuildDir\bin\Release"
$verify = Join-Path $bin 'lidar_verify.exe'
$work = Join-Path $root "$BuildDir\e2e"

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
# 99% of points must lie within this many meters of the true scene. Measured worst case: standard
# depth, p99 ~3 mm (its precision falls off with distance); every other mode stays under 0.5 mm.
$Tolerance = 0.005

# The environments -Api selects ('all' or names), in $Environments order.
function Select-Environments([string[]]$names) {
    foreach ($a in $names) {
        if ($a -ne 'all' -and -not $Environments.Contains($a)) { throw "unknown -Api $a (all, $($Environments.Keys -join ', '))" }
    }
    if ($names -contains 'all') { return @($Environments.Keys) }
    @($Environments.Keys | Where-Object { $names -contains $_ })
}

# Checks that the given programs are built and no fake_game is running, then gives the suite an
# empty log folder.
function Initialize-Suite([string]$logs, [string[]]$needs) {
    foreach ($p in $needs) {
        if (-not (Test-Path $p)) { throw "missing $($p): build it (Release) first" }
    }
    if (Get-Process fake_game -ErrorAction SilentlyContinue) { throw 'a fake_game is already running: close it first' }
    Remove-Item -Recurse -Force $logs -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force $logs | Out-Null
}

# Set the rig's [LIDAR] Profile (the game must be closed); Restore-RigProfiles puts the original
# ReShade.ini files back.
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
function Restore-RigProfiles {
    foreach ($ini in $iniBackups.Keys) {
        if ($null -eq $iniBackups[$ini]) { Remove-Item $ini -ErrorAction SilentlyContinue }
        else { [IO.File]::WriteAllText($ini, $iniBackups[$ini]) }
    }
}

# ReShade in the rig: the D3D9 rig takes the same DLL as the D3D11 one, renamed (docs/development.md).
# Returns why the rig can't run, or $null.
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

# fake_game and lidar_viewer are console programs: started directly (not through the shell) with no
# console window, which would otherwise pop up and take the focus. --background keeps their own
# window from doing so.
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

# ---- Results -------------------------------------------------------------------------------------
$results = [Collections.Generic.List[object]]::new()
function Add-Result($api, $case, $depth, $result, $detail) {
    $results.Add([pscustomobject]@{ Api = $api; Case = $case; Depth = $depth; Result = $result; Detail = $detail })
    $color = @{ PASS = 'Green'; FAIL = 'Red'; SKIP = 'DarkGray' }[$result]
    Write-Host ("{0,-4} {1,-6} {2,-11} {3,-18} {4}" -f $result, $api, $case, $depth, $detail) -ForegroundColor $color
}

# The last line lidar_verify printed that says how it went, for the summary.
function Get-Verdict([string]$log) {
    $lines = @(Get-Content $log | Where-Object { $_ -match 'within|FAIL|no .*frame|no points|mismatch|->|cannot' })
    if ($lines.Count) { $lines[-1].Trim() } else { '' }
}

# Prints the totals and returns the exit code: 1 if anything failed.
function Write-Summary([string]$logs) {
    $failed = @($results | Where-Object Result -eq 'FAIL').Count
    $passed = @($results | Where-Object Result -eq 'PASS').Count
    $skipped = @($results | Where-Object Result -eq 'SKIP').Count
    Write-Host ''
    Write-Host "$passed passed, $failed failed, $skipped skipped. Logs: $logs" -ForegroundColor ($(if ($failed) { 'Red' } else { 'Green' }))
    if ($failed) { 1 } else { 0 }
}
