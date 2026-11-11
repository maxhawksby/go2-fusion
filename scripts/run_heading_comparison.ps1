[CmdletBinding()]
param(
    [switch]$Run,
    [string]$ResumeFrom,
    [ValidateRange(1, 100000)][int]$Seed = 28
)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
    $commit = (& git rev-parse HEAD).Trim()
    if ($ResumeFrom) {
        $manifestPath = (Resolve-Path -LiteralPath $ResumeFrom).Path
        $plan = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
        if ($plan.commit -ne $commit) { throw 'Resume requires the original source commit.' }
    } else {
        $folder = Join-Path $repoRoot ('src/go2_sim/results/ite_hallways/comparisons/' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
        New-Item -ItemType Directory -Force -Path $folder | Out-Null
        $manifestPath = Join-Path $folder 'comparison.json'
        $entries = @()
        for ($index = 0; $index -lt 5; $index++) {
            $modes = @('map_zero', 'toward_goal')
            if ($index % 2 -eq 1) { $modes = @('toward_goal', 'map_zero') }
            foreach ($mode in $modes) {
                $entries += [pscustomobject]@{ seed = $Seed * 1000 + $index; start_index = $index; mode = $mode; status = 'pending'; attempts = @() }
            }
        }
        $plan = [pscustomobject]@{
            commit = $commit; batch_seed = $Seed; timeout_s = 300; world = 'ite_hallways'
            camera = 'disabled'; vlm = 'off'; entries = $entries
        }
    }
    function Save-Plan { $plan | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $manifestPath -Encoding UTF8 }
    Save-Plan
    Write-Host "Manifest: $manifestPath"
    $plan.entries | Select-Object seed, mode, status | Format-Table | Out-Host
    if (-not $Run) { Write-Host 'Plan only. Add -Run to execute; use -ResumeFrom with this manifest to resume.'; return }
    if (@(& git status --porcelain).Count -gt 0) { throw 'Commit source changes before running a comparison.' }
    if (-not (Test-Path 'src/go2_robot_sdk/external_lib/aioice/setup.py')) { throw 'Initialize git submodules before running.' }
    if (@(& docker ps --filter name=oak-ite-run --format '{{.Names}}').Count -gt 0) { throw 'An ITE run is already active.' }
    foreach ($entry in $plan.entries) {
        if ($entry.status -in @('complete', 'recorded_review')) { continue }
        if ((& git rev-parse HEAD).Trim() -ne $commit -or @(& git status --porcelain).Count -gt 0) { throw 'Source changed during comparison.' }
        $attemptNumber = @($entry.attempts).Count + 1
        $variant = "heading_$($entry.seed)_$($entry.mode)_a$attemptNumber" + '_' + (Get-Date -Format 'HHmmssfff')
        $attempt = [pscustomobject]@{ variant = $variant; session = ''; result = $null; error = '' }
        $entry.attempts = @($entry.attempts) + $attempt
        $entry.status = 'running'
        Save-Plan
        try {
            & "$PSScriptRoot/run_ite_hallways.ps1" -Mode Batch -Runs 1 -Seed $plan.batch_seed -StartIndex $entry.start_index -Timeout 300 -Variant $variant -GoalHeadingMode $entry.mode -RecordBag
            $session = Get-ChildItem 'src/go2_sim/results/ite_hallways' -Directory | Where-Object Name -Like "*-$variant-batch" | Select-Object -First 1
            if (-not $session) { throw 'No session folder found.' }
            $attempt.session = $session.FullName
            $csv = Get-ChildItem (Join-Path $session.FullName 'trials') -Directory | ForEach-Object { Get-Item -LiteralPath (Join-Path $_.FullName 'results.csv') -ErrorAction SilentlyContinue } | Select-Object -First 1
            if (-not $csv) { throw 'No trial CSV found.' }
            $result = @(Import-Csv -LiteralPath $csv.FullName)
            if ($result.Count -ne 1 -or [int]$result[0].seed -ne [int]$entry.seed) { throw 'Result seed/count does not match schedule.' }
            $attempt.result = $result[0]
            if ([double]$result[0].time_s -le 0 -or $result[0].end_reason -match 'startup|exited|interrupted|start event') { throw "Infrastructure failure: $($result[0].end_reason)" }
            if ([int]$result[0].collisions -gt 0) { $entry.status = 'recorded_review'; throw 'Collision recorded; inspect before continuing.' }
            if ($result[0].scoreboard_parity -ne 'pass') { $entry.status = 'recorded_review' }
            if ($result[0].scoreboard_parity -ne 'pass') { throw "Measurement parity failure: $($result[0].scoreboard_parity)" }
            $entry.status = 'complete'
            Save-Plan
        } catch {
            $attempt.error = $_.Exception.Message
            if ($entry.status -ne 'recorded_review') { $entry.status = 'needs_review' }
            Save-Plan
            throw "Comparison stopped; evidence retained. Inspect the failed attempt before resuming. $($_.Exception.Message)"
        }
    }
    Write-Host 'Schedule finished. Inspect saved parity, collision and mapping outcomes; this does not mean mapping succeeded.'
} finally { Pop-Location }
