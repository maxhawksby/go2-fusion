[CmdletBinding()]
param(
    [ValidateSet('Validate', 'Smoke', 'Batch', 'Preview')]
    [string]$Mode = 'Validate',
    [ValidateRange(1, 50)][int]$Runs = 5,
    [ValidateRange(1, 100000)][int]$Seed = 28,
    [ValidateRange(30, 3600)][int]$Timeout = 600,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]*$')]
    [string]$Variant = 'master_hallways',
    [string]$Image = 'go2scoreboard-unitree_ros:latest',
    [switch]$RecordBag
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$oldImage = $env:OAK_ITE_IMAGE
Push-Location $repoRoot
try {
    & docker info --format '{{.ServerVersion}}' | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Start Docker Desktop, then retry.' }
    & docker image inspect $Image 2>$null | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Simulator image '$Image' is missing. Use -Image with an existing Go2 simulator image, or follow the simulator wiki image-build instructions."
    }
    $env:OAK_ITE_IMAGE = $Image
    $resultRoot = Join-Path $repoRoot 'src/go2_sim/results/ite_hallways'
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $sessionDir = Join-Path $resultRoot "$stamp-$Variant-$($Mode.ToLower())"
    New-Item -ItemType Directory -Force -Path $sessionDir | Out-Null
    $relativeSession = "src/go2_sim/results/ite_hallways/$(Split-Path $sessionDir -Leaf)"
    $containerOut = "/ros2_ws/$relativeSession/trials"
    $command = @('bash', '-ic', 'python3 -m pytest /ros2_ws/src/go2_sim/test/test_world_tools.py /ros2_ws/src/go2_sim/test/test_ite_hallways.py -q')
    if ($Mode -eq 'Preview') {
        # No explorer starts here; this command only writes the answer-key map.
        $command = @('ros2', 'run', 'go2_sim', 'ground_truth', 'ite_hallways', "/ros2_ws/$relativeSession/ground_truth.yaml")
    }
    if ($Mode -in @('Smoke', 'Batch')) {
        $runCount = $Runs
        $runTimeout = $Timeout
        if ($Mode -eq 'Smoke') { $runCount = 1; $runTimeout = 60 }
        $command = @('ros2', 'run', 'go2_sim', 'trials', '--world', 'ite_hallways',
            '--runs', "$runCount", '--seed', "$Seed", '--timeout', "$runTimeout",
            '--stall-timeout', '180', '--startup-timeout', '180', '--variant', $Variant,
            '--out', $containerOut)
        if (-not $RecordBag) { $command += '--no-scoreboard-bag' }
    }
    $metadata = [ordered]@{
        mode = $Mode; variant = $Variant; seed = $Seed; image = $Image
        commit = (& git rev-parse HEAD); branch = (& git branch --show-current)
        changes = @(& git status --porcelain); command = $command
        recorded_at = (Get-Date -Format o)
    }
    $metadata | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $sessionDir 'session.json') -Encoding UTF8
    # Preserve the exact world even before this branch is committed.
    Copy-Item -LiteralPath 'src/go2_sim/worlds/ite_hallways.sdf' -Destination $sessionDir
    & git diff HEAD --binary --output="$sessionDir/source.patch"
    Write-Host "Mode: $Mode. Output: $sessionDir"
    Write-Host 'Only the simulator is used. Batch completion does not mean mapping succeeded.'
    # Windows PowerShell wraps native stderr as errors, including harmless
    # colcon output. Let the process finish and use its actual exit code.
    $ErrorActionPreference = 'Continue'
    & docker compose -p go2ite -f docker/docker-compose.windows.yml -f docker/docker-compose.ite.yml `
        run --rm --no-deps --name oak-ite-run -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=1 `
        unitree_ros @command 2>&1 | ForEach-Object { "$_" } |
        Tee-Object -FilePath (Join-Path $sessionDir 'console.log')
    $runExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    @{ exit_code = $runExit; finished_at = (Get-Date -Format o) } |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $sessionDir 'exit.json') -Encoding UTF8
    Write-Host "Saved results and logs: $sessionDir"
    if ($runExit -ne 0) { throw "Command exited with $runExit. Inspect console.log." }
}
finally {
    $env:OAK_ITE_IMAGE = $oldImage
    Pop-Location
}
