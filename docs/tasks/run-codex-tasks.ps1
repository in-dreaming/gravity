[CmdletBinding()]
param(
    [string]$StateDir = "zig-out/codex-task-runs",
    # Zero means keep going until the task proves completion or the operator stops it.
    [ValidateRange(0, 1000)]
    [int]$MaxAttempts = 0,
    [ValidateRange(1, 10)]
    [int]$MaxTurnsPerSession = 3,
    [string[]]$TaskDocuments = @(
        "task-21-snapshot-rollback-replay-diff.md",
        "task-22-c-abi-packaging.md",
        "task-22a-spindle-executor-adoption.md",
        "task-23-deterministic-job-system.md"
    )
)

$ErrorActionPreference = "Stop"

# This is deliberately not a parameter: every child invocation is pinned to
# the model selected for this task-chain run.
$Model = "gpt-5.6-terra"
$TaskDirectory = $PSScriptRoot
$SetupDocument = Join-Path $TaskDirectory "setup.md"
$ReadmeDocument = Join-Path $TaskDirectory "README.md"

$RepositoryRoot = (git -C $TaskDirectory rev-parse --show-toplevel).Trim()
if (-not $RepositoryRoot) {
    throw "Unable to determine the Gravity repository root."
}

$StateDirectory = if ([IO.Path]::IsPathRooted($StateDir)) {
    $StateDir
}
else {
    Join-Path $RepositoryRoot $StateDir
}
New-Item -ItemType Directory -Force -Path $StateDirectory | Out-Null
$RunId = Get-Date -Format "yyyyMMdd-HHmmss"

foreach ($required in @($SetupDocument, $ReadmeDocument)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Missing required task-chain document: $required"
    }
}

$codexCommand = @(
    Get-Command codex.ps1 -ErrorAction SilentlyContinue
    Get-Command codex.cmd -ErrorAction SilentlyContinue
    Get-Command codex -All -ErrorAction SilentlyContinue |
        Where-Object { $_.Source -notlike "*\\WindowsApps\\*" }
) | Where-Object { $null -ne $_ } | Select-Object -First 1
if ($null -eq $codexCommand) {
    throw "Unable to find an executable Codex CLI shim outside WindowsApps on PATH."
}
$Codex = if ($codexCommand.Source) { $codexCommand.Source } else { $codexCommand.Definition }

function Get-DocumentText {
    param([Parameter(Mandatory)][string]$Path)

    return [IO.File]::ReadAllText($Path)
}

function Test-CompleteReport {
    param([Parameter(Mandatory)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return $false
    }
    $report = Get-DocumentText -Path $Path
    $complete = $report -match '(?m)^TASK_STATUS:\s*COMPLETE\s*$'
    $nothingRemaining = $report -match '(?ms)未完成项：\s*无\s*(?:\r?\n|$)'
    return $complete -and $nothingRemaining
}

function Get-CodexThreadId {
    param([Parameter(Mandatory)][string]$EventsPath)

    if (-not (Test-Path -LiteralPath $EventsPath -PathType Leaf)) {
        return $null
    }

    foreach ($line in Get-Content -LiteralPath $EventsPath) {
        try {
            $event = $line | ConvertFrom-Json -ErrorAction Stop
            if ($event.type -eq "thread.started" -and $event.thread_id) {
                return $event.thread_id.ToString()
            }
        }
        catch {
            # Native diagnostics and profile output may be mixed into a JSONL log.
        }
    }

    return $null
}

function Get-LatestTaskReport {
    param([Parameter(Mandatory)][string]$TaskId)

    $report = Get-ChildItem -LiteralPath $StateDirectory -Filter "$TaskId*.result.md" -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($report) {
        return Get-DocumentText -Path $report.FullName
    }
    return "No earlier task report is available."
}

function Test-CodexTurnUsedTools {
    param([Parameter(Mandatory)][string]$EventsPath)

    if (-not (Test-Path -LiteralPath $EventsPath -PathType Leaf)) {
        return $false
    }
    return [IO.File]::ReadAllText($EventsPath) -match '"type"\s*:\s*"(?:command_execution|file_change|mcp_tool_call)"'
}

function Invoke-CodexTask {
    param(
        [Parameter(Mandatory)][IO.FileInfo]$Task,
        [Parameter(Mandatory)][int]$Attempt,
        [Parameter(Mandatory)][string]$HandoffText
    )

    $taskId = $Task.BaseName
    $resultPath = Join-Path $StateDirectory "$taskId.$RunId.attempt-$Attempt.result.md"
    $eventsPath = Join-Path $StateDirectory "$taskId.$RunId.attempt-$Attempt.jsonl"
    $readmeText = Get-DocumentText -Path $ReadmeDocument
    $setupText = Get-DocumentText -Path $SetupDocument
    $taskText = Get-DocumentText -Path $Task.FullName

    $prompt = @"
Implement exactly one Gravity task in this repository:
$RepositoryRoot

The complete README, setup document, and task document are included below.
The setup document is authoritative for shared constraints. The task document
defines the only implementation scope for this invocation.

===== TASK INDEX: $ReadmeDocument =====
$readmeText
===== END TASK INDEX =====

===== SETUP DOCUMENT: $SetupDocument =====
$setupText
===== END SETUP DOCUMENT =====

===== TASK DOCUMENT: $($Task.FullName) =====
$taskText
===== END TASK DOCUMENT =====

===== PRIOR ATTEMPT HANDOFF =====
$HandoffText
===== END PRIOR ATTEMPT HANDOFF =====

Required procedure:
1. Re-read README.md, setup.md, and $($Task.Name) from the working tree before editing.
2. Inspect git status and preserve every unrelated staged, unstaged, and untracked change.
3. Verify dependency tasks from implementation and command evidence; do not trust old reports.
4. Build a checklist covering every deliverable, implementation step, validation item,
   and completion condition in the task document.
5. Continue implementing this task from the handoff until every checklist item passes or a concrete,
   evidenced blocker prevents further safe progress.
6. Use production paths. Do not add mocks, stubs, TODOs, skips, reduced assertions,
   fake platform results, or test-only backdoors.
7. Run focused tests while working, then all validation required by the task in
   Debug, ReleaseSafe, ReleaseFast, native/WASM/worker modes where applicable.
8. Inspect the actual build graph when a command name alone does not prove coverage.
9. Review git diff and git diff --check. Do not reset, restore, clean, commit, push,
   create a branch, or modify unrelated user work.
10. Do not start a later task.

Task 21 CLI authority: the task requirement itself authorizes the minimal replay
contract `gravity-replay <replay> [asset.tlv ...]`. Use Task 20's canonical serial
analytic-solver pipeline profile and caller-owned workspaces. Asset-free snapshots
must run without extra files; snapshots referencing asset IDs must require supplied
canonical asset TLVs and verify their IDs/set hash. Implement this contract instead
of requesting confirmation. This does not authorize a new physics algorithm.

The final response must contain exactly one status marker:

TASK_STATUS: COMPLETE

only when every completion condition is satisfied and the unfinished list is
literally empty. Otherwise use:

TASK_STATUS: BLOCKED

The remainder of the final response must use this structure:

实现摘要：
变更文件：
关键不变量：
验证命令与结果：
golden/benchmark/ABI 变化：
未完成项：
下一步：

For COMPLETE, write exactly `未完成项：无`. For BLOCKED, provide exact file,
code, command, and error evidence plus the smallest next action. A time limit,
large scope, or incomplete implementation is not by itself a blocker; preserve
useful progress and continue as far as safely possible.
"@

    $arguments = @(
        "--ask-for-approval", "never",
        "exec",
        "--cd", $RepositoryRoot,
        "--model", $Model,
        "--sandbox", "workspace-write",
        "--json",
        "--output-last-message", $resultPath
    )

    Write-Host "[MODEL] $Model"
    Write-Host "[RESULT] $resultPath"

    $previousErrorActionPreference = $ErrorActionPreference
    $previousNativeCommandPreference = $PSNativeCommandUseErrorActionPreference
    try {
        # Native stderr is diagnostic output, not a PowerShell terminating error.
        $ErrorActionPreference = "Continue"
        $PSNativeCommandUseErrorActionPreference = $false
        $prompt |
            & $Codex @arguments 2>&1 |
            ForEach-Object { $_.ToString() } |
            Tee-Object -FilePath $eventsPath |
            Out-Host
        $exitCode = $LASTEXITCODE
        return [PSCustomObject]@{
            ExitCode = $exitCode
            ResultPath = $resultPath
            EventsPath = $eventsPath
            DiagnosticPath = $eventsPath
        }
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
        $PSNativeCommandUseErrorActionPreference = $previousNativeCommandPreference
    }
}

function Resume-CodexTask {
    param(
        [Parameter(Mandatory)][IO.FileInfo]$Task,
        [Parameter(Mandatory)][int]$Attempt,
        [Parameter(Mandatory)][string]$SessionId
    )

    $taskId = $Task.BaseName
    $resultPath = Join-Path $StateDirectory "$taskId.$RunId.resume-$Attempt.result.md"
    $eventsPath = Join-Path $StateDirectory "$taskId.$RunId.resume-$Attempt.jsonl"
    $readmeText = Get-DocumentText -Path $ReadmeDocument
    $setupText = Get-DocumentText -Path $SetupDocument
    $taskText = Get-DocumentText -Path $Task.FullName

    $prompt = @"
Continue and finish only Gravity task $taskId in the existing repository and session.
Do not restart the investigation or discard useful progress from earlier turns.

Re-read the current working tree, current diff, prior session context, and these
authoritative documents. They are repeated because they may have changed.

===== TASK INDEX: $ReadmeDocument =====
$readmeText
===== END TASK INDEX =====

===== SETUP DOCUMENT: $SetupDocument =====
$setupText
===== END SETUP DOCUMENT =====

===== TASK DOCUMENT: $($Task.FullName) =====
$taskText
===== END TASK DOCUMENT =====

Continue from the unfinished checklist and implement the next concrete items. Preserve
all unrelated work. Run focused validation sequentially; do not repeatedly rerun gates
already proven in this same session unless relevant code changed. On this Windows host,
PowerShell profile diagnostics and a single command timeout are not task blockers; use
nested pwsh -NoProfile where useful, avoid concurrent Zig cache writers, and split long
mode matrices into focused commands.

Resolve apparent wording tension by inspecting the task invariants and choosing the
correctness-preserving implementation that satisfies their intent; document the choice
and tests. Only request user authority when proceeding would materially change public
scope or cause unsafe external effects. Large scope, incomplete work, or the need for
another implementation turn is not a blocker.

For Task 21, the required replay CLI authorizes this minimal contract without further
confirmation: `gravity-replay <replay> [asset.tlv ...]`, Task 20's canonical serial
analytic-solver pipeline profile, caller-owned workspaces, no asset arguments for an
asset-free snapshot, and mandatory canonical asset TLVs with ID/set-hash verification
when the snapshot references assets. Implement it; do not invent a new solver.

Do not commit, push, reset, restore, clean, create a branch, modify unrelated user work,
or start a later task.

The final response must contain exactly one marker: TASK_STATUS: COMPLETE only when all
completion conditions pass, otherwise TASK_STATUS: BLOCKED. Use the required report
sections from the previous prompt. For COMPLETE, write exactly `未完成项：无`.
"@

    $arguments = @(
        "--ask-for-approval", "never",
        "exec", "resume",
        "--model", $Model,
        "--json",
        "--output-last-message", $resultPath,
        $SessionId,
        "-"
    )

    Write-Host "[MODEL] $Model (resume $SessionId)"
    Write-Host "[RESULT] $resultPath"

    $previousErrorActionPreference = $ErrorActionPreference
    $previousNativeCommandPreference = $PSNativeCommandUseErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $PSNativeCommandUseErrorActionPreference = $false
        $prompt |
            & $Codex @arguments 2>&1 |
            ForEach-Object { $_.ToString() } |
            Tee-Object -FilePath $eventsPath |
            Out-Host
        $exitCode = $LASTEXITCODE
        return [PSCustomObject]@{
            ExitCode = $exitCode
            ResultPath = $resultPath
            EventsPath = $eventsPath
            DiagnosticPath = $eventsPath
        }
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
        $PSNativeCommandUseErrorActionPreference = $previousNativeCommandPreference
    }
}

$tasks = foreach ($taskName in $TaskDocuments) {
    $taskPath = if ([IO.Path]::IsPathRooted($taskName)) {
        $taskName
    }
    else {
        Join-Path $TaskDirectory $taskName
    }
    if (-not (Test-Path -LiteralPath $taskPath -PathType Leaf)) {
        throw "Missing task document: $taskPath"
    }
    Get-Item -LiteralPath $taskPath
}

if ($tasks.Count -eq 0) {
    throw "TaskDocuments must contain at least one task document."
}

Set-Location -LiteralPath $RepositoryRoot
Write-Host "[ROOT] $RepositoryRoot"
Write-Host "[CODEX] $Codex"
Write-Host "[MODEL] $Model (forced)"

foreach ($task in $tasks) {
    $taskId = $task.BaseName
    $success = $false
    $sessionId = $null
    $sessionTurns = 0
    $handoffText = Get-LatestTaskReport -TaskId $taskId

    Write-Host ""
    Write-Host "========================================"
    Write-Host "[TASK] $taskId"
    Write-Host "========================================"

    for ($attempt = 1; $MaxAttempts -eq 0 -or $attempt -le $MaxAttempts; $attempt++) {
        $attemptLimit = if ($MaxAttempts -eq 0) { "unlimited" } else { $MaxAttempts }
        Write-Host "[ATTEMPT] $attempt / $attemptLimit"
        if ($sessionId) {
            $run = Resume-CodexTask -Task $task -Attempt $attempt -SessionId $sessionId
            $sessionTurns++
        }
        else {
            Write-Host "[SESSION] Starting a fresh $Model session with the latest handoff."
            $run = Invoke-CodexTask -Task $task -Attempt $attempt -HandoffText $handoffText
            $sessionId = Get-CodexThreadId -EventsPath $run.EventsPath
            $sessionTurns = 1
            if ($sessionId) {
                Write-Host "[SESSION] $sessionId"
            }
        }

        if ($run.ExitCode -ne 0) {
            Write-Warning "Codex exited with code $($run.ExitCode). See $($run.DiagnosticPath)"
            $stderrText = if (Test-Path -LiteralPath $run.DiagnosticPath -PathType Leaf) {
                Get-DocumentText -Path $run.DiagnosticPath
            }
            else {
                ""
            }
            if ($run.ExitCode -eq 2 -and $stderrText -match '(?m)^(error: unexpected argument|Usage: codex)') {
                throw "Codex CLI invocation is invalid; retries cannot repair it. See $($run.DiagnosticPath)"
            }
        }
        elseif (Test-CompleteReport -Path $run.ResultPath) {
            git -C $RepositoryRoot diff --check
            if ($LASTEXITCODE -ne 0) {
                Write-Warning "git diff --check failed after $taskId."
            }
            else {
                Write-Host "[DONE] $taskId"
                $success = $true
                break
            }
        }
        else {
            Write-Warning "Task report did not prove completion: $($run.ResultPath)"
        }

        if (Test-Path -LiteralPath $run.ResultPath -PathType Leaf) {
            $handoffText = Get-DocumentText -Path $run.ResultPath
        }

        $usedTools = Test-CodexTurnUsedTools -EventsPath $run.EventsPath
        $rotateSession = -not $usedTools -or $sessionTurns -ge $MaxTurnsPerSession
        if ($rotateSession) {
            $reason = if (-not $usedTools) { "the turn made no tool progress" } else { "the session reached $MaxTurnsPerSession turns" }
            Write-Warning "Rotating to a fresh $Model session because $reason."
            $sessionId = $null
            $sessionTurns = 0
        }
        elseif ($sessionId) {
            Write-Warning "Task incomplete; continuing session $sessionId with $Model."
        }
        else {
            Write-Warning "No Codex session id was emitted; retrying with a fresh $Model session."
        }

        if ($run.ExitCode -ne 0) {
            Start-Sleep -Seconds 5
        }
    }

    if (-not $success) {
        throw "Task $taskId did not complete after $MaxAttempts attempts. Later tasks were not started."
    }
}

Write-Host ""
Write-Host "All listed task reports claim completion. Running chain-level validation."

$validations = @(
    @("build", "fmt", "--summary", "all"),
    @("build", "test", "--summary", "all"),
    @("build", "test-core-all-modes", "--summary", "all"),
    @("build", "test-pipeline-all-modes", "--summary", "all"),
    @("build", "spindle-check-all-modes", "--summary", "all"),
    @("build", "fuzz", "--summary", "all"),
    @("build", "wasm-validate", "--summary", "all"),
    @("build", "tools", "--summary", "all")
)

foreach ($arguments in $validations) {
    Write-Host "[VERIFY] zig $($arguments -join ' ')"
    & zig @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Chain validation failed: zig $($arguments -join ' ')"
    }
}

git -C $RepositoryRoot diff --check
if ($LASTEXITCODE -ne 0) {
    throw "Final git diff --check failed."
}

Write-Host ""
git -C $RepositoryRoot status --short
Write-Host "All listed tasks and chain-level validation completed with model $Model."
