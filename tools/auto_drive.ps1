# auto_drive.ps1 - run the improvement queue through headless OpenCode sessions.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\auto_drive.ps1 -MaxTasks 1
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\auto_drive.ps1 -DryRun
#
# For each pending task in the queue:
#   1. require a clean worktree, cut a branch auto/<id>-<ts>
#   2. run `opencode run --auto` with the task prompt (timeout per task)
#   3. run the task gate (python tools/regression.py)
#   4. green -> commit + fast-forward main; red -> drop the branch, keep main intact
#   5. update the queue file and append to RM-PRO/auto-drive-log.md

param(
    [string]$Repo  = "C:\Users\Lgnatius\Desktop\RM\autoaim-takeover",
    [string]$Queue = "C:\Users\Lgnatius\Desktop\RM-PRO\auto-queue.json",
    [int]$MaxTasks = 1,
    [int]$TaskMinutes = 25,
    [int]$CooldownSec = 20,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$logPath = Join-Path (Split-Path $Queue) "auto-drive-log.md"
$env:Path = "C:\opencv\opencv\build\x64\vc16\bin;" + $env:Path

function Log([string]$text) {
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $line = "- $ts  $text"
    Add-Content -LiteralPath $logPath -Value $line -Encoding UTF8
    Write-Host $line
}

if (-not (Test-Path $logPath)) {
    Set-Content -LiteralPath $logPath -Value "# 自动推进日志`n" -Encoding UTF8
}

for ($n = 0; $n -lt $MaxTasks; $n++) {
    $data = Get-Content -LiteralPath $Queue -Raw -Encoding UTF8 | ConvertFrom-Json
    $task = $data.tasks | Where-Object { $_.status -eq "pending" } | Select-Object -First 1
    if (-not $task) { Log "队列为空，退出"; break }

    Log "开始 $($task.id)：$($task.title)"
    if ($DryRun) { Write-Host "  prompt: $($task.prompt.Substring(0, [Math]::Min(120, $task.prompt.Length)))..."; continue }

    Push-Location $Repo
    try {
        $dirty = git status --porcelain | Where-Object { $_ -match '^[ MARC][MD]' }
        if ($dirty) { Log "工作区有未提交的修改，跳过（先提交或还原）"; break }

        $ts = Get-Date -Format "MMdd-HHmm"
        $branch = "auto/$($task.id)-$ts"
        git checkout -q -b $branch

        $scriptRel = $task.script
        if (-not $scriptRel) {
            Log "  该任务还没有可执行脚本（tools/tasks/<id>_*.py），跳过"
            $task.status = "blocked"
            $data.tasks | Where-Object { $_.id -eq $task.id } | ForEach-Object { $_.status = $task.status }
            $data | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Queue -Encoding UTF8
            continue
        }

        Log "  执行 py $scriptRel"
        $out = & py $scriptRel 2>&1
        $ok = ($LASTEXITCODE -eq 0)
        $tail = ($out | Select-Object -Last 20) -join "`n"
        Log "  脚本输出尾部：`n$tail"

        Log "  跑门禁"
        $gateOut = & py tools/regression.py 2>&1
        $gateOk = ($LASTEXITCODE -eq 0) -and $ok
        Log ("  门禁 " + $(if ($gateOk) { "PASS" } else { "FAIL" }))

        if ($gateOk) {
            git add -A
            $msg = "auto($($task.id)): $($task.title)"
            git commit -q -m $msg
            git checkout -q main
            git merge -q --ff-only $branch
            Log "  已合入 main：$msg"
            $task.status = "done"
        } else {
            $changed = git status --porcelain
            if ($changed) { git checkout -q -- . }
            git checkout -q main
            git branch -D $branch | Out-Null
            Log "  门禁未过，已丢弃分支 $branch"
            $task.status = "failed"
        }
        $task | Add-Member -NotePropertyName finished -NotePropertyValue (Get-Date -Format "s") -Force
    }
    catch {
        Log "  异常：$($_.Exception.Message)"
        try { git checkout -q main } catch {}
    }
    finally {
        Pop-Location
    }

    $data.tasks | Where-Object { $_.id -eq $task.id } | ForEach-Object { $_.status = $task.status }
    $data | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Queue -Encoding UTF8

    if ($CooldownSec -gt 0 -and $n -lt $MaxTasks - 1) { Start-Sleep -Seconds $CooldownSec }
}
