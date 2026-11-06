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
        $dirty = git status --porcelain
        if ($dirty) { Log "工作区不干净，跳过（先提交或还原）"; break }

        $ts = Get-Date -Format "MMdd-HHmm"
        $branch = "auto/$($task.id)-$ts"
        git checkout -q -b $branch

        $prompt = @"
你是这个仓库的改进代理。约束：
- 只改当前仓库；不要动 C:\Users\Lgnatius\Desktop\RM\template、RM-PRO 以及其它仓库。
- 先读 README / BASELINE / docs，再动手；结论必须来自真实运行，不许估算冒充实测。
- 做完必须跑 `python tools/regression.py`，必须全 PASS；不 PASS 就想办法修到 PASS，修不到就保留现状并在报告里写清楚。
- 只改与本任务相关的文件；不要顺便重构。
- 结束时用 3-6 行说明：改了什么、实测数字、剩下什么。

任务：
$($task.prompt)
"@

        Log "  启动 opencode run（超时 $TaskMinutes 分钟）"
        $job = Start-Job -ScriptBlock {
            param($r, $t, $p)
            Set-Location $r
            & opencode run --dir $r --auto --title $t $p 2>&1
        } -ArgumentList $Repo, "[auto] $($task.id) $($task.title)", $prompt

        $done = Wait-Job $job -Timeout ($TaskMinutes * 60)
        if (-not $done) {
            Stop-Job $job
            Log "  超时，终止本次任务"
            $out = ""
            $ok = $false
        } else {
            $out = Receive-Job $job
            $ok = $true
        }
        Remove-Job $job -Force

        $tail = ($out | Select-Object -Last 25) -join "`n"
        Log "  代理输出尾部：`n$tail"

        Log "  跑门禁"
        $gateOut = & python tools/regression.py 2>&1
        $gateOk = ($LASTEXITCODE -eq 0) -and ($ok -eq $true)
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
