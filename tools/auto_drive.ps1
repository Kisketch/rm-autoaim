# auto_drive.ps1 - run the improvement queue.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\auto_drive.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File tools\auto_drive.ps1 -Loop -IdleMinutes 10
#
# For each pending task in the queue:
#   1. cut a branch auto/<id>-<ts> (only modified tracked files block the run)
#   2. execute `py <task.script>`
#   3. run the gate `py tools/regression.py`
#   4. green -> commit + fast-forward main; red -> drop the branch, main untouched
#   5. update the queue file and append to RM-PRO/auto-drive-log.md
#
# With -Loop the driver stays resident: when the queue is empty it sleeps
# IdleMinutes and checks again. Stop it with Ctrl+C or by killing the process.

param(
    [string]$Repo  = "C:\Users\Lgnatius\Desktop\RM\autoaim-takeover",
    [string]$Queue = "C:\Users\Lgnatius\Desktop\RM-PRO\auto-queue.json",
    [int]$MaxTasks = 1,
    [int]$CooldownSec = 20,
    [switch]$Loop,
    [int]$IdleMinutes = 10,
    [switch]$Notify,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$logPath = Join-Path (Split-Path $Queue) "auto-drive-log.md"
$env:Path = "C:\opencv\opencv\build\x64\vc16\bin;" + $env:Path

# Single instance: two drivers racing on git checkout can orphan a merge.
$mutex = New-Object System.Threading.Mutex($false, "Global\rm_auto_drive")
if (-not $mutex.WaitOne(0)) {
    Write-Host "another auto_drive instance is already running; exiting"
    exit 0
}

function Log([string]$text) {
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $line = "- $ts  $text"
    Add-Content -LiteralPath $logPath -Value $line -Encoding UTF8
    Write-Host $line
}

if (-not (Test-Path $logPath)) {
    Set-Content -LiteralPath $logPath -Value "# 自动推进日志`n" -Encoding UTF8
}

$statusPath = Join-Path (Split-Path $Queue) "auto-status.json"

function Write-Status([string]$task, [string]$state) {
    $pending = 0
    try {
        $pending = @((Get-Content -LiteralPath $Queue -Raw -Encoding UTF8 | ConvertFrom-Json).tasks |
                     Where-Object { $_.status -eq "pending" }).Count
    } catch {}
    [pscustomobject]@{
        updated = (Get-Date -Format "s")
        task    = $task
        state   = $state
        pending = $pending
    } | ConvertTo-Json | Set-Content -LiteralPath $statusPath -Encoding UTF8
    if ($Notify) {
        try {
            Add-Type -AssemblyName System.Windows.Forms
            Add-Type -AssemblyName System.Drawing
            $icon = New-Object System.Windows.Forms.NotifyIcon
            $icon.Icon = [System.Drawing.SystemIcons]::Information
            $icon.Visible = $true
            $icon.ShowBalloonTip(4000, "auto_drive", "$task -> $state (pending $pending)", "Info")
            Start-Sleep -Seconds 1
            $icon.Dispose()
        } catch { }
    }
}

Log ("驱动器启动（Loop=" + $Loop + ", MaxTasks=" + $MaxTasks + "）")

while ($true) {
    $ran = 0
    while ($ran -lt $MaxTasks) {
        $data = Get-Content -LiteralPath $Queue -Raw -Encoding UTF8 | ConvertFrom-Json
        $task = $data.tasks | Where-Object { $_.status -eq "pending" } | Select-Object -First 1
        if (-not $task) { break }

        Log "开始 $($task.id)：$($task.title)"
        if ($DryRun) { Write-Host "  script: $($task.script)"; $ran++; continue }

        Push-Location $Repo
        $status = "failed"
        try {
            $dirty = git status --porcelain | Where-Object { $_ -match '^[ MARC][MD]' }
            if ($dirty) { Log "有未提交的修改，跳到下一个"; break }

            $scriptRel = $task.script
            if (-not $scriptRel -or -not (Test-Path (Join-Path $Repo $scriptRel))) {
                Log "  没有可执行脚本 ($scriptRel)，标 blocked"
                $task.status = "blocked"
                $data | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Queue -Encoding UTF8
                $ran++
                continue
            }

            $ts = Get-Date -Format "MMdd-HHmm"
            $branch = "auto/$($task.id)-$ts"
            git checkout -q -b $branch

            Log "  执行 py $scriptRel"
            $out = & py $scriptRel 2>&1
            Log "  脚本尾行：`n$(($out | Select-Object -Last 6) -join "`n")"

            Log "  跑门禁"
            $gateOut = & py tools/regression.py 2>&1
            $gateOk = ($LASTEXITCODE -eq 0)
            Log ("  门禁 " + $(if ($gateOk) { "PASS" } else { "FAIL" }))

            if ($gateOk) {
                git add -A
                $msg = "auto($($task.id)): $($task.title)"
                $null = git commit -q -m $msg 2>&1
                if ($LASTEXITCODE -eq 0) {
                    git checkout -q main
                    git merge -q --ff-only $branch
                    Log "  已合入 main：$msg"
                    $task.status = "done"
                } else {
                    Log "  没有可提交的改动"
                    git checkout -q main
                    git branch -D $branch | Out-Null
                    $task.status = "done"
                }
            } else {
                if (git status --porcelain) { git checkout -q -- . }
                git checkout -q main
                git branch -D $branch | Out-Null
                Log "  门禁未过，丢弃分支"
                $task.status = "failed"
            }
            $status = $task.status
        }
        catch {
            Log "  异常：$($_.Exception.Message)"
            try { git checkout -q main } catch {}
        }
        finally {
            Pop-Location
        }

        $data.tasks | Where-Object { $_.id -eq $task.id } | ForEach-Object { $_.status = $status }
        $data | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Queue -Encoding UTF8
        Write-Status $task.id $status
        $ran++
        if ($CooldownSec -gt 0) { Start-Sleep -Seconds $CooldownSec }
    }

    if (-not $Loop) { break }
    Log "队列已空，休眠 $IdleMinutes 分钟后重试"
    Start-Sleep -Seconds ($IdleMinutes * 60)
}
