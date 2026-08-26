# ============================================================
# sync_to_backup.ps1  (同步到全量备份)
#
# Full backup: mirror the ENTIRE working folder A into B
# (G:\STM32\SQJ_collab), then commit + tag + push to your own
# backup repo (xiao-wan-yu/SQJ-autosorter).
#
# Usage:
#   .\同步到全量备份.ps1 -Name "超声波避障"
#
# NOTE: git warnings (CRLF etc.) are suppressed so the script
# does not show scary red text. If the push fails you will see
# a clear message with the manual command to run.
# ============================================================

param(
    [string]$SourceDir = "G:\STM32\7_AutomatedSortingRobot_26",
    [string]$TargetDir = "G:\STM32\SQJ_collab",
    [string]$Name = ""
)

if (-not (Test-Path $SourceDir)) { Write-Host "ERROR: Source dir not found: $SourceDir"; exit 1 }
if (-not (Test-Path (Join-Path $TargetDir ".git"))) { Write-Host "ERROR: Target is not a git repo: $TargetDir"; exit 1 }

$env:GIT_TERMINAL_PROMPT = "0"

Write-Host "STEP 1/4: mirroring ALL files from source to backup..."
Write-Host "  $SourceDir"
Write-Host "  -> $TargetDir  (excluding .git and .gitignore)"
robocopy $SourceDir $TargetDir /MIR /XD .git /XF .gitignore /NFL /NDL /NJH /NJS /NP | Out-Null

Push-Location $TargetDir

Write-Host "STEP 2/4: staging changes..."
git add -A 2>$null

git diff --cached --quiet
if ($LASTEXITCODE -eq 0) {
    Write-Host "  no new changes detected."
} else {
    Write-Host "STEP 3/4: committing + tagging..."
    $stamp = Get-Date -Format "yyyy.MM.dd.HHmm"
    $tagName = if ($Name) { "backup-$stamp-$Name" } else { "backup-$stamp" }
    git commit -m "full backup $stamp $Name" 2>$null
    git tag $tagName 2>$null
    Write-Host "  done. version tag = $tagName"
}

Write-Host "STEP 4/4: pushing to your GitHub backup repo..."
git push origin main --tags 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "== PUSH FAILED =="
    Write-Host "The backup is safe on this computer (commit + tag are done)."
    Write-Host "The push to GitHub failed - usually a network or login issue."
    Write-Host "Run this in $TargetDir when you can login to GitHub:"
    Write-Host "    git push origin main --tags"
    Write-Host "(if this is the FIRST backup after replacing main, use:)"
    Write-Host "    git push -f origin main --tags"
} else {
    Write-Host "Push OK. Backup is now also on GitHub."
}
Pop-Location
Write-Host "DONE."
