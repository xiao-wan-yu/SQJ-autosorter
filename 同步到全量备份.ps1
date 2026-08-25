# ============================================================
# sync_to_backup.ps1  (同步到全量备份)
#
# Full backup: mirror the ENTIRE working folder A into B
# (G:\STM32\SQJ_collab), then commit + tag + push to your own
# backup repo (xiao-wan-yu/SQJ-autosorter).
#
# Usage (run in PowerShell, paths are optional):
#   .\同步到全量备份.ps1
#
# NOTE: B's .gitignore is NOT overwritten (it stays in full mode
# so EVERYTHING gets committed there). Only the two repos know.
# ============================================================

param(
    [string]$SourceDir = "G:\STM32\7_AutomatedSortingRobot_26",
    [string]$TargetDir = "G:\STM32\SQJ_collab",
    [string]$Name = ""
)

if (-not (Test-Path $SourceDir)) { Write-Host "ERROR: Source dir not found: $SourceDir"; exit 1 }
if (-not (Test-Path (Join-Path $TargetDir ".git"))) { Write-Host "ERROR: Target is not a git repo: $TargetDir"; exit 1 }

Write-Host "Mirroring ALL files from: $SourceDir"
Write-Host "                     to: $TargetDir   (excluding .git and .gitignore)"
robocopy $SourceDir $TargetDir /MIR /XD .git /XF .gitignore /NFL /NDL /NJH /NJS /NP | Out-Null

Push-Location $TargetDir
git add -A
git diff --cached --quiet
if ($LASTEXITCODE -eq 0) {
    Write-Host "No changes to back up."
} else {
    $stamp = Get-Date -Format "yyyy.MM.dd.HHmm"
    $tagName = if ($Name) { "backup-$stamp-$Name" } else { "backup-$stamp" }
    git commit -m "full backup $stamp $Name"
    git tag $tagName
    Write-Host "Committed and tagged: $tagName"
    git push origin main --tags
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Push failed (auth/network?). Run it manually in $TargetDir :"
        Write-Host '    git push origin main --tags'
        Write-Host "If this is the FIRST backup (main replaced), you may need:"
        Write-Host '    git push -f origin main --tags'
    }
}
Pop-Location
