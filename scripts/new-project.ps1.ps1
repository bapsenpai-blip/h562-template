# Tao project moi tu template nay.
#   powershell -ExecutionPolicy Bypass -File scripts\new-project.ps1 -Name ten_project
#   (tuy chon) -Dest "D:\CODESTM32\ten_project"   mac dinh: canh thu muc template
param(
    [Parameter(Mandatory = $true)][string]$Name,
    [string]$Dest = ""
)
$ErrorActionPreference = "Stop"

$tpl = Split-Path $PSScriptRoot -Parent
if (-not $Dest) { $Dest = Join-Path (Split-Path $tpl -Parent) $Name }
if (Test-Path $Dest) { throw "Thu muc da ton tai: $Dest" }

# Ten target CMake chi gom chu, so, dau gach duoi
$id = $Name -replace '[^A-Za-z0-9_]', '_'

Write-Host ">> Sao chep template -> $Dest"
robocopy $tpl $Dest /E /XD build .git drivers .cache /XF *.elf *.bin *.hex *.map /NFL /NDL /NJH /NJS /NP | Out-Null
if ($LASTEXITCODE -ge 8) { throw "robocopy loi (ma $LASTEXITCODE)" }
$global:LASTEXITCODE = 0

Write-Host ">> Doi ten project: h562_app -> $id"
$utf8 = New-Object System.Text.UTF8Encoding($false)
foreach ($f in "CMakeLists.txt", ".vscode\tasks.json", ".vscode\launch.json") {
    $p = Join-Path $Dest $f
    $c = [System.IO.File]::ReadAllText($p)
    [System.IO.File]::WriteAllText($p, ($c -replace 'h562_app', $id), $utf8)
}

Set-Location $Dest
git init | Out-Null
& powershell -ExecutionPolicy Bypass -File (Join-Path $Dest "scripts\setup.ps1")

# Dung dung phien ban driver giong template
foreach ($sub in "cmsis_core", "cmsis-device-h5", "stm32h5xx-hal-driver") {
    $src = Join-Path $tpl "drivers\$sub"
    if (Test-Path $src) {
        $rev = (git -C $src rev-parse HEAD).Trim()
        git -C "drivers\$sub" checkout -q $rev
        Write-Host ">> drivers/$sub @ $($rev.Substring(0, 8))"
    }
}

git add -A
git commit -q -m "Initial project from h562-template" 2>$null
if ($LASTEXITCODE -ne 0) { Write-Warning "Chua commit duoc (chua cau hinh git user.name/user.email?) - ban tu commit sau." }

Write-Host "`nXong: $Dest"
Write-Host "Mo bang VS Code:  code `"$Dest`""
