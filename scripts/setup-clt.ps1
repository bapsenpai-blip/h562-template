# Cấu hình STM32CubeCLT cho Windows (chạy MỘT lần, không cần quyền admin):
#   powershell -ExecutionPolicy Bypass -File scripts\setup-clt.ps1
# Tùy chọn chỉ định thư mục:  ... -File scripts\setup-clt.ps1 -CltPath "D:\ST\STM32CubeCLT_1.19.0"
param([string]$CltPath = "")
$ErrorActionPreference = "Stop"

if (-not $CltPath) { $CltPath = [Environment]::GetEnvironmentVariable("STM32CLT_PATH", "User") }
if (-not $CltPath) { $CltPath = [Environment]::GetEnvironmentVariable("STM32CLT_PATH", "Machine") }
if (-not $CltPath -or -not (Test-Path $CltPath)) {
    Write-Host ">> Đang tìm STM32CubeCLT trên ổ C: và D: ..."
    $found = Get-ChildItem -Path "D:\", "C:\" -Directory -Depth 2 -Filter "STM32CubeCLT*" -ErrorAction SilentlyContinue |
             Where-Object { Test-Path (Join-Path $_.FullName "GNU-tools-for-STM32") } |
             Sort-Object Name -Descending | Select-Object -First 1
    if ($null -eq $found) { throw "Không tìm thấy STM32CubeCLT. Chạy lại với -CltPath ""D:\đường\dẫn\STM32CubeCLT_x.y.z""" }
    $CltPath = $found.FullName
}
$CltPath = (Resolve-Path $CltPath).Path.TrimEnd('\')
Write-Host ">> STM32CubeCLT: $CltPath"

$bins = @(
    "GNU-tools-for-STM32\bin",
    "CMake\bin",
    "Ninja\bin",
    "STM32CubeProgrammer\bin",
    "STLink-gdb-server\bin"
) | ForEach-Object { Join-Path $CltPath $_ }

foreach ($b in $bins) { if (-not (Test-Path $b)) { Write-Warning "Thiếu thư mục: $b" } }

# Biến STM32CLT_PATH (cho launch.json)
[Environment]::SetEnvironmentVariable("STM32CLT_PATH", $CltPath, "User")

# Thêm vào PATH của user (bỏ các mục STM32CubeCLT cũ để tránh trùng phiên bản)
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
$parts = @()
if ($userPath) { $parts = $userPath.Split(';') | Where-Object { $_ -and ($_ -notmatch 'STM32CubeCLT') } }
$newPath = (@($bins) + $parts) -join ';'
[Environment]::SetEnvironmentVariable("Path", $newPath, "User")

Write-Host ">> Đã đặt STM32CLT_PATH và thêm vào PATH:"
$bins | ForEach-Object { Write-Host "   $_" }
Write-Host "`n!! ĐÓNG HẲN VS Code và mọi cửa sổ terminal rồi mở lại để nhận biến môi trường mới."

