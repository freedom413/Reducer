param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$cmakeRoot = Join-Path $env:LOCALAPPDATA "stm32cube\bundles\cmake"
$ninjaRoot = Join-Path $env:LOCALAPPDATA "stm32cube\bundles\ninja"
$gccRoot = Join-Path $env:LOCALAPPDATA "stm32cube\bundles\gnu-tools-for-stm32"

function Find-BundledTool {
    param(
        [string]$Root,
        [string]$RelativePath
    )

    if (-not (Test-Path -LiteralPath $Root)) {
        throw "STM32Cube bundle directory not found: $Root"
    }

    $tool = Get-ChildItem -LiteralPath $Root -Directory |
        ForEach-Object { Join-Path $_.FullName $RelativePath } |
        Where-Object { Test-Path -LiteralPath $_ } |
        Sort-Object { (Get-Item -LiteralPath $_).LastWriteTimeUtc } -Descending |
        Select-Object -First 1
    if (-not $tool) {
        throw "STM32Cube bundled tool not found under: $Root"
    }
    return $tool
}

$cmake = Find-BundledTool $cmakeRoot "bin\cmake.exe"
$ninja = Find-BundledTool $ninjaRoot "bin\ninja.exe"
$gcc = Find-BundledTool $gccRoot "bin\arm-none-eabi-gcc.exe"
$gccBin = Split-Path -Parent $gcc
$buildDir = Join-Path $repoRoot "build\$Configuration"
$toolchain = Join-Path $repoRoot "cmake\gcc-arm-none-eabi.cmake"

$env:PATH = "$gccBin;$env:PATH"

Write-Host "CMake: $cmake"
Write-Host "Ninja: $ninja"
Write-Host "GCC:   $gcc"

& $cmake --fresh -S $repoRoot -B $buildDir -G Ninja `
    "-DCMAKE_MAKE_PROGRAM:FILEPATH=$ninja" `
    "-DCMAKE_TOOLCHAIN_FILE:FILEPATH=$toolchain" `
    "-DCMAKE_BUILD_TYPE=$Configuration"
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $cmake --build $buildDir --config $Configuration
exit $LASTEXITCODE
