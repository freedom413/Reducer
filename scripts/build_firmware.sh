#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STM32_BUNDLES_DIR="${STM32_BUNDLES_DIR:-$HOME/.local/share/stm32cube/bundles}"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build/stm32cube-debug}"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
TOOLCHAIN_FILE="${TOOLCHAIN_FILE:-$ROOT_DIR/cmake/gcc-arm-none-eabi.cmake}"

find_latest_dir() {
    local parent="$1"
    if [[ ! -d "$parent" ]]; then
        return 1
    fi
    find "$parent" -mindepth 1 -maxdepth 1 -type d | sort -V | tail -n 1
}

if [[ -n "${ARM_GCC_BIN_DIR:-}" ]]; then
    gcc_bin_dir="$ARM_GCC_BIN_DIR"
else
    gcc_root="$(find_latest_dir "$STM32_BUNDLES_DIR/gnu-tools-for-stm32")"
    gcc_bin_dir="$gcc_root/bin"
fi

if [[ -n "${STM32_CMAKE_BIN_DIR:-}" ]]; then
    cmake_bin_dir="$STM32_CMAKE_BIN_DIR"
else
    cmake_root="$(find_latest_dir "$STM32_BUNDLES_DIR/cmake")"
    cmake_bin_dir="$cmake_root/bin"
fi

if [[ ! -x "$gcc_bin_dir/arm-none-eabi-gcc" ]]; then
    echo "arm-none-eabi-gcc not found: $gcc_bin_dir/arm-none-eabi-gcc" >&2
    echo "Set ARM_GCC_BIN_DIR or install STM32Cube bundles first." >&2
    exit 1
fi

if [[ ! -x "$cmake_bin_dir/cmake" ]]; then
    echo "cmake not found: $cmake_bin_dir/cmake" >&2
    echo "Set STM32_CMAKE_BIN_DIR or install STM32Cube bundles first." >&2
    exit 1
fi

mkdir -p "$BUILD_DIR"

export PATH="$gcc_bin_dir:$cmake_bin_dir:$PATH"

echo "Using ARM toolchain: $gcc_bin_dir/arm-none-eabi-gcc"
echo "Using CMake:        $cmake_bin_dir/cmake"
echo "Build directory:    $BUILD_DIR"

"$cmake_bin_dir/cmake" \
    -S "$ROOT_DIR" \
    -B "$BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

"$cmake_bin_dir/cmake" --build "$BUILD_DIR" -j"$(nproc)"
