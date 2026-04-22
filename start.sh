#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# --- Guards: check required tools before starting ---
for cmd in octave cmake make grep sed; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "ERROR: '$cmd' not found. Aborting." >&2
        exit 1
    fi
done

RENODE="/Applications/Renode.app/Contents/MacOS/Renode"
LUT_HEADER="src/includes/motor_lut.h"

# --- Step 1: Generate LUT with GNU Octave ---
echo "=== [1/6] Generating LUT (Octave)... ==="
(cd octave && octave --silent --eval "main; exit;")

if [ ! -f "$LUT_HEADER" ]; then
    echo "ERROR: Octave did not generate '$LUT_HEADER'. Aborting." >&2
    exit 1
fi
echo "OK: $LUT_HEADER generated."

# --- Step 2: Build for POSIX ---
echo "=== [2/6] Building for POSIX... ==="
rm -rf build
cmake -B build -S .
cmake --build build

if [ ! -f "build/test_motor" ]; then
    echo "ERROR: POSIX build failed — 'build/test_motor' not found. Aborting." >&2
    exit 1
fi

# --- Step 3: Run POSIX tests and capture trace ---
echo "=== [3/6] Running POSIX tests... ==="
rm -rf logs/*
mkdir -p logs
build/test_motor > logs/motor_test_posix.log

if ! grep -q "\[TRACE\]|" logs/motor_test_posix.log; then
    echo "ERROR: No [TRACE] lines found in POSIX log. Test binary may have crashed." >&2
    exit 1
fi

# For Validation we need the parsed test data. 
# test_x_parsed.csv identical to the platforms
# Parse POSIX trace
grep "\[TRACE\]|" logs/motor_test_posix.log \
  | sed 's/.*\[TRACE\]|//' \
  | tr -d ' ' > logs/test_posix_parsed.csv
echo "OK: logs/test_posix_parsed.csv created ($(wc -l < logs/test_posix_parsed.csv) lines)."

# --- Step 4: Build for STM32 ---
echo "=== [4/6] Building for STM32... ==="
rm -rf build_stm32
cmake -B build_stm32 -S . -DPLATFORM=stm32 -DCMAKE_TOOLCHAIN_FILE=arm-toolchain.cmake
cmake --build build_stm32

if [ ! -f "build_stm32/test_motor_stm32" ]; then
    echo "ERROR: STM32 build failed — ELF not found. Aborting." >&2
    exit 1
fi

# --- Step 5: Run emulation in Renode ---
echo "=== [5/6] Running Renode emulation... ==="
if [ ! -x "$RENODE" ]; then
    echo "WARNING: Renode not found at '$RENODE'. Skipping STM32 emulation." >&2
else
    "$RENODE" -e "logFile @$PWD/logs/motor_test_stm32.log; include @$PWD/renode/test_run.resc; mach set \"STM32F4\"; start"

    # Wait for Renode to finish writing the log (up to 10 seconds)
    for i in $(seq 1 10); do
        if grep -q "\[TRACE\]|" logs/motor_test_stm32.log 2>/dev/null; then
            break
        fi
        sleep 1
    done

    if ! grep -q "\[TRACE\]|" logs/motor_test_stm32.log 2>/dev/null; then
        echo "ERROR: No [TRACE] lines in Renode log after 10s. Emulation may have failed." >&2
        exit 1
    fi

    grep "\[TRACE\]|" logs/motor_test_stm32.log \
      | sed 's/.*\[TRACE\]|//' \
      | tr -d ' ' > logs/test_stm32_parsed.csv
    echo "OK: logs/test_stm32_parsed.csv created ($(wc -l < logs/test_stm32_parsed.csv) lines)."
fi

# --- Step 6: Validation plots with Octave ---
echo "=== [6/6] Generating validation plots (Octave)... ==="
(cd octave && octave --silent --eval "validation; exit;")
echo "OK: Plots saved to octave/plots/"

echo ""
echo "=== Pipeline finished successfully ==="