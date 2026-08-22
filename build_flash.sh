#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
UF2_FILE="$BUILD_DIR/drone.uf2"

PICOTOOL="$HOME/.pico-sdk/picotool/2.3.0/picotool/picotool"

echo "========================================"
echo " Raspberry Pi Pico 2 Build & Flash"
echo "========================================"

cd "$PROJECT_DIR"

# ==================================================
# 1. Configure
# ==================================================

echo
echo "[1/4] Configuring CMake..."

cmake \
    -S "$PROJECT_DIR" \
    -B "$BUILD_DIR" \
    -DPICO_BOARD=pico2

# ==================================================
# 2. Build
# ==================================================

echo
echo "[2/4] Building..."

cmake \
    --build "$BUILD_DIR" \
    -j"$(nproc)"

# ==================================================
# 3. Check UF2
# ==================================================

echo
echo "[3/4] Checking UF2..."

if [ ! -f "$UF2_FILE" ]; then
    echo
    echo "ERROR: UF2 file was not generated."
    echo "Expected:"
    echo "$UF2_FILE"
    exit 1
fi

echo "UF2 found:"
echo "$UF2_FILE"

# ==================================================
# 4. Flash
# ==================================================

echo
echo "[4/4] Flashing Pico 2..."

if [ ! -x "$PICOTOOL" ]; then
    echo
    echo "ERROR: picotool not found:"
    echo "$PICOTOOL"
    exit 1
fi

echo "Using:"
echo "$PICOTOOL"

# --------------------------------------------------
# Try picotool
# --------------------------------------------------

if "$PICOTOOL" load "$UF2_FILE" -f; then

    echo
    echo "Firmware loaded successfully."

else

    echo
    echo "picotool could not access the Pico."
    echo "Checking for BOOTSEL drive..."

    PICO_DRIVE=""

    for drive in \
        /media/"$USER"/* \
        /run/media/"$USER"/*; do

        if [ -f "$drive/INFO_UF2.TXT" ]; then
            PICO_DRIVE="$drive"
            break
        fi

    done

    if [ -z "$PICO_DRIVE" ]; then
        echo
        echo "ERROR: Pico 2 was not found."
        echo
        echo "Put the Pico 2 into BOOTSEL mode:"
        echo "  1. Disconnect Pico"
        echo "  2. Hold BOOTSEL"
        echo "  3. Connect Pico"
        echo "  4. Release BOOTSEL"
        exit 1
    fi

    echo
    echo "BOOTSEL drive found:"
    echo "$PICO_DRIVE"

    cp "$UF2_FILE" "$PICO_DRIVE/"

    sync

    echo
    echo "UF2 copied successfully."

fi

echo
echo "========================================"
echo " Build and Flash Successful!"
echo "========================================"
echo
echo "Firmware:"
echo "$UF2_FILE"
echo