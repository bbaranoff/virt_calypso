#!/bin/bash
# install.sh — Install Calypso machines (min + high) into QEMU source tree
# Usage: ./install.sh [/path/to/qemu]

set -e

QEMU_DIR="${1:-$HOME/qemu}"
CALYPSO_DIR="$QEMU_DIR/hw/arm/calypso"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Calypso Installer ==="
echo "QEMU source: $QEMU_DIR"
echo ""

# Check QEMU dir exists
if [ ! -f "$QEMU_DIR/meson.build" ]; then
    echo "ERROR: $QEMU_DIR does not look like a QEMU source tree"
    echo "Usage: $0 /path/to/qemu"
    exit 1
fi

# Create calypso directory
mkdir -p "$CALYPSO_DIR"

# Copy source files
echo "[1/6] Copying source files ..."
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_trx.h" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_high.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_trx.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_inth.h" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_inth.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_timer.h" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_timer.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_uart.h" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_uart.c" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_spi.h" "$CALYPSO_DIR/"
cp "$SCRIPT_DIR/hw/arm/calypso/calypso_spi.c" "$CALYPSO_DIR/"
echo "       Copied all source files (machine + QOM peripherals)"

# Install meson.build (ALWAYS overwrite — old version was broken)
echo "[2/6] Installing meson.build ..."
# Check if old broken meson.build exists (one that creates its own arm_ss)
if [ -f "$CALYPSO_DIR/meson.build" ] && grep -q "hw_arch" "$CALYPSO_DIR/meson.build"; then
    echo "       WARNING: Replacing broken meson.build (was overriding arm_ss)"
fi
cp "$SCRIPT_DIR/hw/arm/calypso/meson.build" "$CALYPSO_DIR/"
echo "       Installed correct meson.build"

# Install Kconfig (ALWAYS overwrite — old version may be missing selects)
echo "[3/6] Installing Kconfig ..."
cp "$SCRIPT_DIR/hw/arm/calypso/Kconfig" "$CALYPSO_DIR/"
echo "       Installed Kconfig"

# Patch parent meson.build (hw/arm/meson.build)
echo "[4/6] Patching hw/arm/meson.build ..."
ARM_MESON="$QEMU_DIR/hw/arm/meson.build"
if grep -q "calypso" "$ARM_MESON"; then
    echo "       Already references calypso, skipping"
else
    echo "" >> "$ARM_MESON"
    echo "subdir('calypso')" >> "$ARM_MESON"
    echo "       Added subdir('calypso')"
fi

# Patch parent Kconfig (hw/arm/Kconfig)
echo "[5/6] Patching hw/arm/Kconfig ..."
ARM_KCONFIG="$QEMU_DIR/hw/arm/Kconfig"
if grep -q "calypso" "$ARM_KCONFIG"; then
    # Fix wrong path if present
    if grep -q "source hw/arm/calypso" "$ARM_KCONFIG"; then
        echo "       Fixing incorrect Kconfig path ..."
        sed -i 's|source hw/arm/calypso/Kconfig|source calypso/Kconfig|' "$ARM_KCONFIG"
    fi
    echo "       Already references calypso"
else
    echo "" >> "$ARM_KCONFIG"
    echo "source calypso/Kconfig" >> "$ARM_KCONFIG"
    echo "       Added source calypso/Kconfig"
fi

# Clean build directory if it exists and is broken
echo "[6/6] Checking build directory ..."
BUILD_DIR="$QEMU_DIR/build"
if [ -d "$BUILD_DIR" ]; then
    echo "       Build directory exists. You may need to reconfigure:"
    echo ""
    echo "         cd $QEMU_DIR"
    echo "         rm -rf build && mkdir build && cd build"
    echo "         ../configure --target-list=arm-softmmu"
    echo "         ninja"
else
    echo "       No build directory. Configure and build:"
    echo ""
    echo "         cd $QEMU_DIR && mkdir build && cd build"
    echo "         ../configure --target-list=arm-softmmu"
    echo "         ninja"
fi

echo ""
echo "=== Installation complete ==="
echo ""
echo "Machines installed:"
echo "  calypso-min   — Minimal (256K RAM, stubs)"
echo "  calypso-high  — Full (256K+8M RAM, INTH, timers, keypad, ABB)"
echo ""
echo "Quick test after build:"
echo "  qemu-system-arm -M help | grep calypso"
echo ""
echo "Run:"
echo "  qemu-system-arm -M calypso-high -cpu arm946 \\"
echo "    -kernel /path/to/loader.highram.elf \\"
echo "    -serial pty -monitor stdio -nographic -s -S"
