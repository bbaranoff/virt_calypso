#!/bin/bash
# uninstall.sh — Remove Calypso machines from QEMU source tree
# Usage: ./uninstall.sh [/path/to/qemu]

set -e

QEMU_DIR="${1:-$HOME/qemu}"
CALYPSO_DIR="$QEMU_DIR/hw/arm/calypso"

echo "=== Calypso Uninstaller ==="
echo "QEMU source: $QEMU_DIR"
echo ""

# Remove calypso directory entirely
if [ -d "$CALYPSO_DIR" ]; then
    rm -rf "$CALYPSO_DIR"
    echo "Removed $CALYPSO_DIR"
fi

# Remove from hw/arm/meson.build
ARM_MESON="$QEMU_DIR/hw/arm/meson.build"
if [ -f "$ARM_MESON" ] && grep -q "calypso" "$ARM_MESON"; then
    sed -i "/calypso/d" "$ARM_MESON"
    echo "Cleaned hw/arm/meson.build"
fi

# Remove from hw/arm/Kconfig
ARM_KCONFIG="$QEMU_DIR/hw/arm/Kconfig"
if [ -f "$ARM_KCONFIG" ] && grep -q "calypso" "$ARM_KCONFIG"; then
    sed -i "/calypso/d" "$ARM_KCONFIG"
    echo "Cleaned hw/arm/Kconfig"
fi

echo ""
echo "Done. Reconfigure and rebuild:"
echo "  cd $QEMU_DIR && rm -rf build && mkdir build && cd build"
echo "  ../configure --target-list=arm-softmmu"
echo "  ninja"
