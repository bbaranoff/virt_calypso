#!/usr/bin/env bash
set -euo pipefail

# Ajuste si besoin
QEMU=qemu-system-arm
FW_DIR=~/calypso-pkg/test-firmware
OSMO_FW=~/board/compal_e88
TOOLS=~/calypso-pkg/tools

echo "============================================================"
echo "1. POST-BUILD CHECK"
echo "============================================================"

$QEMU -M help | grep calypso || true
$QEMU -device help | grep calypso || true

echo
echo "============================================================"
echo "2. CALYPSO-MIN (no IRQ)"
echo "============================================================"

# Boot simple (20 lignes max, timeout 3s)
timeout 3s $QEMU -M calypso-min -nographic -d cpu_reset 2>&1 | head -20 || true

# Test firmware
cd "$FW_DIR"
make
timeout 3s $QEMU -M calypso-min \
  -kernel test.elf \
  -nographic \
  -d guest_errors 2>&1 | head -30 || true

echo
echo "============================================================"
echo "3. CALYPSO-HIGH (devices)"
echo "============================================================"

timeout 3s $QEMU -M calypso-high -cpu arm946 \
  -nographic -d cpu_reset 2>&1 | head -20 || true

# UART PTY (juste pour voir apparaître le /dev/pts/X)
timeout 3s $QEMU -M calypso-high -cpu arm946 \
  -serial pty -nographic 2>&1 | grep "pts" || true

echo
echo "============================================================"
echo "4. OSMOCOMBB LOADER (dry run)"
echo "============================================================"

# QEMU en debug, mais coupé après 3s
timeout 3s $QEMU -M calypso-high -cpu arm946 \
  -kernel "$OSMO_FW/loader.highram.elf" \
  -serial null -nographic -s -S 2>&1 | head -20 || true

echo
echo "============================================================"
echo "5. TRX TEST (safe)"
echo "============================================================"

# Loopback 2s
timeout 2s python3 "$TOOLS/trx_test.py" --loopback 2>&1 | head -20 || true

# Inject 2s (peut échouer si RSSI hors plage — on ignore)
timeout 2s python3 "$TOOLS/trx_test.py" --inject 2>&1 | head -20 || true

echo
echo "============================================================"
echo "6. GDB SMOKE"
echo "============================================================"

# QEMU suspendu 2s
timeout 2s $QEMU -M calypso-high -cpu arm946 \
  -kernel firmware.elf \
  -serial null -nographic -s -S 2>&1 | head -10 || true

echo
echo "============================================================"
echo "7. MEMORY MAP"
echo "============================================================"

# Monitor non interactif
timeout 3s $QEMU -M calypso-high -cpu arm946 \
  -nographic -monitor stdio <<< "info mtree
quit" 2>&1 | grep -E "calypso|Timer|UART|INTH|SPI" || true

echo
echo "============================================================"
echo "8. QUICK SMOKE"
echo "============================================================"

timeout 3s $QEMU -M calypso-high -cpu arm946 \
  -nographic -monitor stdio -d guest_errors <<< "info mtree
info qtree
quit" 2>&1 | grep -E "calypso|Timer|UART|INTH|SPI" || true

echo
echo "DONE."
