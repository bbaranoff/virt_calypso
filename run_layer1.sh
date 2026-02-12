#!/usr/bin/env bash
set -e

# ================= CONFIG =================

QEMU=~/qemu/build/qemu-system-arm
OSMO_FW=~/board/compal_e88
OSMOCON=~/osmocom-bb/src/host/osmocon/osmocon
TRX_TEST=~/calypso-pkg/tools/trx_test.py

UART_SYMLINK=/tmp/calypso_uart
PTY_LINK=/tmp/calypso_serial
QEMU_LOG=/tmp/qemu.log

# ========================================

echo "[*] Cleanup old PTYs..."
rm -f "$UART_SYMLINK" "$PTY_LINK" "$QEMU_LOG"

echo "[*] Starting QEMU (calypso-high, daemon)..."

sudo $QEMU -M calypso-high -cpu arm946 \
  -kernel "$OSMO_FW/loader.highram.elf" \
  -chardev pty,id=uart0,path=$UART_SYMLINK \
  -serial chardev:uart0 \
  -display none \
  -s \
  -daemonize \
  -D "$QEMU_LOG"

sleep 1

if [ ! -e "$UART_SYMLINK" ]; then
    echo "[-] UART PTY not created"
    exit 1
fi

echo "[+] UART PTY is $UART_SYMLINK"

echo "[*] Creating socat bridge..."

sudo socat -d -d \
  open:$UART_SYMLINK,raw,echo=0 \
  pty,raw,echo=0,link=$PTY_LINK &

SOCAT_PID=$!

sleep 1

echo "[*] Loading Layer1 via osmocon..."

sudo $OSMOCON -p $PTY_LINK -m c123xor \
  "$OSMO_FW/layer1.highram.bin" &

OSMOCON_PID=$!

sleep 2

echo "[*] Starting TRX loopback..."

python3 $TRX_TEST --loopback &
TRX_PID=$!

echo
echo "=========================================="
echo " Layer1 running"
echo
echo " UART link : $UART_SYMLINK"
echo " Serial   : $PTY_LINK"
echo
echo " socat PID   : $SOCAT_PID"
echo " osmocon PID : $OSMOCON_PID"
echo " trx PID     : $TRX_PID"
echo "=========================================="
echo
echo "Press ENTER to stop."
read

echo "[*] Stopping..."

kill $TRX_PID $OSMOCON_PID $SOCAT_PID || true
sudo pkill -f "qemu-system-arm.*calypso-high" || true

wait || true
echo "[+] Done."
