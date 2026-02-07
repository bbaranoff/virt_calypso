# Calypso TRX Bridge — Virtual GSM PHY

## Architecture

```
┌─────────────────────────────────────────────┐
│  QEMU (calypso-high machine)                │
│                                             │
│  ┌──────────┐     ┌───────────────────┐     │
│  │ OsmocomBB│     │ calypso_trx.c     │     │
│  │ L1 firm- │────▶│                   │     │
│  │ ware     │     │ DSP API RAM       │     │
│  │          │◀────│ TPU / TSP stubs   │     │
│  │ (ARM946) │     │ ULPD stub         │     │
│  └──────────┘     │ TDMA timer        │     │
│                   │ TRX UDP socket ───────────── UDP :4729
│                   └───────────────────┘     │
└─────────────────────────────────────────────┘
                        │
                   TRX protocol (TRXD v0)
                        │
                        ▼
              ┌──────────────────┐
              │  osmo-bts-trx    │
              │  (or loopback    │
              │   test tool)     │
              └──────────────────┘
                        │
                        ▼
              ┌──────────────────┐
              │  osmo-bsc        │
              │  osmo-msc        │
              │  osmo-hlr        │
              └──────────────────┘
```

## What's in the box

### New file: `calypso_trx.c` (~500 lines)

| Component       | Address        | Function                                  |
|-----------------|----------------|-------------------------------------------|
| DSP API RAM     | 0xFFD00000     | 64 KiB shared memory (ARM↔DSP)            |
| TPU             | 0xFFFE0000     | Time Processing Unit — hooks CTRL enable  |
| TSP             | 0xFFFE0800     | Time Serial Port — RF control stub        |
| ULPD            | 0xFFFE2800     | Ultra Low Power Down — clocks, gauging    |
| TDMA timer      | (internal)     | 4.615 ms frame clock → IRQ 4              |
| DSP completion  | (internal)     | Delayed IRQ 15 after TPU enable           |
| TRX UDP socket  | port 4729      | Send TX bursts / receive RX bursts        |

### Fixed IRQ map (now matches real Calypso)

| IRQ | Source          | Old # | New # |
|-----|-----------------|-------|-------|
| 1   | Timer 1         | 3     | 1     |
| 2   | Timer 2         | 4     | 2     |
| 4   | TPU Frame       | —     | 4     |
| 5   | TPU Page        | —     | 5     |
| 7   | UART Modem      | 9     | 7     |
| 8   | Keypad/GPIO     | 5     | 8     |
| 13  | SPI             | 7     | 13    |
| 15  | DSP API         | —     | 15    |
| 18  | UART IrDA       | 2     | 18    |

## Build

```bash
cd ~/calypso-pkg
./install.sh ~/qemu

cd ~/qemu
rm -rf build && mkdir build && cd build
../configure --target-list=arm-softmmu
ninja
```

Verify:
```bash
./qemu-system-arm -M help | grep calypso
# calypso-high   Calypso SoC (highram) with INTH, timers, keypad, ABB
# calypso-min    Calypso SoC minimal machine
```

## Run — Basic test (no TRX)

```bash
qemu-system-arm -M calypso-high -cpu arm946 \
  -kernel ~/trx/src/target/firmware/board/compal_e88/loader.highram.elf \
  -serial pty -monitor stdio -nographic
```

You should see in stderr:
```
[calypso-trx] === Calypso TRX bridge init ===
[calypso-trx] DSP API RAM initialized (64 KiB at 0xffd00000)
[calypso-trx] TRX UDP listening on port 4729 (send to 4829)
[calypso-trx] TDMA started (4.615ms frame timer)
[calypso-trx] === TRX bridge ready ===
```

## Run — With GDB (recommended for debugging)

### Terminal 1: QEMU

```bash
qemu-system-arm -M calypso-high -cpu arm946 \
  -kernel ~/trx/src/target/firmware/board/compal_e88/loader.highram.elf \
  -serial pty -monitor stdio -nographic -s -S
```

### Terminal 2: GDB

```bash
arm-none-eabi-gdb ~/trx/src/target/firmware/board/compal_e88/loader.highram.elf
```

```gdb
target remote localhost:1234

# Break on DSP init
break dsp_init
# Break on TPU init
break tpu_init
# Break on L1 frame handler
break l1s_compl

continue
```

### Terminal 1: Start CPU

```
(qemu) c
```

## How it works — Data flow

### Each TDMA frame (every 4.615 ms):

```
1. QEMU timer fires
   └─▶ Raises IRQ 4 (TPU_FRAME)

2. L1 ISR runs in firmware
   ├─▶ Reads DSP results from API RAM read page
   ├─▶ Writes new DSP tasks to API RAM write page
   ├─▶ Programs TPU scenario (writes to TPU RAM)
   └─▶ Enables TPU (writes TPU_CTRL bit 0)

3. calypso_trx.c intercepts TPU enable
   ├─▶ Reads TX burst from API RAM write page
   ├─▶ Sends via TRX UDP socket
   ├─▶ Checks for pending RX burst from UDP
   ├─▶ Writes RX data to API RAM read page
   └─▶ Schedules IRQ 15 (API) after 10 µs delay

4. L1 processes DSP completion on next frame
```

### TRX UDP packet format (TRXD v0)

**TX (phone → network):**
```
Byte 0:    Timeslot number (0-7)
Bytes 1-4: Frame number (big-endian uint32)
Byte 5:    TX power level
Bytes 6+:  148 burst bits (0 or 1)
Total: 154 bytes
```

**RX (network → phone):**
```
Byte 0:    Timeslot number (0-7)
Bytes 1-4: Frame number (big-endian uint32)
Byte 5:    RSSI (signed dBm)
Bytes 6-7: TOA (timing of arrival, signed int16)
Bytes 8+:  148 soft-bits (0=strong 0, 255=strong 1)
Total: 156 bytes
```

## Debugging — Identify DSP API offsets

The burst data offsets depend on the DSP firmware version. To identify them:

### Method 1: GDB watchpoints

```gdb
# Watch writes to DSP API write page 0
watch *(uint16_t *)0xFFD00000
# Watch writes to DSP API write page 1
watch *(uint16_t *)0xFFD01000
# Watch writes to TPU CTRL (DSP trigger)
watch *(uint16_t *)0xFFFE0000
```

### Method 2: Enable verbose logging

Edit `calypso_trx.c` and set:
```c
#define TRX_DEBUG_DSP    1
#define TRX_DEBUG_TPU    1
```

Rebuild and run — every DSP/TPU access is logged to stderr.

### Method 3: Dump DSP API from GDB

```gdb
# After L1 has been running for a few frames, pause and dump:
^C
# Dump write page 0 (first 128 bytes)
x/64xh 0xFFD00000
# Dump NDB area
x/64xh 0xFFD04000
# Check TPU registers
x/8xh 0xFFFE0000
```

## Loopback test (no BTS needed)

Send a fake RX burst to the phone:

```python
#!/usr/bin/env python3
"""Send a test burst to QEMU Calypso via TRX UDP"""
import socket, struct, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# RX burst: TN=0, FN=100, RSSI=-60, TOA=0, then 148 soft-bits
tn = 0
fn = 100
rssi = 196  # -60 dBm unsigned
toa = 0
# All-ones normal burst (strong 1s)
bits = bytes([255] * 148)

pkt = struct.pack('>BIBH', tn, fn, rssi, toa) + bits
sock.sendto(pkt, ('127.0.0.1', 4729))
print(f"Sent RX burst: TN={tn} FN={fn} len={len(pkt)}")
```

## Next steps

### Phase 1 (current): Infrastructure ✓
- [x] DSP API RAM at 0xFFD00000
- [x] TPU stub with CTRL hook
- [x] TSP stub
- [x] ULPD stub (clocks, gauging)
- [x] TDMA frame timer (4.615 ms → IRQ 4)
- [x] DSP completion (→ IRQ 15)
- [x] TRX UDP socket
- [x] Correct IRQ numbers

### Phase 2: Firmware boot
- [ ] Run firmware with debug logging, identify hang points
- [ ] Add missing MMIO stubs as needed (firmware will crash on unmapped access)
- [ ] Verify DSP init sequence works (firmware reads DSP ID from API RAM)
- [ ] Verify clock init (ULPD gauging)

### Phase 3: Burst extraction
- [ ] Identify exact burst data offsets in API RAM (using GDB/logging)
- [ ] Refine `calypso_dsp_process()` to extract real burst bits
- [ ] Verify TX bursts appear on UDP
- [ ] Verify RX bursts reach L1

### Phase 4: Integration
- [ ] Connect to osmo-bts-trx via TRX protocol
- [ ] Verify RACH / channel assignment
- [ ] End-to-end: QEMU phone → osmo-bts → osmo-bsc → call/SMS

## File reference

| File              | Lines | Function                           |
|-------------------|-------|------------------------------------|
| calypso_trx.h     | ~140  | Constants, IRQ map, public API     |
| calypso_trx.c     | ~500  | DSP/TPU/TSP/ULPD/TDMA/TRX bridge  |
| calypso_high.c    | ~790  | Machine init (updated for TRX)     |
| calypso.c         | ~340  | calypso-min machine (unchanged)    |
