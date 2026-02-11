# Calypso SoC QEMU Machine — Install Package

Émulation du SoC TI Calypso (ARM946E-S) pour QEMU 9.2.
Deux machines : `calypso-min` (stubs) et `calypso-high` (INTH, timers, keypad, ABB, TRX bridge).

## Installation rapide

```bash
unzip calypso-qemu.zip
cd calypso-pkg
chmod +x install.sh
./install.sh ~/qemu
```

Puis rebuild complet :

```bash
cd ~/qemu
rm -rf build && mkdir build && cd build
../configure --target-list=arm-softmmu
ninja
```

Vérifier :

```bash
./qemu-system-arm -M help | grep calypso
```

## Contenu du package

```
calypso-pkg/
├── hw/arm/calypso/
│   ├── calypso.c            # Machine calypso-min (stubs)
│   ├── calypso_high.c       # Machine calypso-high (QOM instantiation)
│   ├── calypso_trx.c        # DSP/TPU/TRX bridge — virtual GSM PHY
│   ├── calypso_trx.h        # Header (IRQ map, DSP API, constantes)
│   ├── calypso_inth.{c,h}   # QOM: Interrupt controller (SysBusDevice)
│   ├── calypso_timer.{c,h}  # QOM: GP/Watchdog timer (SysBusDevice)
│   ├── calypso_uart.{c,h}   # QOM: UART + 64B FIFO (SysBusDevice)
│   ├── calypso_spi.{c,h}    # QOM: SPI + TWL3025 ABB (SysBusDevice)
│   ├── meson.build           # Build config
│   └── Kconfig               # Kernel config
├── test-firmware/
│   ├── test-firmware.c       # Firmware de test bare-metal
│   ├── linker.ld             # Linker script (RAM @ 0x00800000)
│   └── Makefile              # Build firmware (arm-none-eabi-gcc)
├── tools/
│   ├── trx.py                # TRX Bridge — pont osmo-bts-trx <-> QEMU
│   └── trx_test.py           # Outil de test loopback TRX UDP
├── docs/
│   ├── REGISTERS.md           # Référence des registres périphériques
│   ├── README-TRX.md          # Documentation TRX bridge + architecture
│   ├── MIGRATION.md           # Guide migration QEMU 9.2
│   └── PATCH-INFO.txt         # Info patch pour upstream
├── install.sh                # Installation automatique
├── uninstall.sh              # Désinstallation
└── README.md                 # Ce fichier
```

## Machines disponibles

| Feature          | calypso-min  | calypso-high  |
|------------------|--------------|---------------|
| Internal RAM     | 256K         | 256K          |
| External RAM     | —            | 8 MiB         |
| INTH (IRQ ctrl)  | —            | ✓ 32 IRQs     |
| Timer 1 w/ IRQ   | stub         | ✓ real ticks  |
| Timer 2 w/ IRQ   | —            | ✓ real ticks  |
| Keypad w/ IRQ    | —            | ✓ press/release|
| SPI + TWL3025    | stub         | ✓ register R/W|
| UART (×2)        | stub         | ✓ FIFO + IRQ  |
| DSP API RAM      | —            | ✓ 64 KiB      |
| TPU / TSP / ULPD | —            | ✓ stubs       |
| TRX UDP bridge   | —            | ✓ port 4729   |
| FCCH/SCH sync    | —            | ✓ simulation  |
| Flash            | 4 MiB        | 4 MiB         |

## Utilisation

### Lancement simple

```bash
qemu-system-arm -M calypso-high -cpu arm946 \
  -kernel firmware.elf \
  -serial pty -monitor stdio -nographic -s -S
```

### Workflow complet (3 terminaux)

**Terminal 1 — QEMU :**
```bash
sudo qemu-system-arm -M calypso-high -cpu arm946 \
  -kernel ~/trx/src/target/firmware/board/compal_e88/loader.highram.elf \
  -serial pty -monitor stdio -nographic -s -S
```
Note le `/dev/pts/X` affiché.

**Terminal 2 — socat + osmocon :**
```bash
sudo socat -x open:/dev/pts/X,raw,echo=0 pty,raw,echo=0,link=/tmp/serial1
```

Puis dans un autre terminal ou via Docker :
```bash
osmocon -p /tmp/serial1 -m c123xor layer1.highram.bin
```

**Terminal 1 — Démarrer le CPU :**
```
(qemu) c
```

### TRX Bridge (pont vers osmo-bts-trx)

```bash
# Terminal dédié
python3 tools/trx.py
```

Voir `docs/README-TRX.md` pour la documentation complète de l'architecture TRX.

### Test loopback TRX (sans BTS)

```bash
python3 tools/trx_test.py --loopback
```

### Debug GDB

```bash
arm-none-eabi-gdb firmware.elf
(gdb) target remote localhost:1234
(gdb) c
```

### Simuler appui power (GDB)

```
^C
set *(unsigned short *)0xFFFE4806 = 0x0001
set *(unsigned short *)0xFFFE4808 = 0x0001
c
```

### Compiler le firmware de test

```bash
cd test-firmware
make
make test    # lancer avec QEMU
make debug   # lancer avec GDB
```

## Désinstallation

```bash
./uninstall.sh ~/qemu
cd ~/qemu && rm -rf build && mkdir build && cd build
../configure --target-list=arm-softmmu && ninja
```

## Dépannage

**`arm_load_kernel` undefined :** Le meson.build du sous-dossier calypso ne doit PAS
contenir `arm_ss = ss.source_set()` ni `hw_arch +=`. Relancer `install.sh` corrige ça.

**`g_str_has_suffix TYPE_MACHINE_SUFFIX` assertion :** Le nom de la machine doit finir
par `-machine`. On utilise `MACHINE_TYPE_NAME()` qui fait ça automatiquement.

**`No such file: hw/arm/calypso/Kconfig` :** Le chemin dans `hw/arm/Kconfig` doit être
`source calypso/Kconfig` (relatif), pas `source hw/arm/calypso/Kconfig`. Relancer
`install.sh` corrige ça.

**`unknown type uint64_t` :** `qemu/osdep.h` doit être le PREMIER include dans chaque .c.

**DSP API version check fails :** Le firmware lit `DSP_DL_STATUS` à 0xFFD00000.
`calypso_trx.c` initialise la RAM DSP avec `status=0x0002` et `version=0x3606`.

## Documentation complémentaire

| Document              | Description                                      |
|-----------------------|--------------------------------------------------|
| `docs/README-TRX.md`  | Architecture TRX, data flow, packet format       |
| `docs/REGISTERS.md`   | Référence registres (memory map, UART, SPI, etc) |
| `docs/MIGRATION.md`   | Migration depuis anciennes versions QEMU         |
| `docs/PATCH-INFO.txt` | Informations patch pour soumission upstream       |
