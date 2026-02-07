# Calypso SoC QEMU Machine — Install Package

Émulation du SoC TI Calypso (ARM946E-S) pour QEMU 9.2.
Deux machines : `calypso-min` (stubs) et `calypso-high` (INTH, timers, keypad, ABB).

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
│   ├── calypso.c           # Machine calypso-min (stubs)
│   ├── calypso_high.c      # Machine calypso-high (INTH, timers, keypad, ABB)
│   ├── meson.build         # Build config
│   └── Kconfig             # Kernel config
├── test-firmware/
│   ├── test-firmware.c     # Firmware de test
│   ├── linker.ld           # Linker script
│   └── Makefile            # Build firmware
├── docs/
│   ├── REGISTERS.md        # Référence des registres
│   ├── MIGRATION.md        # Guide migration QEMU 9.2
│   └── PATCH-INFO.txt      # Info patch
├── install.sh              # Installation automatique
├── uninstall.sh            # Désinstallation
└── README.md               # Ce fichier
```

## Machines disponibles

| Feature          | calypso-min | calypso-high |
|------------------|-------------|--------------|
| Internal RAM     | 256K        | 256K         |
| External RAM     | —           | 8 MiB        |
| INTH (IRQ ctrl)  | —           | ✓ 32 IRQs   |
| Timer 1 w/ IRQ   | stub        | ✓ real ticks |
| Timer 2 w/ IRQ   | —           | ✓ real ticks |
| Keypad w/ IRQ    | —           | ✓ press/release |
| SPI + TWL3025    | stub        | ✓ register R/W |
| Flash            | 4 MiB       | 4 MiB        |

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
