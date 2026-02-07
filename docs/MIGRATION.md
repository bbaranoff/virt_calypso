# Guide de migration vers QEMU 9.2

Ce document explique les changements entre l'ancienne version et la nouvelle version compatible QEMU 9.2.

## Principales modifications

### 1. Création du CPU

**Ancienne version (QEMU < 9.0):**
```c
s->cpu = ARM_CPU(cpu_create(ms->cpu_type ? ms->cpu_type : ARM_CPU_TYPE_NAME("arm946")));
```

**Nouvelle version (QEMU 9.2):**
```c
Object *cpuobj = object_new(ms->cpu_type);
s->cpu = ARM_CPU(cpuobj);

if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
    error_report_err(err);
    exit(1);
}
```

**Raison:** L'API `cpu_create()` a été dépréciée. QEMU 9.x utilise le pattern object_new + qdev_realize.

### 2. Gestion des erreurs

**Ancienne version:**
```c
memory_region_init_ram(&s->ram, NULL, "calypso.ram", ms->ram_size, &error_fatal);
```

**Nouvelle version:**
```c
Error *err = NULL;
memory_region_init_ram(&s->ram, NULL, "calypso.ram", ms->ram_size, &error_fatal);
// Plus de vérifications explicites avec Error *err où nécessaire
```

**Raison:** Meilleure gestion des erreurs et conformité avec les standards QEMU modernes.

### 3. Headers includes

**Ancienne version:**
```c
#include "target/arm/cpu.h"
```

**Nouvelle version:**
```c
#include "cpu.h"
#include "qapi/error.h"
#include "hw/char/serial-mm.h"
```

**Raison:** Réorganisation des headers dans QEMU 9.x.

### 4. Flash CFI

**Ancienne version:**
```c
pflash_cfi01_register(CALYPSO_FLASH_BASE, "calypso.flash", CALYPSO_FLASH_SIZE,
                      dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                      FLASH_SECTOR_SIZE, 4, 
                      0x0089, 0x8871, 0x0089, 0x8871, 0);
```

**Nouvelle version:**
```c
pflash_cfi01_register(CALYPSO_FLASH_BASE, "calypso.flash", 
                      CALYPSO_FLASH_SIZE,
                      dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                      FLASH_SECTOR_SIZE, 1,  // <- Changé de 4 à 1
                      0x0089, 0x8871, 0x0089, 0x8871, 0);
```

**Raison:** Paramètre de largeur de bus corrigé (1 = 16-bit).

### 5. Utilisation de cpu_set_pc

**Ancienne version:**
```c
cpu_set_pc(CPU(s->cpu), entry);
```

**Nouvelle version:**
```c
CPUState *cs = CPU(s->cpu);
cpu_set_pc(cs, entry);
```

**Raison:** Clarté du code et conformité aux standards QEMU.

### 6. Byte swapping

**Ancienne version:**
```c
uint32_t flash_addr = tswap32(CALYPSO_FLASH_BASE);
```

**Nouvelle version:**
```c
uint32_t flash_addr = cpu_to_le32(CALYPSO_FLASH_BASE);
```

**Raison:** `cpu_to_le32()` est la fonction standard QEMU, plus claire et portable.

## Checklist de migration

- [ ] Remplacer cpu_create() par object_new() + qdev_realize()
- [ ] Vérifier tous les includes
- [ ] Ajouter Error *err où nécessaire
- [ ] Mettre à jour les appels à pflash_cfi01_register
- [ ] Utiliser cpu_to_le32 au lieu de tswap32
- [ ] Tester la compilation avec QEMU 9.2
- [ ] Tester le chargement du firmware
- [ ] Vérifier les logs

## Erreurs de compilation courantes

### Erreur: "implicit declaration of function 'cpu_create'"

**Solution:** Remplacer par le nouveau pattern object_new + qdev_realize.

### Erreur: "unknown type name 'ARMCPU'"

**Solution:** Ajouter `#include "cpu.h"` au lieu de `#include "target/arm/cpu.h"`.

### Erreur: "undefined reference to serial_mm_init"

**Solution:** Ajouter `#include "hw/char/serial-mm.h"`.

### Warning: "implicit function declaration cpu_set_pc"

**Solution:** S'assurer que `#include "cpu.h"` est présent.

## Tests de régression

Après migration, vérifier:

1. **Compilation:**
   ```bash
   cd qemu-build
   ninja
   ```

2. **Détection de la machine:**
   ```bash
   ./qemu-system-arm -M help | grep calypso
   ```

3. **Chargement basique:**
   ```bash
   ./qemu-system-arm -M calypso-min -nographic
   ```

4. **Chargement firmware:**
   ```bash
   ./qemu-system-arm -M calypso-min -kernel firmware.elf -nographic
   ```

5. **Vérification du patch flash:**
   ```bash
   ./qemu-system-arm -M calypso-min -kernel firmware.elf -d guest_errors
   # Vérifier qu'aucune erreur n'est rapportée
   ```

## Améliorations futures

Suggestions pour améliorer le support:

1. **Interruptions:** Implémenter le contrôleur d'interruptions
2. **DMA:** Ajouter le support DMA
3. **Timers complets:** Implémenter les fonctionnalités complètes des timers
4. **Real UART:** Utiliser un vrai périphérique série au lieu du stub
5. **Device tree:** Ajouter le support device tree
6. **Tests unitaires:** Créer des tests qtest

## Ressources

- [QEMU 9.2 Release Notes](https://wiki.qemu.org/ChangeLog/9.2)
- [QEMU Developer Guide](https://qemu.readthedocs.io/en/latest/)
- [ARM CPU API](https://qemu.readthedocs.io/en/latest/devel/qom.html)

## Support

En cas de problème:

1. Vérifier la version QEMU: `qemu-system-arm --version`
2. Consulter les logs de compilation
3. Activer le logging détaillé: `-d guest_errors,cpu`
4. Comparer avec d'autres machines ARM dans `hw/arm/`
