# Référence des Registres Calypso

Documentation des registres et périphériques émulés dans la machine Calypso.

## Table des Matières

1. [Carte Mémoire Globale](#carte-mémoire-globale)
2. [UART](#uart)
3. [SPI](#spi)
4. [Timer](#timer)
5. [Périphériques MMIO](#périphériques-mmio)
6. [Alias Mémoire](#alias-mémoire)

---

## Carte Mémoire Globale

| Adresse Base | Taille | Nom | Type | Description |
|--------------|--------|-----|------|-------------|
| 0x00000000 | 128 KiB | RAM_ALIAS0 | RAM | Alias de la RAM principale |
| 0x00800000 | 256 KiB | RAM | RAM | RAM principale |
| 0x02000000 | 4 MiB | FLASH | Flash | NOR Flash CFI01 |
| 0xFFFE1800 | 256 B | 18XX | MMIO | Registres 8-bit |
| 0xFFFE2800 | 256 B | 28XX | MMIO | Registres 8-bit |
| 0xFFFE3000 | 256 B | SPI | MMIO | Interface SPI |
| 0xFFFE3800 | 256 B | TIMER1 | MMIO | Timer 1 |
| 0xFFFE4800 | 256 B | 48XX | MMIO | Registres 16-bit "ready" |
| 0xFFFE6800 | 256 B | 68XX | MMIO | Registres mixtes 8/16-bit |
| 0xFFFE8000 | 256 B | 80XX | MMIO | Registres 8-bit |
| 0xFFFEF000 | 256 B | F0XX | MMIO | Registres 16-bit |
| 0xFFFF5000 | 256 B | 50XX | MMIO | Registres 8-bit |
| 0xFFFF5800 | 256 B | UART | MMIO | UART stub |
| 0xFFFF9800 | 256 B | 98XX | MMIO | Registres 16-bit |
| 0xFFFFF900 | 256 B | F9XX | MMIO | Registres 16-bit |
| 0xFFFFFA00 | 256 B | FAXX | MMIO | Registres 16-bit |
| 0xFFFFFB00 | 256 B | SYSFB | MMIO | System FB - 16-bit |
| 0xFFFFFC00 | 256 B | FCXX | MMIO | Registres 16-bit |
| 0xFFFFFD00 | 256 B | SYSFD | MMIO | System FD - 16-bit |
| 0xFFFFFF00 | 256 B | FFXX | MMIO | Registres 8-bit |
| 0xFFFF0000 | 64 KiB | VECTORS_HI | Alias | Alias RAM pour vecteurs high |

---

## UART

**Adresse de base:** 0xFFFF5800  
**Taille:** 256 octets  
**Type:** 8-bit

### Registres

| Offset | Nom | R/W | Description |
|--------|-----|-----|-------------|
| 0x00 | TX/RX | R/W | Données transmission/réception |
| 0x11 | STATUS | R | Registre de status |

### Bits du registre STATUS (0x11)

| Bit | Nom | Description |
|-----|-----|-------------|
| 6-5 | TX_READY | Transmetteur prêt (toujours 0x3 = prêt) |

### Utilisation

```c
#define UART_BASE   0xFFFF5800
#define UART_DATA   (*(volatile uint8_t *)(UART_BASE + 0x00))
#define UART_STATUS (*(volatile uint8_t *)(UART_BASE + 0x11))

// Écriture d'un caractère
void uart_putc(char c) {
    UART_DATA = c;
}

// Lecture du status
uint8_t uart_status(void) {
    return UART_STATUS; // Retourne toujours 0x60 (prêt)
}
```

### Notes

- L'UART actuel est un stub simple
- Toutes les données écrites sont loggées sur la console QEMU
- Le status retourne toujours "prêt" (pas de polling bloquant)
- Pas d'implémentation de réception réelle

---

## SPI

**Adresse de base:** 0xFFFE3000  
**Taille:** 256 octets  
**Type:** 16-bit

### Registres

| Offset | Nom | R/W | Taille | Description |
|--------|-----|-----|--------|-------------|
| 0x06 | STATUS | R | 16-bit | Registre de status |
| 0x0A | DATA | R/W | 16-bit | Données SPI |

### Bits du registre STATUS (0x06)

| Bit | Nom | Description |
|-----|-----|-------------|
| 0 | TX_READY | Transmetteur prêt |
| 1 | RX_READY | Récepteur prêt |

### Utilisation

```c
#define SPI_BASE   0xFFFE3000
#define SPI_STATUS (*(volatile uint16_t *)(SPI_BASE + 0x06))
#define SPI_DATA   (*(volatile uint16_t *)(SPI_BASE + 0x0A))

// Envoi de données
void spi_send(uint16_t data) {
    SPI_DATA = data;
    // Après écriture, le status passe à ready et DATA devient 0xFFFF
}

// Lecture du status
uint16_t spi_status(void) {
    return SPI_STATUS; // Bit 0 et 1 = ready
}

// Lecture de données
uint16_t spi_receive(void) {
    return SPI_DATA; // Retourne 0xFFFF après une écriture
}
```

### Notes

- Implémentation stub
- Toute écriture dans DATA déclenche:
  - `spi_rx = 0xFFFF`
  - `spi_status = 0x0003` (TX_READY | RX_READY)
- Utile pour éviter les boucles de polling infinies

---

## Timer

**Adresse de base:** 0xFFFE3800  
**Taille:** 256 octets  
**Type:** Mixte 8/16-bit

### Registres

| Offset | Nom | R/W | Taille | Description |
|--------|-----|-----|--------|-------------|
| 0x02 | CNT | R | 8/16-bit | Compteur du timer |

### Utilisation

```c
#define TIMER_BASE 0xFFFE3800
#define TIMER_CNT8  (*(volatile uint8_t *)(TIMER_BASE + 0x02))
#define TIMER_CNT16 (*(volatile uint16_t *)(TIMER_BASE + 0x02))

// Lecture 8-bit
uint8_t timer_val8 = TIMER_CNT8;

// Lecture 16-bit
uint16_t timer_val16 = TIMER_CNT16;
```

### Notes

- Le compteur s'incrémente à chaque lecture
- Implémentation basique pour éviter les boucles de polling
- Pas de configuration de période ou d'interruption

---

## Périphériques MMIO

### Blocs de Registres 8-bit

Les blocs suivants utilisent des registres 8-bit avec lecture/écriture simple:

- **0xFFFE1800** (18XX) - 256 octets
- **0xFFFE2800** (28XX) - 256 octets
- **0xFFFF5000** (50XX) - 256 octets
- **0xFFFE8000** (80XX) - 256 octets (avec logging)
- **0xFFFFFF00** (FFXX) - 256 octets (avec logging)

#### Registres spéciaux 8-bit

| Bloc | Offset | Valeur | Description |
|------|--------|--------|-------------|
| Tous | 0x04 | 0xFF | Anti-polling |
| Tous | 0x09 | 0xFF | Anti-polling |
| Tous | 0x03 | 0x01 | Status "ready" |

### Blocs de Registres 16-bit

Les blocs suivants utilisent des registres 16-bit alignés:

- **0xFFFFFB00** (SYSFB) - 256 octets
- **0xFFFFFD00** (SYSFD) - 256 octets
- **0xFFFF9800** (98XX) - 256 octets
- **0xFFFFF900** (F9XX) - 256 octets
- **0xFFFFFA00** (FAXX) - 256 octets
- **0xFFFFFC00** (FCXX) - 256 octets
- **0xFFFEF000** (F0XX) - 256 octets

#### Registre spécial 16-bit

| Bloc | Offset | Valeur | Description |
|------|--------|--------|-------------|
| 48XX | 0x0A | 0xFFFF | Toujours "ready" |

### Bloc Mixte 8/16-bit

**0xFFFE6800** (68XX) - Supporte les accès 8-bit et 16-bit

#### Registres spéciaux mixtes

| Offset | Taille | Valeur | Description |
|--------|--------|--------|-------------|
| 0x00 | 8-bit | 0xFF | Anti-polling |

---

## Alias Mémoire

### Alias RAM à 0x00000000

- **Source:** RAM principale (0x00800000)
- **Destination:** 0x00000000
- **Taille:** 128 KiB
- **Priorité:** 1 (haute)
- **Usage:** Accès bas pour les vecteurs et code de démarrage

### Alias RAM High Vectors à 0xFFFF0000

- **Source:** RAM principale (0x00800000)
- **Destination:** 0xFFFF0000
- **Taille:** 64 KiB
- **Priorité:** -1 (basse)
- **Usage:** Vecteurs d'interruption en mode high vectors

### Alias UART à 0x00000000

- **Source:** UART (0xFFFF5800)
- **Destination:** 0x00000000
- **Taille:** 256 octets
- **Priorité:** -2 (très basse)
- **Usage:** Accès UART pour firmware legacy

### Alias FAXX à 0x00000000

- **Source:** FAXX (0xFFFFFA00)
- **Destination:** 0x00000000
- **Taille:** 256 octets
- **Priorité:** -3
- **Usage:** Compatibilité

### Alias FCXX à 0x00000000

- **Source:** FCXX (0xFFFFFC00)
- **Destination:** 0x00000000
- **Taille:** 256 octets
- **Priorité:** -4
- **Usage:** Capture accès à 0x00000002

### Alias F9XX à 0x00000100

- **Source:** F9XX (0xFFFFF900)
- **Destination:** 0x00000100
- **Taille:** 256 octets
- **Priorité:** -5
- **Usage:** Capture accès à 0x100/0x102/0x104

### Ordre de Priorité des Alias

En cas d'accès à une adresse couverte par plusieurs régions:

1. Priorité 1: RAM_ALIAS0
2. Priorité 0: (défaut)
3. Priorité -1: VECTORS_HI
4. Priorité -2: UART_ALIAS0
5. Priorité -3: FAXX_ALIAS0
6. Priorité -4: FCXX_ALIAS0
7. Priorité -5: F9XX_ALIAS0100

---

## NOR Flash CFI01

**Adresse de base:** 0x02000000  
**Taille:** 4 MiB  
**Type:** CFI Flash

### Caractéristiques

- **Taille secteur:** 64 KiB
- **Largeur de bus:** 16-bit (1 = x16)
- **ID Fabricant:** 0x0089 (Intel)
- **ID Device:** 0x8871

### Utilisation

```c
#define FLASH_BASE 0x02000000

// Lecture directe
uint16_t *flash = (uint16_t *)FLASH_BASE;
uint16_t data = flash[offset];

// Commandes CFI (non implémentées dans le stub actuel)
```

### Notes

- Peut être attachée avec `-pflash image.bin`
- Le firmware peut patcher l'adresse flash à 0x008147e8
- Émulation CFI01 complète fournie par QEMU

---

## Patch Automatique du Firmware

Le code QEMU patch automatiquement la variable `the_flash` lors du chargement:

**Adresse:** 0x008147e8  
**Valeur écrite:** 0x02000000 (CALYPSO_FLASH_BASE)  
**Format:** Little-endian 32-bit

```c
// Dans le firmware à 0x008147e8:
uint32_t *the_flash; // Sera automatiquement = 0x02000000
```

---

## Exemples de Code

### Initialisation Basique

```c
#include <stdint.h>

#define UART_BASE 0xFFFF5800
#define UART_DATA (*(volatile uint8_t *)(UART_BASE + 0x00))

void uart_init(void) {
    // Pas de configuration nécessaire - stub ready
}

void uart_putc(char c) {
    UART_DATA = c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

int main(void) {
    uart_init();
    uart_puts("Hello Calypso!\r\n");
    
    while (1);
    return 0;
}
```

### Lecture Timer

```c
#define TIMER_BASE 0xFFFE3800
#define TIMER_CNT (*(volatile uint16_t *)(TIMER_BASE + 0x02))

void delay(void) {
    uint16_t start = TIMER_CNT;
    uint16_t now;
    
    do {
        now = TIMER_CNT;
    } while ((now - start) < 1000);
}
```

### Test SPI

```c
#define SPI_BASE 0xFFFE3000
#define SPI_STATUS (*(volatile uint16_t *)(SPI_BASE + 0x06))
#define SPI_DATA (*(volatile uint16_t *)(SPI_BASE + 0x0A))

uint16_t spi_transfer(uint16_t data) {
    SPI_DATA = data;
    while (!(SPI_STATUS & 0x03)); // Wait ready (ne bloquera pas)
    return SPI_DATA; // Retournera 0xFFFF
}
```

---

## Notes de Développement

### Logging

Les accès suivants sont loggés automatiquement:

- **UART TX:** Tous les caractères écrits
- **80XX:** Toutes les écritures
- **FFXX:** Toutes les écritures
- **Timer:** Toutes les écritures

Pour activer le logging QEMU:
```bash
qemu-system-arm -M calypso-min -d guest_errors -D qemu.log ...
```

### Anti-Polling

Certains registres retournent des valeurs "safe" pour éviter les boucles infinies:

- Offsets 0x04, 0x09: 0xFF
- Offset 0x03: 0x01
- SPI STATUS: 0x0003
- UART STATUS: 0x60
- Timer: incrémentation automatique

### Extensions Futures

Fonctionnalités à implémenter:

1. Interruptions réelles (NVIC/VIC)
2. Timer avec périodes configurables
3. UART complète avec RX
4. DMA
5. LCD controller
6. Audio
7. Keypad
8. Battery management
