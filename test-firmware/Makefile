# Makefile pour compiler le firmware de test Calypso

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# Options de compilation
CPU = -mcpu=arm946e-s
CFLAGS = $(CPU) -mthumb-interwork -nostdlib -ffreestanding -Wall -Wextra -O2 -g
LDFLAGS = -T linker.ld -nostdlib -Wl,-Map=test.map

# Fichiers
TARGET = test
SOURCES = test-firmware.c
OBJECTS = $(SOURCES:.c=.o)

.PHONY: all clean disasm info

all: $(TARGET).elf $(TARGET).bin

# Compilation
%.o: %.c
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c -o $@ $<

# Link
$(TARGET).elf: $(OBJECTS) linker.ld
	@echo "LD $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJECTS)
	@$(SIZE) $@

# Conversion en binaire brut
$(TARGET).bin: $(TARGET).elf
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@

# Désassemblage
disasm: $(TARGET).elf
	@echo "Désassemblage..."
	@$(OBJDUMP) -D $< > $(TARGET).disasm
	@echo "Écrit dans $(TARGET).disasm"

# Informations
info: $(TARGET).elf
	@echo ""
	@echo "=== Informations sur le firmware ==="
	@echo ""
	@$(SIZE) -A $<
	@echo ""
	@echo "Point d'entrée:"
	@$(OBJDUMP) -f $< | grep "start address"
	@echo ""
	@echo "Sections:"
	@$(OBJDUMP) -h $<

# Nettoyage
clean:
	@echo "Nettoyage..."
	@rm -f $(OBJECTS) $(TARGET).elf $(TARGET).bin $(TARGET).map $(TARGET).disasm

# Test avec QEMU
test: $(TARGET).elf
	@echo "Lancement du test avec QEMU..."
	qemu-system-arm -M calypso-min \
		-kernel $(TARGET).elf \
		-nographic \
		-d guest_errors

# Test avec GDB
debug: $(TARGET).elf
	@echo "Lancement en mode debug (port 1234)..."
	@echo "Connectez GDB avec: $(PREFIX)gdb $(TARGET).elf -ex 'target remote :1234'"
	qemu-system-arm -M calypso-min \
		-kernel $(TARGET).elf \
		-nographic \
		-s -S

help:
	@echo "Cibles disponibles:"
	@echo "  all     - Compiler le firmware (défaut)"
	@echo "  disasm  - Générer le désassemblage"
	@echo "  info    - Afficher les informations sur le firmware"
	@echo "  test    - Tester avec QEMU"
	@echo "  debug   - Lancer en mode debug avec GDB"
	@echo "  clean   - Nettoyer les fichiers générés"
	@echo "  help    - Afficher cette aide"
