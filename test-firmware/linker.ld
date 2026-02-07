/*
 * Linker script pour firmware Calypso
 * 
 * Carte mémoire:
 *   RAM:   0x00800000 - 0x0081FFFF (128 KiB)
 *   Flash: 0x02000000 - 0x023FFFFF (4 MiB)
 */

MEMORY
{
    /* RAM principale */
    RAM (rwx) : ORIGIN = 0x00800000, LENGTH = 128K
    
    /* Flash (optionnel) */
    FLASH (rx) : ORIGIN = 0x02000000, LENGTH = 4M
}

/* Stack pointer initial (fin de la RAM) */
__stack_top = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(reset_handler)

SECTIONS
{
    /* Vecteurs d'interruption - doivent être au début */
    .vectors : ALIGN(4)
    {
        KEEP(*(.vectors))
    } > RAM
    
    /* Code */
    .text : ALIGN(4)
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > RAM
    
    /* Données initialisées */
    .data : ALIGN(4)
    {
        *(.data*)
        . = ALIGN(4);
    } > RAM
    
    /* Données non-initialisées */
    .bss : ALIGN(4)
    {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM
    
    /* Informations de debug (non chargées) */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_str      0 : { *(.debug_str) }
    .debug_ranges   0 : { *(.debug_ranges) }
    
    /* Supprime les sections non utilisées */
    /DISCARD/ :
    {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.comment)
    }
}
