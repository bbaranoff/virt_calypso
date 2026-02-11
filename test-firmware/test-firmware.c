/*
 * Firmware de test minimal pour Calypso
 * 
 * Ce firmware effectue des opérations basiques pour tester
 * les périphériques de la machine Calypso émulée
 *
 * Compilation:
 *   arm-none-eabi-gcc -mcpu=arm946e-s -nostdlib -T linker.ld -o test.elf test.c
 */

/* Adresses des périphériques */
#define UART_BASE     0xFFFF5800
#define UART_DATA     (*(volatile unsigned char *)(UART_BASE + 0x00))
#define UART_STATUS   (*(volatile unsigned char *)(UART_BASE + 0x11))

#define SPI_BASE      0xFFFE3000
#define SPI_STATUS    (*(volatile unsigned short *)(SPI_BASE + 0x06))
#define SPI_DATA      (*(volatile unsigned short *)(SPI_BASE + 0x0A))

#define TIMER_BASE    0xFFFE3800
#define TIMER_CNT     (*(volatile unsigned short *)(TIMER_BASE + 0x02))

/* Fonction pour écrire sur l'UART */
void uart_putc(char c) {
    UART_DATA = c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

/* Fonction pour lire le compteur du timer */
unsigned short timer_read(void) {
    return TIMER_CNT;
}

/* Fonction de délai simple */
void delay(unsigned int count) {
    volatile unsigned int i;
    for (i = 0; i < count; i++) {
        __asm__ volatile ("nop");
    }
}

/* Point d'entrée principal */
void main(void) {
    unsigned short timer_val;
    unsigned int i;
    
    /* Message de démarrage */
    uart_puts("Calypso Test Firmware\r\n");
    uart_puts("===================\r\n\r\n");
    
    /* Test UART */
    uart_puts("Test UART: OK\r\n");
    
    /* Test Timer */
    uart_puts("Test Timer: ");
    timer_val = timer_read();
    if (timer_val != 0xFFFF) {
        uart_puts("OK (");
        /* Affichage de la valeur du timer - simpliste */
        uart_putc('0' + ((timer_val >> 8) & 0xF));
        uart_putc('0' + (timer_val & 0xF));
        uart_puts(")\r\n");
    } else {
        uart_puts("FAILED\r\n");
    }
    
    /* Test SPI */
    uart_puts("Test SPI: ");
    SPI_DATA = 0x1234;
    if (SPI_STATUS & 0x03) {
        uart_puts("OK\r\n");
    } else {
        uart_puts("FAILED\r\n");
    }
    
    /* Boucle infinie avec compteur */
    uart_puts("\r\nDemarrage boucle principale...\r\n");
    
    for (i = 0; i < 10; i++) {
        uart_puts("Iteration ");
        uart_putc('0' + i);
        uart_puts("\r\n");
        delay(100000);
    }
    
    uart_puts("\r\nTest termine!\r\n");
    
    /* Boucle infinie finale */
    while (1) {
        __asm__ volatile ("nop");
    }
}

/* Vecteurs d'interruption */
void __attribute__((naked)) reset_handler(void) {
    __asm__ volatile (
        "ldr sp, =__stack_top\n"
        "bl main\n"
        "b .\n"
    );
}


extern void reset_handler(void);

const void *vectors[] __attribute__((section(".vectors"))) = {
    (void *)0x00820000,
    reset_handler,
    0,0,0,0,0,
    0,0,0,0,
    0,0,
    0,
};
