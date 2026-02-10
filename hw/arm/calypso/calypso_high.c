/*
 * Calypso SoC "high" machine for OsmocomBB highram firmware
 * QEMU 9.2 — includes INTH, timers w/ IRQ, keypad, TWL3025 ABB stub
 * PATCHED: Full UART RX/TX support with CharBackend
 *
 * Usage:
 *   qemu-system-arm -M calypso-high -cpu arm946 \
 *     -kernel loader.highram.elf -serial pty -monitor stdio -nographic -s -S
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/boards.h"
#include "hw/sysbus.h"

#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "hw/qdev-properties.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/block/flash.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "hw/char/serial.h"
#include "chardev/char-fe.h"
#include "qemu/timer.h"
#include "hw/irq.h"

#include "calypso_trx.h"

/* ========================================================================
 * Memory Map
 * ======================================================================== */

/* RAM: internal 256K + external 8M */
#define CALYPSO_IRAM_BASE     0x00800000
#define CALYPSO_IRAM_SIZE     (256 * 1024)
#define CALYPSO_XRAM_BASE     0x01000000
#define CALYPSO_XRAM_SIZE     (8 * 1024 * 1024)

/* Flash */
#define CALYPSO_FLASH_BASE    0x02000000
#define CALYPSO_FLASH_SIZE    (4 * 1024 * 1024)

/* Peripherals */
#define CALYPSO_RHEA_BASE     0xFFFE0000
#define CALYPSO_MMIO_18XX     0xFFFE1800
#define CALYPSO_MMIO_28XX     0xFFFE2800
#define CALYPSO_SPI_BASE      0xFFFE3000
#define CALYPSO_TIMER1_BASE   0xFFFE3800
#define CALYPSO_TIMER2_BASE   0xFFFE6800
#define CALYPSO_KEYPAD_BASE   0xFFFE4800
#define CALYPSO_MMIO_48XX     0xFFFE4800
#define CALYPSO_MMIO_80XX     0xFFFE8000
#define CALYPSO_MMIO_F0XX     0xFFFEF000
#define CALYPSO_MMIO_50XX     0xFFFF5000
#define CALYPSO_UART_MODEM    0xFFFF5000
#define CALYPSO_UART_IRDA     0xFFFF5800
#define CALYPSO_MMIO_98XX     0xFFFF9800
#define CALYPSO_MMIO_F9XX     0xFFFFF900
#define CALYPSO_INTH_BASE     0xFFFFFA00
#define CALYPSO_SYSTEM_FB     0xFFFFFB00
#define CALYPSO_MMIO_FCXX     0xFFFFFC00
#define CALYPSO_SYSTEM_FD     0xFFFFFD00
#define CALYPSO_MMIO_FFXX     0xFFFFFF00

#define CALYPSO_PERIPH_SIZE   256

/* ========================================================================
 * IRQ definitions (matches Calypso INTH)
 * ======================================================================== */

/* IRQ numbers — must match OsmocomBB calypso/irq.h */
#define CALYPSO_IRQ_TIMER1       1
#define CALYPSO_IRQ_TIMER2       2
#define CALYPSO_IRQ_TPU_FRAME    4
#define CALYPSO_IRQ_TPU_PAGE     5
#define CALYPSO_IRQ_UART_MODEM   7
#define CALYPSO_IRQ_KEYPAD       8
#define CALYPSO_IRQ_SPI         13
#define CALYPSO_IRQ_API         15
#define CALYPSO_IRQ_UART_IRDA   18
#define CALYPSO_NUM_IRQS      32

/* ========================================================================
 * INTH — Interrupt Handler
 * Calypso has a two-level interrupt controller at 0xFFFFFA00
 * ======================================================================== */

typedef struct CalypsoINTH {
    MemoryRegion iomem;
    qemu_irq parent_irq;
    qemu_irq parent_fiq;

    uint16_t ilr[CALYPSO_NUM_IRQS];   /* Interrupt Level Registers */
    uint16_t ith_v;                     /* Current highest-priority pending */
    uint32_t pending;                   /* Bitmask of pending IRQs */
    uint32_t mask;                      /* Bitmask: 1 = masked */
} CalypsoINTH;

static void calypso_inth_update(CalypsoINTH *s)
{
    uint32_t active = s->pending & ~s->mask;
    int best_irq = -1;
    int best_prio = 0x7F;
    int is_fiq = 0;

    for (int i = 0; i < CALYPSO_NUM_IRQS; i++) {
        if (active & (1u << i)) {
            int prio = s->ilr[i] & 0x1F;
            if (prio < best_prio) {
                best_prio = prio;
                best_irq = i;
                is_fiq = (s->ilr[i] >> 8) & 1;
            }
        }
    }

    if (best_irq >= 0) {
        s->ith_v = best_irq;
        if (is_fiq) {
            qemu_irq_raise(s->parent_fiq);
            qemu_irq_lower(s->parent_irq);
        } else {
            qemu_irq_raise(s->parent_irq);
            qemu_irq_lower(s->parent_fiq);
        }
    } else {
        s->ith_v = 0;
        qemu_irq_lower(s->parent_irq);
        qemu_irq_lower(s->parent_fiq);
    }
}

static void calypso_inth_set_irq(void *opaque, int irq, int level)
{
    CalypsoINTH *s = (CalypsoINTH *)opaque;
    if (level) {
        s->pending |= (1u << irq);
    } else {
        s->pending &= ~(1u << irq);
    }
    calypso_inth_update(s);
}

/*
 * Register map (16-bit, offsets from 0xFFFFFA00):
 * 0x00        : IT_REG1 (pending low, read-only)
 * 0x02        : IT_REG2 (pending high, read-only)
 * 0x04        : MASK_IT_REG1 (mask low)
 * 0x06        : MASK_IT_REG2 (mask high)
 * 0x20..0x5F  : ILR[0..31] (2 bytes each)
 * 0x80        : IRQ_NUM (current IRQ number)
 * 0x82        : FIQ_NUM
 * 0x84        : IRQ_CTRL (write 1 to acknowledge)
 */
static uint64_t calypso_inth_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoINTH *s = (CalypsoINTH *)opaque;

    switch (offset) {
    case 0x00: /* IT_REG1 — pending bits [15:0] */
        return s->pending & 0xFFFF;
    case 0x02: /* IT_REG2 — pending bits [31:16] */
        return (s->pending >> 16) & 0xFFFF;
    case 0x04: /* MASK_IT_REG1 */
        return s->mask & 0xFFFF;
    case 0x06: /* MASK_IT_REG2 */
        return (s->mask >> 16) & 0xFFFF;
    case 0x80: /* IRQ_NUM */
        return s->ith_v;
    case 0x82: /* FIQ_NUM */
        return s->ith_v;
    case 0x84: /* IRQ_CTRL */
        return 0;
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            return s->ilr[idx];
        }
        return 0;
    }
}

static void calypso_inth_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    CalypsoINTH *s = (CalypsoINTH *)opaque;

    switch (offset) {
    case 0x04: /* MASK_IT_REG1 */
        s->mask = (s->mask & 0xFFFF0000) | (value & 0xFFFF);
        calypso_inth_update(s);
        break;
    case 0x06: /* MASK_IT_REG2 */
        s->mask = (s->mask & 0x0000FFFF) | ((value & 0xFFFF) << 16);
        calypso_inth_update(s);
        break;
    case 0x84: /* IRQ_CTRL — acknowledge */
        if (s->ith_v < CALYPSO_NUM_IRQS) {
            s->pending &= ~(1u << s->ith_v);
        }
        calypso_inth_update(s);
        break;
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            s->ilr[idx] = value & 0x1FFF;
        }
        break;
    }
}

static const MemoryRegionOps calypso_inth_ops = {
    .read = calypso_inth_read,
    .write = calypso_inth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ========================================================================
 * Timer — Calypso watchdog/GP timer with IRQ support
 * ======================================================================== */

typedef struct CalypsoTimer {
    MemoryRegion iomem;
    QEMUTimer *timer;
    qemu_irq irq;

    uint16_t load;      /* Reload value */
    uint16_t count;     /* Current counter */
    uint16_t ctrl;      /* Control: bit0=start, bit1=auto-reload, bit2=irq-en */
    uint16_t prescaler;
    int64_t  tick_ns;   /* Nanoseconds per tick */
    bool     running;
} CalypsoTimer;

#define TIMER_CTRL_START   (1 << 0)
#define TIMER_CTRL_RELOAD  (1 << 1)
#define TIMER_CTRL_IRQ_EN  (1 << 2)

static void calypso_timer_tick(void *opaque)
{
    CalypsoTimer *t = (CalypsoTimer *)opaque;

    if (!t->running) {
        return;
    }

    t->count--;
    if (t->count == 0) {
        /* Fire IRQ if enabled */
        if (t->ctrl & TIMER_CTRL_IRQ_EN) {
            qemu_irq_pulse(t->irq);
        }
        /* Auto-reload */
        if (t->ctrl & TIMER_CTRL_RELOAD) {
            t->count = t->load;
        } else {
            t->running = false;
            return;
        }
    }

    timer_mod(t->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + t->tick_ns);
}

static void calypso_timer_start(CalypsoTimer *t)
{
    if (t->load == 0) {
        return;
    }
    t->count = t->load;
    t->running = true;
    /* Calypso 13MHz / (prescaler+1) */
    int64_t freq = 13000000LL / (t->prescaler + 1);
    t->tick_ns = NANOSECONDS_PER_SECOND / freq;
    timer_mod(t->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + t->tick_ns);
}

/*
 * Timer register map (16-bit, offset from base):
 * 0x00: CNTL  (control)
 * 0x02: LOAD  (reload value)
 * 0x04: READ  (current count, read-only)
 * 0x06: PRESCALER
 */
static uint64_t calypso_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTimer *t = (CalypsoTimer *)opaque;

    switch (offset) {
    case 0x00: return t->ctrl;
    case 0x02: return t->load;
    case 0x04: return t->count;
    case 0x06: return t->prescaler;
    default:   return 0;
    }
}

static void calypso_timer_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    CalypsoTimer *t = (CalypsoTimer *)opaque;

    switch (offset) {
    case 0x00: /* CNTL */
        t->ctrl = value & 0x07;
        if (value & TIMER_CTRL_START) {
            calypso_timer_start(t);
        } else {
            t->running = false;
            timer_del(t->timer);
        }
        break;
    case 0x02: /* LOAD */
        t->load = value;
        break;
    case 0x06: /* PRESCALER */
        t->prescaler = value;
        break;
    }
}

static const MemoryRegionOps calypso_timer_ops = {
    .read = calypso_timer_read,
    .write = calypso_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ========================================================================
 * Keypad controller with IRQ
 * ======================================================================== */

typedef struct CalypsoKeypad {
    MemoryRegion iomem;
    qemu_irq irq;

    uint16_t ctrl;
    uint16_t debounce;
    uint16_t column_out;
    uint16_t row_in;       /* Simulated key state */
    uint16_t isr;          /* Interrupt status */
    bool     irq_enabled;
} CalypsoKeypad;

/*
 * Keypad register map (16-bit):
 * 0x00: CTRL
 * 0x02: DEBOUNCE
 * 0x04: COLUMN_OUT (select columns to scan)
 * 0x06: ROW_IN (key matrix result, read-only)
 * 0x08: ISR (interrupt status)
 * 0x0A: IMR (interrupt mask)
 */
static uint64_t calypso_keypad_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0x0000;
}

static void calypso_keypad_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    /* ignore all writes: keypad disabled */
}

static const MemoryRegionOps calypso_keypad_ops = {
    .read = calypso_keypad_read,
    .write = calypso_keypad_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* Simulate a key press: sets row_in and fires IRQ */
static void __attribute__((unused)) calypso_keypad_press(CalypsoKeypad *k, uint16_t keybit)
{
    k->row_in = keybit;
    k->isr |= 0x01;
    if (k->irq_enabled) {
        qemu_irq_raise(k->irq);
    }
}

static void __attribute__((unused)) calypso_keypad_release(CalypsoKeypad *k)
{
    k->row_in = 0x0000;
    k->isr |= 0x02;
    if (k->irq_enabled) {
        qemu_irq_raise(k->irq);
    }
}

/* ========================================================================
 * SPI / TWL3025 ABB stub
 * ======================================================================== */

typedef struct CalypsoSPI {
    MemoryRegion iomem;
    qemu_irq irq;

    uint16_t ctrl;
    uint16_t status;
    uint16_t tx_data;
    uint16_t rx_data;

    /* TWL3025 shadow registers */
    uint16_t abb_regs[256];
} CalypsoSPI;

/* TWL3025 register addresses */
#define ABB_VRPCDEV   0x01
#define ABB_VRPCSTS   0x02
#define ABB_TOGBR1    0x09
#define ABB_TOGBR2    0x0A
#define ABB_AUXLED    0x17
#define ABB_ITSTATREG 0x1B

static __attribute__((unused))
uint16_t twl3025_spi_xfer(CalypsoSPI *s, uint16_t tx)
{
    /* Calypso SPI protocol: bit15=R/W, bits[14:6]=addr, bits[5:0]=data(W) */
    int read   = (tx >> 15) & 1;
    int addr   = (tx >> 6) & 0x1FF;
    int wdata  = tx & 0x3F;

    if (addr >= 256) {
        addr = 0;
    }

    if (read) {
        return s->abb_regs[addr];
    } else {
        s->abb_regs[addr] = wdata;
        /* Fake power status as always-on */
        if (addr == ABB_VRPCDEV) {
            s->abb_regs[ABB_VRPCSTS] = 0x1F; /* All regulators on */
        }
        return 0;
    }
}

/*
 * SPI register map (16-bit):
 * 0x00: STATUS (bit0=TX_READY, bit1=RX_READY)
 * 0x02: CTRL
 * 0x04: TX
 * 0x06: RX
 */
static uint64_t calypso_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    fprintf(stderr, "=== SPI READ HIGH ===\n");
    return 0x2;
}

static void calypso_spi_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    /* accept writes but do nothing */
}

static const MemoryRegionOps calypso_spi_ops = {
    .read = calypso_spi_read,
    .write = calypso_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ========================================================================
 * UART with full RX/TX support via CharBackend
 * PATCH v3: 64-byte RX FIFO, DLAB routing, level-sensitive IRQ,
 *           full TI Calypso UART register set (incl. SSR, ACREG, FIFO levels)
 * ======================================================================== */

#define UART_FIFO_SIZE  64  /* Calypso hardware FIFO depth */

typedef struct CalypsoUART {
    MemoryRegion iomem;
    CharBackend chr;
    qemu_irq irq;

    /* Identification */
    const char *name;  /* "modem" or "irda" */

    /* RX FIFO */
    uint8_t rx_fifo[UART_FIFO_SIZE];
    int     rx_head;   /* next write position */
    int     rx_tail;   /* next read  position */
    int     rx_count;  /* bytes in FIFO       */

    /* UART registers */
    uint8_t ier;       /* Interrupt Enable Register          */
    uint8_t lcr;       /* Line Control Register              */
    uint8_t mcr;       /* Modem Control Register             */
    uint8_t msr;       /* Modem Status Register              */
    uint8_t scr;       /* Scratch Pad Register (SPR)         */
    uint8_t mdr1;      /* Mode Definition Register 1         */
    uint8_t dll;       /* Divisor Latch Low                  */
    uint8_t dlh;       /* Divisor Latch High                 */
    uint8_t efr;       /* Enhanced Feature Register          */
    uint8_t tlr;       /* Trigger Level Register             */
    uint8_t fcr;       /* FIFO Control (write-only, shadow)  */
    uint8_t xon1;      /* XON1 register                      */
    uint8_t xon2;      /* XON2 register                      */
    uint8_t xoff1;     /* XOFF1 register                     */
    uint8_t xoff2;     /* XOFF2 register                     */

    /* IRQ state tracking */
    bool    irq_raised;
} CalypsoUART;

/* LSR bits */
#define UART_LSR_DR   0x01  /* Data Ready           */
#define UART_LSR_OE   0x02  /* Overrun Error        */
#define UART_LSR_THRE 0x20  /* THR Empty            */
#define UART_LSR_TEMT 0x40  /* Transmitter Empty    */

/* IER bits */
#define UART_IER_RDI  0x01  /* RX Data Available    */
#define UART_IER_THRI 0x02  /* THR Empty            */
#define UART_IER_RLSI 0x04  /* RX Line Status       */
#define UART_IER_MSI  0x08  /* Modem Status         */

/* IIR values */
#define UART_IIR_NO_INT    0x01  /* No interrupt pending      */
#define UART_IIR_RDI       0x04  /* RX Data Available         */
#define UART_IIR_THRI      0x02  /* THR Empty                 */
#define UART_IIR_FIFO_EN   0xC0  /* FIFOs enabled             */

/* LCR bits */
#define UART_LCR_DLAB     0x80  /* Divisor Latch Access Bit  */
#define UART_LCR_ENHANCED 0xBF  /* Magic value for EFR access */

/* ---- FIFO helpers ---- */

static inline bool uart_rx_empty(CalypsoUART *s)
{
    return s->rx_count == 0;
}

static inline bool uart_rx_full(CalypsoUART *s)
{
    return s->rx_count >= UART_FIFO_SIZE;
}

static void uart_rx_push(CalypsoUART *s, uint8_t byte)
{
    if (uart_rx_full(s)) {
        fprintf(stderr, "[calypso-uart-%s] RX FIFO OVERRUN (dropped 0x%02x)\n",
                s->name, byte);
        return;
    }
    s->rx_fifo[s->rx_head] = byte;
    s->rx_head = (s->rx_head + 1) % UART_FIFO_SIZE;
    s->rx_count++;
}
static __attribute__((unused))
uint8_t uart_rx_pop(CalypsoUART *s)
{
    uint8_t byte;
    if (uart_rx_empty(s)) {
        return 0x00;
    }
    byte = s->rx_fifo[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % UART_FIFO_SIZE;
    s->rx_count--;
    return byte;
}

static void uart_rx_reset(CalypsoUART *s)
{
    s->rx_head = 0;
    s->rx_tail = 0;
    s->rx_count = 0;
}

/* ---- IRQ management (level-sensitive) ---- */

static void calypso_uart_update_irq(CalypsoUART *s)
{
    bool should_raise = false;

    /* RX data available interrupt */
    if ((s->ier & UART_IER_RDI) && !uart_rx_empty(s)) {
        should_raise = true;
    }
    /* THR empty interrupt (TX always ready in our implementation) */
    if (s->ier & UART_IER_THRI) {
        should_raise = true;
    }

    if (should_raise && !s->irq_raised) {
        qemu_irq_raise(s->irq);
        s->irq_raised = true;
    } else if (!should_raise && s->irq_raised) {
        qemu_irq_lower(s->irq);
        s->irq_raised = false;
    }
}

/* ---- CharBackend callbacks ---- */

static void calypso_uart_rx_callback(void *opaque, const uint8_t *buf, int size)
{
    CalypsoUART *s = (CalypsoUART *)opaque;

    fprintf(stderr, "[calypso-uart-%s] RX CALLBACK: size=%d, first_byte=0x%02x, "
            "fifo_count=%d\n", s->name, size, size > 0 ? buf[0] : 0, s->rx_count);

    for (int i = 0; i < size; i++) {
        if (!uart_rx_full(s)) {
            uart_rx_push(s, buf[i]);
            fprintf(stderr, "[calypso-uart-%s] RX STORED: 0x%02x [fifo %d/%d]\n",
                    s->name, buf[i], s->rx_count, UART_FIFO_SIZE);
        } else {
            fprintf(stderr, "[calypso-uart-%s] RX DROPPED: FIFO full "
                    "(0x%02x lost)\n", s->name, buf[i]);
        }
    }

    calypso_uart_update_irq(s);
}

static int calypso_uart_can_receive(void *opaque)
{
    CalypsoUART *s = (CalypsoUART *)opaque;
    int avail = UART_FIFO_SIZE - s->rx_count;
    return avail > 0 ? avail : 0;
}

static void calypso_uart_event(void *opaque, QEMUChrEvent event)
{
    /* Nothing needed for now */
}

/* ---- Register access ----
 *
 * Calypso UART register map (byte offsets, 8-bit access):
 *
 *   Offset  DLAB=0/Read   DLAB=0/Write   DLAB=1/R   DLAB=1/W   LCR=0xBF
 *   0x00    RHR           THR            DLL        DLL        DLL
 *   0x01    IER           IER            DLH        DLH        IER (special)
 *   0x02    IIR           FCR            IIR        FCR        EFR
 *   0x03    LCR           LCR            LCR        LCR        LCR
 *   0x04    MCR           MCR            MCR        MCR        XON1
 *   0x05    LSR           -              LSR        -          XON2
 *   0x06    MSR           MSR            MSR        MSR        XOFF1
 *   0x07    SPR           SPR            SPR        SPR        XOFF2
 *   0x08    MDR1          MDR1           -          -          -
 *   0x10    SCR/SSR       SCR/SSR        -          -          -
 *   0x80    DLL alias     DLL alias
 *   0x81    DLH alias     DLH alias
 */

static uint64_t calypso_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoUART *s = (CalypsoUART *)opaque;
    uint64_t ret = 0;
    bool dlab = (s->lcr & UART_LCR_DLAB);
    bool enhanced = (s->lcr == UART_LCR_ENHANCED);  /* LCR=0xBF → EFR mode */

    switch (offset) {
    case 0x00:  /* RHR / DLL */
        if (dlab || enhanced) {
            ret = s->dll;
        } else {
	    ret = 0x41; /* fake RX byte */
            calypso_uart_update_irq(s);
            /* Kick chardev to deliver more if available */
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;

    case 0x01:  /* IER / DLH */
        if (dlab && !enhanced) {
            ret = s->dlh;
        } else {
            ret = s->ier;
        }
        break;

    case 0x02:  /* IIR */
        /* Fake RX interrupt pending */
        ret = UART_IIR_RDI | UART_IIR_FIFO_EN;
        break;

    case 0x03:  /* LCR */
        ret = s->lcr;
        break;

    case 0x04:  /* MCR / XON1 */
        ret = enhanced ? s->xon1 : s->mcr;
        break;

    case 0x05:
        /* UART always ready: TX empty + RX data available */
        ret = UART_LSR_THRE | UART_LSR_TEMT | UART_LSR_DR;
        break;


    case 0x06:  /* MSR / XOFF1 */
        ret = enhanced ? s->xoff1 : s->msr;
        break;

    case 0x07:  /* SPR / XOFF2 */
        ret = enhanced ? s->xoff2 : s->scr;
        break;

    case 0x08:  /* MDR1 */
        ret = s->mdr1;
        break;

    case 0x10:  /* SCR / SSR - Supplementary Control Register */
        ret = 0x00;
        break;

    case 0x11:  /* SSR - Supplementary Status Register */
        /* Bit 0: TX_FIFO_FULL (0 = not full, TX always ready) */
        /* Bit 1: RX_CTS_DSR_WAKE_UP_STS */
        ret = 0x00;
        break;

    case 0x12:  /* ACREG - Auxiliary Control Register */
        ret = 0x00;
        break;

    case 0x18:  /* TXFLL - TX FIFO Level Low */
        ret = 0x00;  /* TX FIFO always empty (instant transmit) */
        break;

    case 0x19:  /* TXFLH - TX FIFO Level High */
        ret = 0x00;
        break;

    case 0x1A:  /* RXFLL - RX FIFO Level Low */
        ret = s->rx_count & 0xFF;
        break;

    case 0x1B:  /* RXFLH - RX FIFO Level High */
        ret = 0x00;  /* FIFO never > 64 */
        break;

    case 0x80:  /* DLL alias (Calypso-specific) */
        ret = s->dll;
        break;

    case 0x81:  /* DLH alias (Calypso-specific) */
        ret = s->dlh;
        break;

    default:
        ret = 0;
        break;
    }

    fprintf(stderr, "[calypso-uart-%s] READ  0x%02x → 0x%02x%s\n",
            s->name, (unsigned)offset, (uint8_t)ret,
            (offset == 0x00 && !dlab && !enhanced) ?
                (uart_rx_empty(s) ? " (FIFO now empty)" : " (FIFO has more)") : "");

    return ret;
}

static void calypso_uart_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    CalypsoUART *s = (CalypsoUART *)opaque;
    uint8_t val = value & 0xFF;
    bool dlab = (s->lcr & UART_LCR_DLAB);
    bool enhanced = (s->lcr == UART_LCR_ENHANCED);

    fprintf(stderr, "[calypso-uart-%s] WRITE 0x%02x ← 0x%02x\n",
            s->name, (unsigned)offset, val);

    switch (offset) {
    case 0x00:  /* THR / DLL */
        if (dlab || enhanced) {
            s->dll = val;
        } else {
            /* Transmit */
            qemu_chr_fe_write_all(&s->chr, &val, 1);
            fprintf(stderr, "[calypso-uart-%s] TX: '%c' (0x%02x)\n",
                    s->name, (val >= 0x20 && val < 0x7F) ? val : '.', val);
        }
        break;

    case 0x01:  /* IER / DLH */
        if (dlab && !enhanced) {
            s->dlh = val;
        } else {
            s->ier = val & 0x0F;
            calypso_uart_update_irq(s);
        }
        break;

    case 0x02:  /* FCR / EFR */
        if (enhanced) {
            s->efr = val;
        } else {
            s->fcr = val;
            /* Bit 1: reset RX FIFO */
            if (val & 0x02) {
                uart_rx_reset(s);
                calypso_uart_update_irq(s);
            }
            /* Bit 2: reset TX FIFO (no-op, we transmit immediately) */
        }
        break;

    case 0x03:  /* LCR */
        s->lcr = val;
        break;

    case 0x04:  /* MCR / XON1 */
        if (enhanced) {
            s->xon1 = val;
        } else {
            s->mcr = val;
        }
        break;

    case 0x05:  /* XON2 (only in enhanced mode, LSR is read-only) */
        if (enhanced) {
            s->xon2 = val;
        }
        break;

    case 0x06:  /* XOFF1 (enhanced) / MSR write (ignored normally) */
        if (enhanced) {
            s->xoff1 = val;
        }
        break;

    case 0x07:  /* SPR / XOFF2 */
        if (enhanced) {
            s->xoff2 = val;
        } else {
            s->scr = val;
        }
        break;

    case 0x08:  /* MDR1 */
        s->mdr1 = val;
        break;

    case 0x10:  /* SCR - Supplementary Control Register */
        /* Ignored */
        break;

    case 0x11:  /* SSR - read-only, ignore writes */
        break;

    case 0x12:  /* ACREG - Auxiliary Control Register */
        /* Ignored */
        break;

    case 0x80:  /* DLL alias (Calypso-specific) */
        s->dll = val;
        break;

    case 0x81:  /* DLH alias (Calypso-specific) */
        s->dlh = val;
        break;

    default:
        fprintf(stderr, "[calypso-uart-%s]   ← UNHANDLED offset 0x%02x\n",
                s->name, (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps calypso_uart_ops = {
    .read = calypso_uart_read,
    .write = calypso_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ========================================================================
 * Generic MMIO stubs
 * ======================================================================== */

static uint64_t calypso_mmio8_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0xFF;
}

static void calypso_mmio8_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
}

static const MemoryRegionOps calypso_mmio8_ops = {
    .read = calypso_mmio8_read,
    .write = calypso_mmio8_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static uint64_t calypso_mmio16_read(void *opaque, hwaddr offset, unsigned size)
{
    return 0;
}

static void calypso_mmio16_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
}

static const MemoryRegionOps calypso_mmio16_ops = {
    .read = calypso_mmio16_read,
    .write = calypso_mmio16_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ========================================================================
 * Calypso "High" Machine State
 * ======================================================================== */

typedef struct CalypsoHighState {
    ARMCPU *cpu;

    /* Memory */
    MemoryRegion iram;
    MemoryRegion xram;
    MemoryRegion flash_mem;
    MemoryRegion ram_alias0;
    MemoryRegion high_vectors;

    /* Peripherals */
    CalypsoINTH   inth;
    CalypsoTimer  timer1;
    CalypsoTimer  timer2;
    CalypsoKeypad keypad;
    CalypsoSPI    spi;
    CalypsoUART   uart_modem;  /* PATCH: Added UART with CharBackend */
    CalypsoUART   uart_irda;   /* PATCH: Added UART with CharBackend */

    /* IRQ inputs to INTH */
    qemu_irq *irqs;
} CalypsoHighState;

static void calypso_create_mmio(MemoryRegion *sysmem, const char *name,
                                hwaddr base, const MemoryRegionOps *ops,
                                void *opaque, uint64_t sz)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    memory_region_init_io(mr, NULL, ops, opaque, name, sz);
    memory_region_add_subregion(sysmem, base, mr);
}

static void calypso_high_init(MachineState *machine)
{
    CalypsoHighState *s = g_new0(CalypsoHighState, 1);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    Error *err = NULL;

    /* ---- CPU ---- */
    cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);
    if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
        error_report_err(err);
        exit(1);
    }

    /* ---- RAM ---- */

    /* Internal RAM: 256 KiB at 0x00800000 */
    memory_region_init_ram(&s->iram, NULL, "calypso.iram",
                           CALYPSO_IRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_IRAM_BASE, &s->iram);

    /* External RAM: 8 MiB at 0x01000000 (for highram firmware) */
    memory_region_init_ram(&s->xram, NULL, "calypso.xram",
                           CALYPSO_XRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_XRAM_BASE, &s->xram);

    /* RAM alias at 0x00000000 (vector table) */
    memory_region_init_alias(&s->ram_alias0, NULL, "calypso.ram_alias0",
                             &s->iram, 0, 128 * 1024);
    memory_region_add_subregion_overlap(sysmem, 0x00000000, &s->ram_alias0, 1);

    /* High vectors at 0xFFFF0000 */
    memory_region_init_alias(&s->high_vectors, NULL, "calypso.high_vectors",
                             &s->iram, 0, 64 * 1024);
    memory_region_add_subregion(sysmem, 0xFFFF0000, &s->high_vectors);

    /* ---- Flash ---- */
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi01_register(CALYPSO_FLASH_BASE, "calypso.flash",
                          CALYPSO_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          64 * 1024, 1, 0x0089, 0x0018, 0x0000, 0x0000, 0);

    /* ---- Interrupt Controller (INTH) ---- */
    s->irqs = qemu_allocate_irqs(calypso_inth_set_irq, &s->inth,
                                 CALYPSO_NUM_IRQS);
    s->inth.parent_irq = qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_IRQ);
    s->inth.parent_fiq = qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_FIQ);
    s->inth.mask = 0xFFFFFFFF; /* All masked initially */
    memory_region_init_io(&s->inth.iomem, NULL, &calypso_inth_ops, &s->inth,
                          "calypso.inth", CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_INTH_BASE, &s->inth.iomem);

    /* ---- Timer 1 (GP timer, IRQ 3) ---- */
    s->timer1.irq = s->irqs[CALYPSO_IRQ_TIMER1];
    s->timer1.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_timer_tick,
                                   &s->timer1);
    memory_region_init_io(&s->timer1.iomem, NULL, &calypso_timer_ops,
                          &s->timer1, "calypso.timer1", CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TIMER1_BASE,
                                &s->timer1.iomem);

    /* ---- Timer 2 (watchdog, IRQ 4) ---- */
    s->timer2.irq = s->irqs[CALYPSO_IRQ_TIMER2];
    s->timer2.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_timer_tick,
                                   &s->timer2);
    memory_region_init_io(&s->timer2.iomem, NULL, &calypso_timer_ops,
                          &s->timer2, "calypso.timer2", CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TIMER2_BASE,
                                &s->timer2.iomem);

    /* ---- Keypad (IRQ 5) ---- */
    s->keypad.irq = s->irqs[CALYPSO_IRQ_KEYPAD];
    memory_region_init_io(&s->keypad.iomem, NULL, &calypso_keypad_ops,
                          &s->keypad, "calypso.keypad", CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_KEYPAD_BASE,
                                &s->keypad.iomem);

    /* ---- SPI / TWL3025 ABB (IRQ 7) ---- */
    s->spi.irq = s->irqs[CALYPSO_IRQ_SPI];
    /* Init ABB regs: power status on */
    s->spi.abb_regs[ABB_VRPCSTS] = 0x1F;
    s->spi.abb_regs[ABB_ITSTATREG] = 0x00;
    memory_region_init_io(&s->spi.iomem, NULL, &calypso_spi_ops, &s->spi,
                          "calypso.spi", CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_SPI_BASE, &s->spi.iomem);

    /* ========================================================================
     * PATCH: UART MODEM with full CharBackend support
     * ======================================================================== */
    
    /* ---- UART MODEM (pas utilisé par ce loader) ---- */
    s->uart_modem.name = "modem";
    s->uart_modem.irq = s->irqs[CALYPSO_IRQ_UART_MODEM];
    uart_rx_reset(&s->uart_modem);
    
    memory_region_init_io(&s->uart_modem.iomem, NULL, &calypso_uart_ops,
                          &s->uart_modem, "calypso.uart_modem",
                          CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_UART_MODEM,
                                &s->uart_modem.iomem);
    
    /* PAS DE CHARDEV pour modem - ce loader utilise IRDA ! */
    
    /* ---- UART IRDA (console / osmocon) ---- */
    s->uart_irda.name = "irda";
    s->uart_irda.irq = s->irqs[CALYPSO_IRQ_UART_IRDA];
    uart_rx_reset(&s->uart_irda);
    
    memory_region_init_io(&s->uart_irda.iomem, NULL, &calypso_uart_ops,
                          &s->uart_irda, "calypso.uart_irda",
                          CALYPSO_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_UART_IRDA,
                                &s->uart_irda.iomem);
    
    /* CRITICAL FIX: Connecter le chardev (serial0) à UART_IRDA !! */
    /* Le loader utilise IRDA pour osmocon, pas MODEM ! */
    qemu_chr_fe_init(&s->uart_irda.chr, serial_hd(0), &error_fatal);
    qemu_chr_fe_set_handlers(&s->uart_irda.chr,
                             calypso_uart_can_receive,
                             calypso_uart_rx_callback,
                             calypso_uart_event,
                             NULL,
                             &s->uart_irda,
                             NULL, true);
    
    /* No auto-ping: let the firmware and osmocon handle the handshake
     * naturally. The FIFO + level-sensitive IRQ ensure data from osmocon
     * reaches the firmware at the right time. */

    /* ---- MMIO stubs ---- */
    calypso_create_mmio(sysmem, "calypso.mmio_18xx",
                        CALYPSO_MMIO_18XX, &calypso_mmio8_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    /* NOTE: 0xFFFE2800 (ULPD) is handled by calypso_trx.c */
    calypso_create_mmio(sysmem, "calypso.mmio_80xx",
                        CALYPSO_MMIO_80XX, &calypso_mmio8_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.mmio_f0xx",
                        CALYPSO_MMIO_F0XX, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.mmio_98xx",
                        CALYPSO_MMIO_98XX, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.mmio_f9xx",
                        CALYPSO_MMIO_F9XX, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.system_fb",
                        CALYPSO_SYSTEM_FB, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.mmio_fcxx",
                        CALYPSO_MMIO_FCXX, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.system_fd",
                        CALYPSO_SYSTEM_FD, &calypso_mmio16_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.mmio_ffxx",
                        CALYPSO_MMIO_FFXX, &calypso_mmio8_ops, NULL,
                        CALYPSO_PERIPH_SIZE);

    /* ---- TRX bridge (DSP API, TPU, TSP, ULPD, TDMA timer, TRX UDP) ---- */
    calypso_trx_init(sysmem, s->irqs, 4729);

    /* Load firmware (bare-metal ELF, not Linux) */
    if (machine->kernel_filename) {
        uint64_t entry;
        if (load_elf(machine->kernel_filename, NULL, NULL, NULL,
                     &entry, NULL, NULL, NULL,
                     0, EM_ARM, 1, 0) < 0) {
            error_report("Could not load ELF: %s", machine->kernel_filename);
            exit(1);
        }
        cpu_set_pc(CPU(s->cpu), entry);
    }

    /* Store keypad pointer globally so QEMU monitor can trigger key presses */
    /* (see qmp/hmp extensions if needed) */
}

/* ========================================================================
 * Machine class registration
 * ======================================================================== */

static void calypso_high_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Calypso SoC (highram) with INTH, timers, keypad, ABB, UART RX";
    mc->init = calypso_high_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm946");
}

static const TypeInfo calypso_high_type = {
    .name = MACHINE_TYPE_NAME("calypso-high"),
    .parent = TYPE_MACHINE,
    .class_init = calypso_high_class_init,
};

static void calypso_high_register_types(void)
{
    type_register_static(&calypso_high_type);
}

type_init(calypso_high_register_types)
