/*
 * Calypso SoC "high" machine for OsmocomBB highram firmware
 * QEMU 9.2 — includes INTH, timers w/ IRQ, keypad, TWL3025 ABB stub
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
    CalypsoKeypad *k = (CalypsoKeypad *)opaque;

    switch (offset) {
    case 0x00: return k->ctrl;
    case 0x02: return k->debounce;
    case 0x04: return k->column_out;
    case 0x06: return k->row_in;
    case 0x08: return k->isr;
    case 0x0A: return k->irq_enabled ? 0xFFFF : 0x0000;
    default:   return 0xFFFF;
    }
}

static void calypso_keypad_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    CalypsoKeypad *k = (CalypsoKeypad *)opaque;

    switch (offset) {
    case 0x00:
        k->ctrl = value;
        break;
    case 0x02:
        k->debounce = value;
        break;
    case 0x04:
        k->column_out = value;
        break;
    case 0x08: /* ISR: write 1 to clear */
        k->isr &= ~value;
        if (k->isr == 0) {
            qemu_irq_lower(k->irq);
        }
        break;
    case 0x0A: /* IMR */
        k->irq_enabled = (value != 0);
        break;
    }
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

static uint16_t twl3025_spi_xfer(CalypsoSPI *s, uint16_t tx)
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
    CalypsoSPI *s = (CalypsoSPI *)opaque;

    switch (offset) {
    case 0x00: return 0x0003; /* TX_READY | RX_READY always */
    case 0x02: return s->ctrl;
    case 0x04: return s->tx_data;
    case 0x06: return s->rx_data;
    default:   return 0;
    }
}

static void calypso_spi_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    CalypsoSPI *s = (CalypsoSPI *)opaque;

    switch (offset) {
    case 0x02: /* CTRL */
        s->ctrl = value;
        break;
    case 0x04: /* TX — triggers SPI transfer */
        s->tx_data = value;
        s->rx_data = twl3025_spi_xfer(s, value);
        /* Raise SPI IRQ if enabled */
        qemu_irq_pulse(s->irq);
        break;
    }
}

static const MemoryRegionOps calypso_spi_ops = {
    .read = calypso_spi_read,
    .write = calypso_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ========================================================================
 * UART with proper status bits
 * ======================================================================== */

static uint64_t calypso_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case 0x00: return 0x00;  /* RHR — no data */
    case 0x04: /* LSR: TX empty + TX holding empty + no errors */
        return 0x60;
    case 0x05:
        return 0xFF;
    case 0x08: /* MSR */
        return 0xB0; /* CTS + DSR */
    case 0x10: /* SSR */
        return 0x01; /* TX FIFO not full */
    default:
        return 0;
    }
}

static void calypso_uart_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    if (offset == 0x00) {
        char c = (char)(value & 0xFF);
        printf("[calypso-uart] '%c' (0x%02x)\n",
               (c >= 32 && c < 127) ? c : '.', (unsigned)value);
        fflush(stdout);
    }
}

static const MemoryRegionOps calypso_uart_ops = {
    .read = calypso_uart_read,
    .write = calypso_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
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

    /* ---- UARTs ---- */
    calypso_create_mmio(sysmem, "calypso.uart_modem",
                        CALYPSO_UART_MODEM, &calypso_uart_ops, NULL,
                        CALYPSO_PERIPH_SIZE);
    calypso_create_mmio(sysmem, "calypso.uart_irda",
                        CALYPSO_UART_IRDA, &calypso_uart_ops, NULL,
                        CALYPSO_PERIPH_SIZE);

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

    mc->desc = "Calypso SoC (highram) with INTH, timers, keypad, ABB";
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
