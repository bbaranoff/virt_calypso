/*
 * calypso_trx.c — Calypso DSP/TPU/TRX bridge for virtual GSM
 *
 * This module provides the missing peripherals between OsmocomBB L1 firmware
 * and a TRX UDP endpoint (e.g. osmo-bts-trx or a virtual radio bridge).
 *
 * Architecture:
 *
 *   OsmocomBB L1 firmware (in QEMU)
 *       │ writes TX bursts to DSP API RAM
 *       │ programs TPU scenario
 *       │ enables TPU
 *       ▼
 *   calypso_trx.c (this file)
 *       │ intercepts TPU enable → extracts burst from API RAM
 *       │ sends via TRX UDP socket
 *       │ receives RX bursts from TRX UDP
 *       │ injects into API RAM → fires IRQ_API
 *       │ TDMA timer fires IRQ_TPU_FRAME every 4.615 ms
 *       ▼
 *   TRX UDP endpoint (osmo-bts-trx / virtual radio)
 *
 * QEMU 9.2 compatible.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#include "calypso_trx.h"

/* =====================================================================
 * Debug logging
 * ===================================================================== */

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

/* Set to 1 for verbose DSP/TPU register access logging */
#define TRX_DEBUG_DSP    0
#define TRX_DEBUG_TPU    0
#define TRX_DEBUG_TSP    0
#define TRX_DEBUG_ULPD   0
#define TRX_DEBUG_TDMA   0

/* =====================================================================
 * TRX state
 * ===================================================================== */

typedef struct CalypsoTRX {
    /* IRQ lines (borrowed from INTH) */
    qemu_irq *irqs;

    /* ----- DSP API RAM ----- */
    MemoryRegion dsp_iomem;
    uint16_t     dsp_ram[CALYPSO_DSP_SIZE / 2];  /* 64K as 16-bit words */
    uint8_t      dsp_page;                        /* Current DSP page (0/1) */

    /* ----- TPU ----- */
    MemoryRegion tpu_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[1024];                   /* TPU instruction RAM */
    bool         tpu_enabled;

    /* ----- TSP ----- */
    MemoryRegion tsp_iomem;
    uint16_t     tsp_regs[CALYPSO_TSP_SIZE / 2];

    /* ----- ULPD ----- */
    MemoryRegion ulpd_iomem;
    uint16_t     ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t     ulpd_counter;

    /* ----- TDMA frame timing ----- */
    QEMUTimer   *tdma_timer;
    uint32_t     fn;              /* GSM frame number */
    bool         tdma_running;

    /* ----- DSP task completion timer ----- */
    QEMUTimer   *dsp_timer;

    /* ----- TRX UDP socket ----- */
    int          trx_fd;          /* Data socket fd (-1 if disabled) */
    int          trx_port;
    struct sockaddr_in trx_remote;  /* Remote endpoint to send TX bursts */
    bool         trx_connected;

    /* ----- Burst buffer ----- */
    uint8_t      tx_burst[GSM_BURST_BITS];
    uint8_t      rx_burst[GSM_BURST_BITS];
    bool         rx_pending;
    uint8_t      rx_tn;
    int8_t       rx_rssi;
    int16_t      rx_toa;
} CalypsoTRX;

static CalypsoTRX *g_trx;  /* Global for timer callbacks */

/* =====================================================================
 * DSP API RAM — shared memory between ARM and (virtual) DSP
 * ===================================================================== */

static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val;

    if (offset >= CALYPSO_DSP_SIZE) {
        return 0;
    }

    if (size == 2) {
        val = s->dsp_ram[offset / 2];
    } else if (size == 4) {
        val = s->dsp_ram[offset / 2] |
              ((uint32_t)s->dsp_ram[offset / 2 + 1] << 16);
    } else {
        val = ((uint8_t *)s->dsp_ram)[offset];
    }

#if TRX_DEBUG_DSP
    TRX_LOG("DSP read  [0x%04x] = 0x%04x (size=%d)", (unsigned)offset,
            (unsigned)val, size);
#endif
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    if (offset >= CALYPSO_DSP_SIZE) {
        return;
    }

#if TRX_DEBUG_DSP
    TRX_LOG("DSP write [0x%04x] = 0x%04x (size=%d)", (unsigned)offset,
            (unsigned)value, size);
#endif

    if (size == 2) {
        s->dsp_ram[offset / 2] = (uint16_t)value;
    } else if (size == 4) {
        s->dsp_ram[offset / 2] = (uint16_t)value;
        s->dsp_ram[offset / 2 + 1] = (uint16_t)(value >> 16);
    } else {
        ((uint8_t *)s->dsp_ram)[offset] = (uint8_t)value;
    }

    /* Track DSP page changes in NDB area */
    if (offset == DSP_API_NDB + NDB_W_D_DSP_PAGE * 2) {
        s->dsp_page = value & 1;
    }
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read,
    .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* =====================================================================
 * TRX UDP — send TX burst, receive RX burst
 * ===================================================================== */

static void trx_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                            uint8_t *bits, int nbits)
{
    uint8_t pkt[TRX_PKT_LEN_TX];

    if (s->trx_fd < 0 || !s->trx_connected) {
        return;
    }

    pkt[0] = tn;
    pkt[1] = (fn >> 24) & 0xFF;
    pkt[2] = (fn >> 16) & 0xFF;
    pkt[3] = (fn >> 8)  & 0xFF;
    pkt[4] = fn & 0xFF;
    pkt[5] = 0;  /* TX power attenuation */

    int copy = (nbits > GSM_BURST_BITS) ? GSM_BURST_BITS : nbits;
    memcpy(&pkt[TRX_HDR_LEN_TX], bits, copy);
    if (copy < GSM_BURST_BITS) {
        memset(&pkt[TRX_HDR_LEN_TX + copy], 0, GSM_BURST_BITS - copy);
    }

    sendto(s->trx_fd, pkt, TRX_PKT_LEN_TX, 0,
           (struct sockaddr *)&s->trx_remote, sizeof(s->trx_remote));
}

static void trx_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint8_t buf[512];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(s->trx_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
    if (n < TRX_HDR_LEN_RX + 1) {
        return;
    }

    /* Remember remote for TX responses */
    if (!s->trx_connected) {
        s->trx_remote = src;
        s->trx_connected = true;
        TRX_LOG("TRX connected from %s:%d",
                inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }

    /* Parse RX burst (downlink to phone) */
    s->rx_tn   = buf[0];
    /* FN from packet (ignore for now, use our own FN) */
    s->rx_rssi = (int8_t)buf[5];
    s->rx_toa  = (int16_t)((buf[6] << 8) | buf[7]);

    int burst_len = n - TRX_HDR_LEN_RX;
    if (burst_len > GSM_BURST_BITS) burst_len = GSM_BURST_BITS;
    memcpy(s->rx_burst, &buf[TRX_HDR_LEN_RX], burst_len);
    s->rx_pending = true;

#if TRX_DEBUG_TDMA
    TRX_LOG("TRX RX burst TN=%d RSSI=%d len=%d", s->rx_tn, s->rx_rssi,
            burst_len);
#endif
}

static void trx_socket_init(CalypsoTRX *s, int port)
{
    struct sockaddr_in addr;

    s->trx_port = port;
    s->trx_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (s->trx_fd < 0) {
        TRX_LOG("WARNING: Cannot create TRX socket: %s", strerror(errno));
        return;
    }

    int reuse = 1;
    setsockopt(s->trx_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(s->trx_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        TRX_LOG("WARNING: Cannot bind TRX port %d: %s", port, strerror(errno));
        close(s->trx_fd);
        s->trx_fd = -1;
        return;
    }

    /* Default remote: localhost:port+100 (configurable later) */
    memset(&s->trx_remote, 0, sizeof(s->trx_remote));
    s->trx_remote.sin_family = AF_INET;
    s->trx_remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->trx_remote.sin_port = htons(port + 100);

    qemu_set_fd_handler(s->trx_fd, trx_receive_cb, NULL, s);
    TRX_LOG("TRX UDP listening on port %d (send to %d)", port, port + 100);
}

/* =====================================================================
 * DSP task processing — extract/inject bursts
 * ===================================================================== */

static void calypso_dsp_process(CalypsoTRX *s)
{
    /*
     * Called when TPU is enabled (firmware triggered DSP).
     * Extract TX burst from write page, send via TRX.
     * Inject RX burst into read page if available.
     */

    uint16_t *w_page, *r_page, *ndb;
    uint16_t task_d, task_u;

    /* Determine active pages */
    if (s->dsp_page == 0) {
        w_page = &s->dsp_ram[DSP_API_W_PAGE0 / 2];
        r_page = &s->dsp_ram[DSP_API_R_PAGE0 / 2];
    } else {
        w_page = &s->dsp_ram[DSP_API_W_PAGE1 / 2];
        r_page = &s->dsp_ram[DSP_API_R_PAGE1 / 2];
    }
    ndb = &s->dsp_ram[DSP_API_NDB / 2];

    /* Read task words from write page header */
    task_d = w_page[0];  /* d_task_d */
    task_u = w_page[2];  /* d_task_u */

    /* ---- Handle TX (uplink) burst ---- */
    if (task_u != 0) {
        /*
         * TX burst data is in the write page.
         * Offset depends on DSP firmware version.
         * Typical: starts at word offset 0x19 (byte 0x32) in page.
         * Each burst = 78 words = 156 bytes (148 useful bits).
         */
        uint16_t *burst_w = &w_page[0x19];
        uint8_t bits[GSM_BURST_BITS];

        for (int i = 0; i < GSM_BURST_BITS && i < GSM_BURST_WORDS * 2; i++) {
            /* Each word holds one bit (bit 0) in standard API format */
            if (i < 78) {
                bits[i] = burst_w[i] & 1;
            } else {
                bits[i] = burst_w[78 + (i - 78)] & 1;
            }
        }

        trx_send_burst(s, 0 /* TN from TPU scenario */, s->fn, bits,
                        GSM_BURST_BITS);

        TRX_LOG("TX burst FN=%u task_u=0x%04x", s->fn, task_u);
    }

    /* ---- Handle RX (downlink) burst injection ---- */
    if (task_d != 0 && s->rx_pending) {
        /*
         * Inject RX burst into read page.
         * Write soft-bit decisions: 0xFF = strong '1', 0x00 = strong '0'
         * DSP format: one 16-bit word per soft-bit.
         */
        uint16_t *burst_r = &r_page[0x19];

        for (int i = 0; i < GSM_BURST_BITS; i++) {
            /* Convert soft-bits (0-255) to DSP format */
            burst_r[i] = s->rx_burst[i];
        }

        /* Set result flags in read page */
        r_page[0] = 1;    /* d_bursttype: normal burst */
        r_page[1] = 0;    /* d_result: OK */

        s->rx_pending = false;
    } else if (task_d != 0) {
        /* No RX data available — provide empty/noise burst */
        uint16_t *burst_r = &r_page[0x19];
        for (int i = 0; i < GSM_BURST_BITS; i++) {
            burst_r[i] = 128;  /* Erasure (uncertain bit) */
        }
        r_page[0] = 0;  /* No burst */
        r_page[1] = 0;
    }

    /* Clear task words (DSP "consumed" them) */
    w_page[0] = 0;
    w_page[2] = 0;

    /* Write frame number to NDB for firmware to read */
    ndb[NDB_W_D_FN] = (uint16_t)(s->fn & 0xFFFF);
}

/* DSP completion timer callback — fires after "DSP processing" delay */
static void calypso_dsp_done(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    /* Fire DSP API interrupt */
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_API]);
}

/* =====================================================================
 * TPU — Time Processing Unit
 * ===================================================================== */

static uint64_t calypso_tpu_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    switch (offset) {
    case TPU_CTRL:
        val = s->tpu_regs[TPU_CTRL / 2];
        break;
    case TPU_IDLE:
        val = 1;  /* Always idle (processing is instant) */
        break;
    case TPU_INT_CTRL:
        val = s->tpu_regs[TPU_INT_CTRL / 2];
        break;
    case TPU_INT_STAT:
        val = 0;  /* No pending TPU interrupts */
        break;
    case TPU_DSP_PAGE:
        val = s->dsp_page;
        break;
    case TPU_FRAME:
        val = (uint16_t)(s->fn % 2715648);
        break;
    case TPU_OFFSET:
        val = s->tpu_regs[TPU_OFFSET / 2];
        break;
    case TPU_SYNCHRO:
        val = s->tpu_regs[TPU_SYNCHRO / 2];
        break;
    default:
        if (offset >= TPU_RAM_BASE && offset < TPU_RAM_BASE + sizeof(s->tpu_ram)) {
            val = s->tpu_ram[(offset - TPU_RAM_BASE) / 2];
        } else {
            val = s->tpu_regs[offset / 2 < CALYPSO_TPU_SIZE / 2 ?
                              offset / 2 : 0];
        }
        break;
    }

#if TRX_DEBUG_TPU
    TRX_LOG("TPU read  [0x%04x] = 0x%04x", (unsigned)offset, (unsigned)val);
#endif
    return val;
}

static void calypso_tpu_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

#if TRX_DEBUG_TPU
    TRX_LOG("TPU write [0x%04x] = 0x%04x", (unsigned)offset, (unsigned)value);
#endif

    /* Store register */
    if (offset / 2 < CALYPSO_TPU_SIZE / 2) {
        s->tpu_regs[offset / 2] = (uint16_t)value;
    }

    /* TPU instruction RAM */
    if (offset >= TPU_RAM_BASE && offset < TPU_RAM_BASE + sizeof(s->tpu_ram)) {
        s->tpu_ram[(offset - TPU_RAM_BASE) / 2] = (uint16_t)value;
        return;
    }

    switch (offset) {
    case TPU_CTRL:
        if ((value & TPU_CTRL_ENABLE) && !s->tpu_enabled) {
            /* TPU enabled — firmware triggered DSP processing */
            s->tpu_enabled = true;

            /* Process DSP tasks immediately */
            calypso_dsp_process(s);

            /* Schedule DSP completion IRQ after small delay (10 µs) */
            timer_mod_ns(s->dsp_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
        }
        if (value & TPU_CTRL_RESET) {
            s->tpu_enabled = false;
        }
        break;

    case TPU_OFFSET:
    case TPU_SYNCHRO:
        /* Stored above, no special action */
        break;

    case TPU_INT_CTRL:
        /* Interrupt control — stored above */
        break;

    case TPU_DSP_PAGE:
        s->dsp_page = value & 1;
        break;
    }
}

static const MemoryRegionOps calypso_tpu_ops = {
    .read = calypso_tpu_read,
    .write = calypso_tpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* =====================================================================
 * TSP — Time Serial Port (RF transceiver control)
 * ===================================================================== */

static uint64_t calypso_tsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    switch (offset) {
    case TSP_RX_REG:
        val = 0xFFFF;  /* RX data ready (fake) */
        break;
    case TSP_CTRL1:
        val = s->tsp_regs[TSP_CTRL1 / 2];
        break;
    default:
        if (offset / 2 < CALYPSO_TSP_SIZE / 2) {
            val = s->tsp_regs[offset / 2];
        }
        break;
    }

#if TRX_DEBUG_TSP
    TRX_LOG("TSP read  [0x%02x] = 0x%04x", (unsigned)offset, (unsigned)val);
#endif
    return val;
}

static void calypso_tsp_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

#if TRX_DEBUG_TSP
    TRX_LOG("TSP write [0x%02x] = 0x%04x", (unsigned)offset, (unsigned)value);
#endif

    if (offset / 2 < CALYPSO_TSP_SIZE / 2) {
        s->tsp_regs[offset / 2] = (uint16_t)value;
    }
}

static const MemoryRegionOps calypso_tsp_ops = {
    .read = calypso_tsp_read,
    .write = calypso_tsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* =====================================================================
 * ULPD — Ultra Low Power Down (clocks, gauging, GSM timer)
 * ===================================================================== */

static uint64_t calypso_ulpd_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    switch (offset) {
    case ULPD_SETUP_CLK13:
        val = 0x2003;  /* CLK13 setup: enabled, stable */
        break;
    case ULPD_SETUP_SLICER:
        val = 0;
        break;
    case ULPD_SETUP_VTCXO:
        val = 0;
        break;
    case ULPD_COUNTER_HI:
        s->ulpd_counter += 100;  /* Simulate counter advancing */
        val = (s->ulpd_counter >> 16) & 0xFFFF;
        break;
    case ULPD_COUNTER_LO:
        val = s->ulpd_counter & 0xFFFF;
        break;
    case ULPD_GAUGING_CTRL:
        val = 0x0001;  /* Gauging complete */
        break;
    case ULPD_GSM_TIMER:
        val = (uint16_t)(s->fn & 0xFFFF);
        break;
    default:
        if (offset / 2 < CALYPSO_ULPD_SIZE / 2) {
            val = s->ulpd_regs[offset / 2];
        }
        break;
    }

#if TRX_DEBUG_ULPD
    TRX_LOG("ULPD read  [0x%02x] = 0x%04x", (unsigned)offset, (unsigned)val);
#endif
    return val;
}

static void calypso_ulpd_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

#if TRX_DEBUG_ULPD
    TRX_LOG("ULPD write [0x%02x] = 0x%04x", (unsigned)offset, (unsigned)value);
#endif

    if (offset / 2 < CALYPSO_ULPD_SIZE / 2) {
        s->ulpd_regs[offset / 2] = (uint16_t)value;
    }

    switch (offset) {
    case ULPD_GAUGING_CTRL:
        /* Firmware starts gauging — we complete instantly */
        break;
    }
}

static const MemoryRegionOps calypso_ulpd_ops = {
    .read = calypso_ulpd_read,
    .write = calypso_ulpd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* =====================================================================
 * TDMA frame timer — drives L1 at 4.615 ms per frame
 * ===================================================================== */

static void calypso_tdma_tick(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    /* Advance frame number */
    s->fn = (s->fn + 1) % GSM_HYPERFRAME;

    /* Reset TPU enabled flag (new frame, new scenario needed) */
    s->tpu_enabled = false;
    s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_ENABLE;

#if TRX_DEBUG_TDMA
    if ((s->fn % 1000) == 0) {
        TRX_LOG("TDMA FN=%u", s->fn);
    }
#endif

    /* Fire TPU frame interrupt — this wakes up L1 */
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

    /* Schedule next frame */
    if (s->tdma_running) {
        timer_mod_ns(s->tdma_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
    }
}

/* =====================================================================
 * Public interface: start/stop TDMA
 *
 * The firmware will call these via TPU init or we auto-start.
 * For now, we auto-start when the machine initializes.
 * ===================================================================== */

static void calypso_tdma_start(CalypsoTRX *s)
{
    if (s->tdma_running) return;
    s->tdma_running = true;
    s->fn = 0;
    TRX_LOG("TDMA started (4.615ms frame timer)");
    timer_mod_ns(s->tdma_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
}

/* =====================================================================
 * DSP API RAM initialization
 *
 * Pre-fill API RAM with values the firmware expects on boot.
 * ===================================================================== */

static void calypso_dsp_api_init(CalypsoTRX *s)
{
    memset(s->dsp_ram, 0, sizeof(s->dsp_ram));

    /*
     * The DSP ROM normally boots and writes these values.
     * We fake them so the firmware doesn't hang waiting for DSP boot.
     *
     * Key locations in API RAM (byte offsets from base):
     *   0x0000: Should contain DSP ID after boot
     *   NDB area: various control words
     */

    /* DSP alive marker — firmware checks this during dsp_init() */
    s->dsp_ram[0] = 0x0001;

    /* NDB area: set d_dsp_page to 0 */
    s->dsp_page = 0;
    s->dsp_ram[DSP_API_NDB / 2 + NDB_W_D_DSP_PAGE] = 0;

    TRX_LOG("DSP API RAM initialized (%d KiB at 0x%08x)",
            CALYPSO_DSP_SIZE / 1024, CALYPSO_DSP_BASE);
}

/* =====================================================================
 * calypso_trx_init() — Main entry point
 * ===================================================================== */

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs, int trx_port)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s;
    s->irqs = irqs;
    s->trx_fd = -1;

    TRX_LOG("=== Calypso TRX bridge init ===");

    /* ---- DSP API RAM ---- */
    memory_region_init_io(&s->dsp_iomem, NULL, &calypso_dsp_ops, s,
                          "calypso.dsp_api", CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_DSP_BASE, &s->dsp_iomem);
    calypso_dsp_api_init(s);

    /* ---- TPU ---- */
    memory_region_init_io(&s->tpu_iomem, NULL, &calypso_tpu_ops, s,
                          "calypso.tpu", CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TPU_BASE, &s->tpu_iomem);

    /* ---- TSP ---- */
    memory_region_init_io(&s->tsp_iomem, NULL, &calypso_tsp_ops, s,
                          "calypso.tsp", CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TSP_BASE, &s->tsp_iomem);

    /* ---- ULPD ---- */
    memory_region_init_io(&s->ulpd_iomem, NULL, &calypso_ulpd_ops, s,
                          "calypso.ulpd", CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_ULPD_BASE, &s->ulpd_iomem);

    /* ---- TDMA frame timer ---- */
    s->tdma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_tdma_tick, s);

    /* ---- DSP completion timer ---- */
    s->dsp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_dsp_done, s);

    /* ---- TRX UDP socket ---- */
    if (trx_port > 0) {
        trx_socket_init(s, trx_port);
    } else {
        TRX_LOG("TRX UDP disabled (use -M calypso-high,trx-port=4729 to enable)");
    }

    /* ---- Auto-start TDMA ---- */
    calypso_tdma_start(s);

    TRX_LOG("=== TRX bridge ready ===");
    TRX_LOG("  DSP API:  0x%08x (%d KiB)", CALYPSO_DSP_BASE,
            CALYPSO_DSP_SIZE / 1024);
    TRX_LOG("  TPU:      0x%08x", CALYPSO_TPU_BASE);
    TRX_LOG("  TSP:      0x%08x", CALYPSO_TSP_BASE);
    TRX_LOG("  ULPD:     0x%08x", CALYPSO_ULPD_BASE);
    TRX_LOG("  TDMA:     4.615ms frame timer → IRQ %d", CALYPSO_IRQ_TPU_FRAME);
    TRX_LOG("  DSP done: → IRQ %d", CALYPSO_IRQ_API);
    if (s->trx_fd >= 0) {
        TRX_LOG("  TRX UDP:  port %d", trx_port);
    }
}
