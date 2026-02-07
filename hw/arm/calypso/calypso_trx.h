/*
 * calypso_trx.h — Calypso DSP/TPU/TRX bridge for virtual GSM
 */

#ifndef CALYPSO_TRX_H
#define CALYPSO_TRX_H

#include "hw/irq.h"
#include "exec/memory.h"

/* =====================================================================
 * Correct Calypso IRQ map (from OsmocomBB calypso/irq.h)
 * ===================================================================== */

#define CALYPSO_IRQ_WATCHDOG       0
#define CALYPSO_IRQ_TIMER1         1
#define CALYPSO_IRQ_TIMER2         2
#define CALYPSO_IRQ_TSP_RX         3
#define CALYPSO_IRQ_TPU_FRAME      4
#define CALYPSO_IRQ_TPU_PAGE       5
#define CALYPSO_IRQ_SIM            6
#define CALYPSO_IRQ_UART_MODEM     7
#define CALYPSO_IRQ_KEYPAD_GPIO    8
#define CALYPSO_IRQ_RTC_TIMER      9
#define CALYPSO_IRQ_RTC_ALARM      10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14
#define CALYPSO_IRQ_API            15
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_NUM_IRQS           32

/* =====================================================================
 * Hardware addresses
 * ===================================================================== */

#define CALYPSO_DSP_BASE      0xFFD00000
#define CALYPSO_DSP_SIZE      (64 * 1024)

#define DSP_API_W_PAGE0       0x0000
#define DSP_API_W_PAGE1       0x1000
#define DSP_API_R_PAGE0       0x2000
#define DSP_API_R_PAGE1       0x3000
#define DSP_API_NDB           0x4000
#define DSP_API_PARAM         0x4800
#define DSP_PAGE_SIZE         0x1000

#define CALYPSO_TPU_BASE      0xFFFE0000
#define CALYPSO_TPU_SIZE      0x0800

#define CALYPSO_TSP_BASE      0xFFFE0800
#define CALYPSO_TSP_SIZE      0x0100

#define CALYPSO_ULPD_BASE     0xFFFE2800
#define CALYPSO_ULPD_SIZE     0x0100

/* TPU register offsets */
#define TPU_CTRL              0x0000
#define TPU_IDLE              0x0002
#define TPU_INT_CTRL          0x0004
#define TPU_INT_STAT          0x0006
#define TPU_DSP_PAGE          0x0008
#define TPU_FRAME             0x000A
#define TPU_OFFSET            0x000C
#define TPU_SYNCHRO           0x000E
#define TPU_IT_DSP_PG         0x0020
#define TPU_RAM_BASE          0x0400

#define TPU_CTRL_ENABLE       (1 << 0)
#define TPU_CTRL_RESET        (1 << 1)
#define TPU_CTRL_IDLE         (1 << 2)

/* TSP register offsets */
#define TSP_TX_REG            0x00
#define TSP_CTRL1             0x02
#define TSP_CTRL2             0x04
#define TSP_TX_SIZE           0x06
#define TSP_RX_REG            0x08
#define TSP_MASK1             0x0A
#define TSP_ACT               0x0C
#define TSP_ACT_L             0x0E

/* ULPD register offsets */
#define ULPD_SETUP_CLK13      0x00
#define ULPD_SETUP_SLICER     0x02
#define ULPD_SETUP_VTCXO      0x04
#define ULPD_SETUP_RF         0x06
#define ULPD_DCXO_SETUP       0x08
#define ULPD_ITP_1            0x12
#define ULPD_ITP_2            0x14
#define ULPD_COUNTER_HI       0x1C
#define ULPD_COUNTER_LO       0x1E
#define ULPD_GAUGING_CTRL     0x24
#define ULPD_GSM_TIMER        0x28

/* GSM timing */
#define GSM_TDMA_NS           4615000
#define GSM_HYPERFRAME        2715648
#define GSM_BURST_BITS        148
#define GSM_BURST_WORDS       78

/* TRX UDP protocol (TRXD v0) */
#define TRX_DEFAULT_PORT      4729
#define TRX_HDR_LEN_TX        6
#define TRX_HDR_LEN_RX        8
#define TRX_BURST_LEN         148
#define TRX_PKT_LEN_TX        (TRX_HDR_LEN_TX + TRX_BURST_LEN)
#define TRX_PKT_LEN_RX        (TRX_HDR_LEN_RX + TRX_BURST_LEN)

/* NDB key offsets (word offsets from NDB base) */
#define NDB_W_D_DSP_PAGE      0x0000
#define NDB_W_D_TASK_D        0x0001
#define NDB_W_D_TASK_U        0x0002
#define NDB_W_D_FN            0x0003

/* Public interface */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs, int trx_port);

#endif /* CALYPSO_TRX_H */
