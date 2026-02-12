/* Initialization for the Compal E88 (Motorola C115...C123) */

/* (C) 2010 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <stdint.h>
#include <stdio.h>

#include <debug.h>
#include <ctors.h>
#include <memory.h>
#include <board.h>
#include <keypad.h>
#include <console.h>
#include <flash/cfi_flash.h>

#include <calypso/irq.h>
#include <calypso/clock.h>
#include <calypso/dma.h>
#include <calypso/rtc.h>
#include <calypso/timer.h>
#include <uart.h>
#include <calypso/backlight.h>

#include <comm/sercomm.h>
#include <comm/timer.h>

#include <abb/twl3025.h>
#include <rf/trf6151.h>
#include <fb/framebuffer.h>
#include <battery/compal_e88.h>
#include "../compal/keymap.h"

#define ARMIO_LATCH_OUT 0xfffe4802
#define IO_CNTL_REG	0xfffe4804
#define ASIC_CONF_REG	0xfffef008

static void board_io_init(void)
{
	uint16_t reg;

	reg = readw(ASIC_CONF_REG);
	/* LCD Set I/O(3) / SA0 to I/O(3) mode */
	reg &= ~(1 << 10);
	/* Set function pins to I2C Mode */
	reg |= ((1 << 12) | (1 << 7));		/* SCL / SDA */
	/* TWL3025: Set SPI+RIF RX clock to rising edge */
	reg |= (1 << 13) | (1 << 14);
	writew(reg, ASIC_CONF_REG);

	/* LCD Set I/O(3) to output mode */
	reg = readw(IO_CNTL_REG);
	reg &= ~(1 << 3);
	writew(reg, IO_CNTL_REG);

	/* LCD Set I/O(3) output low */
	reg = readw(ARMIO_LATCH_OUT);
	reg &= ~(1 << 3);
	writew(reg, ARMIO_LATCH_OUT);
}

void board_init(int with_irq)
{
        /* clocks + memory */
        calypso_mem_cfg(CALYPSO_nCS0, 3, CALYPSO_MEM_16bit, 1);
        calypso_clock_set(2, CALYPSO_PLL13_104_MHZ, ARM_MCLK_DIV_2);

        board_io_init();

        calypso_bootrom(with_irq);
        calypso_exceptions_install();

        if (with_irq)
                irq_init();

        /* UART only */
        sercomm_bind_uart(UART_MODEM);
        cons_bind_uart(UART_MODEM);

        uart_init(UART_MODEM, with_irq);
        uart_baudrate(UART_MODEM, UART_115200);

        puts("QEMU CALYPSO LOADER ALIVE\n");
}

