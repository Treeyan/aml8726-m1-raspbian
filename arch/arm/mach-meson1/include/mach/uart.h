/*
 *	arch/arm/mach-meson6/include/mach/uart.h
 *
 *  Copyright (C) 2013 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Basic register address definitions in physical memory and
 * some block defintions for core devices like the timer.
 * copy from linux kernel
 */

#ifndef __MACH_MESSON_UART_REGS_H
#define __MACH_MESSON_UART_REGS_H

#define UART_A     0
#define UART_B     1

#define MESON_UART_PORT_NUM 2

#define MESON_UART_NAME     "uart_a","uart_b"
#define MESON_UART_LINE     UART_A,UART_B,
#define MESON_UART_FIFO     64,64
#define MESON_UART_ADDRS    ((void *)P_UART0_WFIFO), ((void *)P_UART1_WFIFO)
#define MESON_UART_IRQS		INT_UART_0, INT_UART_1
#define MESON_UART_CLK_NAME "uart0","uart1"
#endif
