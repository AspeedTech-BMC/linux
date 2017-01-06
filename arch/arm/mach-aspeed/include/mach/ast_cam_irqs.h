/*
 *  arch/arm/plat-aspeed/include/plat/irqs.h
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _AST_CAM_IRQS_H_
#define _AST_CAM_IRQS_H_

#define NR_IRQS							(AST_VIC_NUM + AST_FIQ_NUM  + ARCH_NR_I2C + ARCH_NR_GPIOS + ARCH_NR_SDHCI)

#define IRQ_SDHCI_CHAIN_START			(IRQ_GPIO_CHAIN_START + ARCH_NR_SDHCI)

#define IRQ_GPIO_CHAIN_START			(IRQ_I2C_CHAIN_START + ARCH_NR_I2C)

#define IRQ_I2C_CHAIN_START			(AST_VIC_NUM + AST_FIQ_NUM)

#define FIQ_START 						AST_VIC_NUM

#define AST_FIQ_NUM						64

#define AST_VIC_NUM						64

#define IRQ_SDRAM_ECC					0
#define IRQ_MIC							1
#define IRQ_MAC0						2			/* MAC 1 interrupt */
#define IRQ_CRYPTO						4
#define IRQ_VHUB						5
#define IRQ_EHCI0						5
#define IRQ_XDMA						6
#define IRQ_VIDEO						7
#define IRQ_LPC							8
#define IRQ_UART1						9			/* UART 1 interrupt */
#define IRQ_UART0						10			/* UART 3 interrupt */
#define IRQ_EGFX						11			
#define IRQ_I2C							12
#define IRQ_EHCI1						13
#define IRQ_HID							13
#define IRQ_UHCI							14
#define IRQ_PECI							15
#define IRQ_TIMER0						16			/* TIMER 1 interrupt */
#define IRQ_TIMER1						17			/* TIMER 2 interrupt */
#define IRQ_TIMER2						18			/* TIMER 3 interrupt */
#define IRQ_SMC							19
#define IRQ_GPIO							20
#define IRQ_SCU							21
#define IRQ_RTC							22
#define IRQ_ESPI							23
//24 reserverd 
#define IRQ_CRT							25
#define IRQ_SDHC0						26
#define IRQ_WDT							27
#define IRQ_TACHO						28
#define IRQ_2D							29
#define IRQ_SYS_WAKEUP					30
#define IRQ_ADC							31
#define IRQ_UART2						32			/* UART 2 interrupt */
#define IRQ_UART3						33			/* UART 3 interrupt */
#define IRQ_UART4						34			/* UART 4 interrupt */
#define IRQ_TIMER3						35			/* TIMER 4 interrupt */
#define IRQ_TIMER4						36
#define IRQ_TIMER5						37
#define IRQ_TIMER6						38
#define IRQ_TIMER7						39			/* TIMER 8 interrupt */
#define IRQ_SGPIO						40			/* SGPIO Master  interrupt */
#define IRQ_SGPIO_SLAVE					41
#define IRQ_MCTP						42
#define IRQ_JTAG							43
#define IRQ_HOST						44
#define IRQ_CPU							45
#define IRQ_MAILBOX						46
#define IRQ_EXT0							47			/* GPIOL1 */
#define IRQ_EXT1							48			/* GPIOL3 */
#define IRQ_EXT2							49			/* GPIOM1 */
#define IRQ_UART_SDMA					50
#define IRQ_SPI							59			
#define IRQ_H264						61
#define IRQ_FORMATTER					62
#define IRQ_P2X							63

#endif
