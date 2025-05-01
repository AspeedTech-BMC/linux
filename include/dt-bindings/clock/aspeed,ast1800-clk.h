/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for AST2700 clock controller.
 *
 * Copyright (c) 2023 Aspeed Technology Inc.
 */

#ifndef __DT_BINDINGS_CLOCK_AST1800_H
#define __DT_BINDINGS_CLOCK_AST1800_H

#define AST1800_CLK_GATE_MCU         (0)
#define AST1800_CLK_GATE_CONFIG      (1)
#define AST1800_CLK_GATE_MEM         (2)
#define AST1800_CLK_GATE_LTPI        (3)
#define AST1800_CLK_GATE_SPI_SLAVE   (4)
#define AST1800_CLK_GATE_I2C_SLAVE   (5)
#define AST1800_CLK_GATE_SRAM        (6)
#define AST1800_CLK_GATE_UART_DBG    (7)
/* reserved bit 8 ~ 31 */
#define AST1800_CLK_GATE_I3C0        (32)
#define AST1800_CLK_GATE_I3C1        (33)
#define AST1800_CLK_GATE_I3C2        (34)
#define AST1800_CLK_GATE_I3C3        (35)
#define AST1800_CLK_GATE_I3C4        (36)
#define AST1800_CLK_GATE_I3C5        (37)
#define AST1800_CLK_GATE_I3C6        (38)
#define AST1800_CLK_GATE_I3C7        (39)
#define AST1800_CLK_GATE_I3C8        (40)
#define AST1800_CLK_GATE_I3C9        (41)
#define AST1800_CLK_GATE_I3C10       (42)
#define AST1800_CLK_GATE_I3C11       (43)
#define AST1800_CLK_GATE_I3C12       (44)
#define AST1800_CLK_GATE_I3C13       (45)
#define AST1800_CLK_GATE_I3C14       (46)
#define AST1800_CLK_GATE_I3C15       (47)
/* reserved bit 16 ~ 31 */
#define AST1800_CLK_GATE_EFPGA0      (64)
#define AST1800_CLK_GATE_EFPGA1      (65)
#define AST1800_CLK_GATE_EFPGA2      (66)
#define AST1800_CLK_GATE_EFPGA3      (67)
#define AST1800_CLK_GATE_EFPGA4      (68)
#define AST1800_CLK_GATE_EFPGA5      (69)
#define AST1800_CLK_GATE_EFPGA6      (70)
#define AST1800_CLK_GATE_EFPGA7      (71)
#define AST1800_CLK_GATE_EFPGA_DBG   (72)

#define AST1800_CLKIN                (AST1800_CLK_GATE_EFPGA_DBG + 0)
#define AST1800_CLK_HPLL             (AST1800_CLK_GATE_EFPGA_DBG + 1)

#define AST1800_CLK_HPLL_DIV2        (AST1800_CLK_GATE_EFPGA_DBG + 2)
#define AST1800_CLK_HPLL_DIV4        (AST1800_CLK_GATE_EFPGA_DBG + 3)
#define AST1800_CLK_HPLL_DIV5        (AST1800_CLK_GATE_EFPGA_DBG + 4)

#define AST1800_CLK_EPLL             (AST1800_CLK_GATE_EFPGA_DBG + 5)
#define AST1800_CLK_LPLL             (AST1800_CLK_GATE_EFPGA_DBG + 6)
#define AST1800_CLK_UXCLK            (AST1800_CLK_GATE_EFPGA_DBG + 7)
#define AST1800_CLK_HUXCLK           (AST1800_CLK_GATE_EFPGA_DBG + 8)

#define AST1800_CLK_UARTX            (AST1800_CLK_GATE_EFPGA_DBG + 9)
#define AST1800_CLK_HUARTX           (AST1800_CLK_GATE_EFPGA_DBG + 10)

#define AST1800_CLK_AHB              (AST1800_CLK_GATE_EFPGA_DBG + 11)
#define AST1800_CLK_APB              (AST1800_CLK_GATE_EFPGA_DBG + 12)
#define AST1800_CLK_I3C              (AST1800_CLK_GATE_EFPGA_DBG + 13)

#define AST1800_NUM_CLKS             (AST1800_CLK_I3C + 1)

#endif
