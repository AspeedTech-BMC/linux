/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for AST1800 reset controller.
 *
 * Copyright (c) 2025 Aspeed Technology Inc.
 */

#ifndef _MACH_ASPEED_AST1800_RESET_H_
#define _MACH_ASPEED_AST1800_RESET_H_

#define AST1800_RESET_CONFIG      (0)
#define AST1800_RESET_MEM         (1)
#define AST1800_RESET_LTPI        (2)
#define AST1800_RESET_EFPGA_DBG   (3)
#define AST1800_RESET_SPI_SLAVE   (4)
#define AST1800_RESET_I2C_SLAVE   (5)
/* reserved 6 7 */
#define AST1800_RESET_I2C_MASTER  (8)
#define AST1800_RESET_I3C_DMA     (9)
#define AST1800_RESET_ADC         (10)
#define AST1800_RESET_GPIO        (11)
#define AST1800_RESET_JTAG        (12)
#define AST1800_RESET_PWM         (13)
#define AST1800_RESET_UART_DBG    (14)
/* reserved 15~31 */
#define AST1800_RESET_I3C0        (32)
#define AST1800_RESET_I3C1        (33)
#define AST1800_RESET_I3C2        (34)
#define AST1800_RESET_I3C3        (35)
#define AST1800_RESET_I3C4        (36)
#define AST1800_RESET_I3C5        (37)
#define AST1800_RESET_I3C6        (38)
#define AST1800_RESET_I3C7        (39)
#define AST1800_RESET_I3C8        (40)
#define AST1800_RESET_I3C9        (41)
#define AST1800_RESET_I3C10       (42)
#define AST1800_RESET_I3C11       (43)
#define AST1800_RESET_I3C12       (44)
#define AST1800_RESET_I3C13       (45)
#define AST1800_RESET_I3C14       (46)
#define AST1800_RESET_I3C15       (47)
#define AST1800_RESET_I2C0        (48)
#define AST1800_RESET_I2C1        (49)
#define AST1800_RESET_I2C2        (50)
#define AST1800_RESET_I2C3        (51)
#define AST1800_RESET_I2C4        (52)
#define AST1800_RESET_I2C5        (53)
#define AST1800_RESET_I2C6        (54)
#define AST1800_RESET_I2C7        (55)
#define AST1800_RESET_I2C8        (56)
#define AST1800_RESET_I2C9        (57)
#define AST1800_RESET_I2C10       (58)
#define AST1800_RESET_I2C11       (59)
#define AST1800_RESET_I2C12       (60)
#define AST1800_RESET_I2C13       (61)
#define AST1800_RESET_I2C14       (62)
#define AST1800_RESET_I2C15       (63)

#define AST1800_RESET_NUMS        (AST1800_RESET_I2C15 + 1)

#endif /* _MACH_ASPEED_AST1800_RESET_H_ */
