/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
/*
 * Copyright 2024 NXP
 */

#ifndef _DT_BINDINGS_I3C_I3C_H
#define _DT_BINDINGS_I3C_I3C_H

#define I2C_FM      (1 << 4)
#define I2C_FM_PLUS (0 << 4)

#define I2C_FILTER  (0 << 5)
#define I2C_NO_FILTER_HIGH_FREQUENCY    (1 << 5)
#define I2C_NO_FILTER_LOW_FREQUENCY     (2 << 5)

#define I3C_BUS_CONTEXT_MIPI_BASIC_V1_0_0	0x10
#define I3C_BUS_CONTEXT_MIPI_BASIC_V1_1_0	0x11
#define I3C_BUS_CONTEXT_MIPI_BASIC_V1_1_1	0x31
#define I3C_BUS_CONTEXT_MIPI_BASIC_V1_2_0	0x12
#define I3C_BUS_CONTEXT_JESD403			0x80
#define I3C_BUS_CONTEXT_MCTP			0x81

#endif
