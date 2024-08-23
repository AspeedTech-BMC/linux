/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 Aspeed Technology Inc.
 */
#ifndef __ASPEED_PCIE_MMBI_H__
#define __ASPEED_PCIE_MMBI_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define __ASPEED_PCIE_MMBI_MAGIC	0xb8

/*
 *  - ASPEED_PCIE_MMBI_HOST_INT
 *      Triggle Host interrupt
 */
#define ASPEED_PCIE_MMBI_HOST_INT	_IO(__ASPEED_PCIE_MMBI_MAGIC, \
					     0x00)

#endif
