/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 Aspeed Technology Inc.
 */
#ifndef __ASPEED_PCIE_MMBI_H__
#define __ASPEED_PCIE_MMBI_H__

struct aspeed_pcie_mmbi;

struct aspeed_mmbi_channel {
	struct aspeed_pcie_mmbi *mmbi;
	struct device *dev;

	bool bmc_int_en;
	u8 bmc_int_value;
	u32 bmc_int_location;
	u8 __iomem *bmc_int_vmem;

	bool host_int_en;
	u8 host_int_location;
	u8 host_int_value;
};

#endif
