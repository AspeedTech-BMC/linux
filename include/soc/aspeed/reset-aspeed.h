/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 ASPEED Technology Inc.
 * Author: Ryan Chen <ryan_chen@aspeedtech.com>
 */

#ifndef __RESET_ASPEED_H__
#define __RESET_ASPEED_H__

#if IS_ENABLED(CONFIG_RESET_ASPEED)
int aspeed_reset_controller_register(struct device *clk_dev, void __iomem *base,
				     const char *adev_name);
#else
int aspeed_reset_controller_register(struct device *clk_dev, void __iomem *base,
				     const char *adev_name)
{
	return -ENODEV;
}
#endif /* if IS_ENABLED(CONFIG_RESET_ASPEED) */

#endif /* __RESET_ASPEED_H__ */
