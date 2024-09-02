// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2024 ASPEED Technology Inc.
 */

#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset-controller.h>

#include <dt-bindings/reset/aspeed,ast2700-reset.h>

struct aspeed_reset {
	struct reset_controller_dev rcdev;
	spinlock_t lock; /* protect register read-modify-write cycle */
	void __iomem *base;
};

struct aspeed_reset_info {
	unsigned int nr_resets;
	unsigned int assert_offset;
	unsigned int status_offset;
	void	(*reset_init)(struct aspeed_reset *reset);
};

#define to_aspeed_reset(p) container_of(p, struct aspeed_reset, rcdev)

static int aspeed_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x220 : 0x200;

	if (id == SCU1_RESET_PCIE2RST)
		writel(readl(rc->base + 0x908) & ~BIT(0), rc->base + 0x908);
	else
		writel(rst, rc->base + reg);
	return 0;
}

static int aspeed_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x220 : 0x200;

	if (id == SCU1_RESET_PCIE2RST)
		writel(readl(rc->base + 0x908) | BIT(0), rc->base + 0x908);
	else
		/* Use set to clear register */
		writel(rst, rc->base + reg + 0x04);
	return 0;
}

static int aspeed_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);
	u32 rst = BIT(id % 32);
	u32 reg = id >= 32 ? 0x220 : 0x200;

	return (readl(rc->base + reg) & rst);
}

static const struct reset_control_ops aspeed_reset_ops = {
	.assert = aspeed_reset_assert,
	.deassert = aspeed_reset_deassert,
	.status = aspeed_reset_status,
};

void ast2700_reset_init(struct aspeed_reset *reset)
{
	/*
	 * Ast2700-scu1 A0 workaround:
	 * I3C reset should assert all of the I3C controllers simultaneously.
	 * Otherwise, it may lead to failure in accessing I3C registers.
	 */
	if (!FIELD_GET(GENMASK(23, 16), readl(reset->base)))
		writel(0xffff0000, reset->base);
}

static int aspeed_reset_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	struct aspeed_reset_info *info = (struct aspeed_reset_info *)(id->driver_data);
	struct aspeed_reset *reset;
	struct device *dev = &adev->dev;

	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	spin_lock_init(&reset->lock);

	reset->rcdev.owner     = THIS_MODULE;
	reset->rcdev.nr_resets = info->nr_resets;
	reset->rcdev.ops       = &aspeed_reset_ops;
	reset->rcdev.of_node   = dev->parent->of_node;
	reset->rcdev.dev	      = dev;
	reset->rcdev.of_reset_n_cells = 1;
	reset->base            = (void __iomem *)adev->dev.platform_data;
	if (!reset->base)
		return -ENOMEM;

	dev_set_drvdata(dev, reset);
	if (info->reset_init)
		info->reset_init(reset);

	return devm_reset_controller_register(dev, &reset->rcdev);
}

static const struct aspeed_reset_info ast2700_reset0_info = {
	.nr_resets = SCU0_RESET_NUMS,
	.assert_offset = 0x2F8,
	.status_offset = 0x308,
};

static const struct aspeed_reset_info ast2700_reset1_info = {
	.nr_resets = SCU1_RESET_NUMS,
	.assert_offset = 0x38,
	.status_offset = 0x3C,
	.reset_init = ast2700_reset_init,
};

static const struct auxiliary_device_id aspeed_reset_ids[] = {
	{	.name = "clk_ast2700.reset0",
		.driver_data = (kernel_ulong_t)&ast2700_reset0_info,
	},
	{	.name = "clk_ast2700.reset1",
		.driver_data = (kernel_ulong_t)&ast2700_reset1_info,
	},
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, aspeed_reset_ids);

static struct auxiliary_driver aspeed_reset_driver = {
	.probe		= aspeed_reset_probe,
	.id_table	= aspeed_reset_ids,
};

module_auxiliary_driver(aspeed_reset_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED SoC Reset Controller Driver");
MODULE_LICENSE("GPL");
