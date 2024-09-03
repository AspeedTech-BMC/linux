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

#define SCU0_RESET_CTRL1 0x200
#define SCU0_RESET_CTRL2 0x220
#define SCU1_RESET_CTRL1 0x200
#define SCU1_RESET_CTRL2 0x220
#define SCU1_PCIE3_CTRL 0x908

struct aspeed_reset;

struct ast2700_reset_signal {
	int dedicated_clr; /* dedicated reset clr offset */
	u32 offset, bit;
};

struct aspeed_reset_info {
	unsigned int nr_resets;
	struct ast2700_reset_signal const *signal;
	void	(*reset_init)(struct aspeed_reset *reset);
};

struct aspeed_reset {
	struct reset_controller_dev rcdev;
	struct aspeed_reset_info *info;
	spinlock_t lock; /* protect register read-modify-write cycle */
	void __iomem *base;
};

static const struct ast2700_reset_signal ast2700_reset0_signals[] = {
	[SCU0_RESET_SDRAM] = { 1, SCU0_RESET_CTRL1, BIT(0) },
	[SCU0_RESET_DDRPHY] = { 1, SCU0_RESET_CTRL1, BIT(1) },
	[SCU0_RESET_RSA]     = { 1, SCU0_RESET_CTRL1, BIT(2) },
	[SCU0_RESET_SHA3]	= { 1, SCU0_RESET_CTRL1, BIT(3) },
	[SCU0_RESET_HACE]	= { 1, SCU0_RESET_CTRL1, BIT(4) },
	[SCU0_RESET_SOC]	= { 1, SCU0_RESET_CTRL1, BIT(5) },
	[SCU0_RESET_VIDEO]	= { 1, SCU0_RESET_CTRL1, BIT(6) },
	[SCU0_RESET_2D]	= { 1, SCU0_RESET_CTRL1, BIT(7) },
	[SCU0_RESET_PCIS]	= { 1, SCU0_RESET_CTRL1, BIT(8) },
	[SCU0_RESET_RVAS0]		= { 1, SCU0_RESET_CTRL1, BIT(9) },
	[SCU0_RESET_RVAS1]		= { 1, SCU0_RESET_CTRL1, BIT(10) },
	[SCU0_RESET_SM3]		= { 1, SCU0_RESET_CTRL1, BIT(11) },
	[SCU0_RESET_SM4]		= { 1, SCU0_RESET_CTRL1, BIT(12) },
	[SCU0_RESET_CRT0]	= { 1, SCU0_RESET_CTRL1, BIT(13) },
	[SCU0_RESET_ECC]	= { 1, SCU0_RESET_CTRL1, BIT(14) },
	[SCU0_RESET_DP_PCI]	= { 1, SCU0_RESET_CTRL1, BIT(15) },
	[SCU0_RESET_UFS]	= { 1, SCU0_RESET_CTRL1, BIT(16) },
	[SCU0_RESET_EMMC]	= { 1, SCU0_RESET_CTRL1, BIT(17) },
	[SCU0_RESET_PCIE1RST]	= { 1, SCU0_RESET_CTRL1, BIT(18) },
	[SCU0_RESET_PCIE1RSTOE]	= { 1, SCU0_RESET_CTRL1, BIT(19) },
	[SCU0_RESET_PCIE0RST]		= { 1, SCU0_RESET_CTRL1, BIT(20) },
	[SCU0_RESET_PCIE0RSTOE]	= { 1, SCU0_RESET_CTRL1, BIT(21) },
	[SCU0_RESET_JTAG]	= { 1, SCU0_RESET_CTRL1, BIT(22) },
	[SCU0_RESET_MCTP0] = { 1, SCU0_RESET_CTRL1, BIT(23) },
	[SCU0_RESET_MCTP1]		= { 1, SCU0_RESET_CTRL1, BIT(24) },
	[SCU0_RESET_XDMA0]	= { 1, SCU0_RESET_CTRL1, BIT(25) },
	[SCU0_RESET_XDMA1]	= { 1, SCU0_RESET_CTRL1, BIT(26) },
	[SCU0_RESET_H2X1]	= { 1, SCU0_RESET_CTRL1, BIT(27) },
	[SCU0_RESET_DP]	= { 1, SCU0_RESET_CTRL1, BIT(28) },
	[SCU0_RESET_DP_MCU]	= { 1, SCU0_RESET_CTRL1, BIT(29) },
	[SCU0_RESET_SSP]	= { 1, SCU0_RESET_CTRL1, BIT(30) },
	[SCU0_RESET_H2X0]	= { 1, SCU0_RESET_CTRL1, BIT(31) },
	[SCU0_RESET_PORTA_VHUB]	= { 1, SCU0_RESET_CTRL2, BIT(0) },
	[SCU0_RESET_PORTA_PHY3]	= { 1, SCU0_RESET_CTRL2, BIT(1) },
	[SCU0_RESET_PORTA_XHCI]	= { 1, SCU0_RESET_CTRL2, BIT(2) },
	[SCU0_RESET_PORTB_VHUB]	= { 1, SCU0_RESET_CTRL2, BIT(3) },
	[SCU0_RESET_PORTB_PHY3]	= { 1, SCU0_RESET_CTRL2, BIT(4) },
	[SCU0_RESET_PORTB_XHCI]	= { 1, SCU0_RESET_CTRL2, BIT(5) },
	[SCU0_RESET_PORTA_VHUB_EHCI]	= { 1, SCU0_RESET_CTRL2, BIT(6) },
	[SCU0_RESET_PORTB_VHUB_EHCI]	= { 1, SCU0_RESET_CTRL2, BIT(7) },
	[SCU0_RESET_UHCI]	= { 1, SCU0_RESET_CTRL2, BIT(8) },
	[SCU0_RESET_TSP]	= { 1, SCU0_RESET_CTRL2, BIT(9) },
	[SCU0_RESET_E2M0]	= { 1, SCU0_RESET_CTRL2, BIT(10) },
	[SCU0_RESET_E2M1]	= { 1, SCU0_RESET_CTRL2, BIT(11) },
	[SCU0_RESET_VLINK]	= { 1, SCU0_RESET_CTRL2, BIT(12) },
};

static const struct ast2700_reset_signal ast2700_reset1_signals[] = {
	[SCU1_RESET_LPC0] = { 1, SCU1_RESET_CTRL1, BIT(0) },
	[SCU1_RESET_LPC1] = { 1, SCU1_RESET_CTRL1, BIT(1) },
	[SCU1_RESET_MII]     = { 1, SCU1_RESET_CTRL1, BIT(2) },
	[SCU1_RESET_PECI]	= { 1, SCU1_RESET_CTRL1, BIT(3) },
	[SCU1_RESET_PWM]	= { 1, SCU1_RESET_CTRL1, BIT(4) },
	[SCU1_RESET_MAC0]	= { 1, SCU1_RESET_CTRL1, BIT(5) },
	[SCU1_RESET_MAC1]	= { 1, SCU1_RESET_CTRL1, BIT(6) },
	[SCU1_RESET_MAC2]	= { 1, SCU1_RESET_CTRL1, BIT(7) },
	[SCU1_RESET_ADC]	= { 1, SCU1_RESET_CTRL1, BIT(8) },
	[SCU1_RESET_SD]		= { 1, SCU1_RESET_CTRL1, BIT(9) },
	[SCU1_RESET_ESPI0]		= { 1, SCU1_RESET_CTRL1, BIT(10) },
	[SCU1_RESET_ESPI1]		= { 1, SCU1_RESET_CTRL1, BIT(11) },
	[SCU1_RESET_JTAG1]		= { 1, SCU1_RESET_CTRL1, BIT(12) },
	[SCU1_RESET_SPI0]	= { 1, SCU1_RESET_CTRL1, BIT(13) },
	[SCU1_RESET_SPI1]	= { 1, SCU1_RESET_CTRL1, BIT(14) },
	[SCU1_RESET_SPI2]	= { 1, SCU1_RESET_CTRL1, BIT(15) },
	[SCU1_RESET_I3C0]	= { 1, SCU1_RESET_CTRL1, BIT(16) },
	[SCU1_RESET_I3C1]	= { 1, SCU1_RESET_CTRL1, BIT(17) },
	[SCU1_RESET_I3C2]	= { 1, SCU1_RESET_CTRL1, BIT(18) },
	[SCU1_RESET_I3C3]	= { 1, SCU1_RESET_CTRL1, BIT(19) },
	[SCU1_RESET_I3C4]		= { 1, SCU1_RESET_CTRL1, BIT(20) },
	[SCU1_RESET_I3C5]	= { 1, SCU1_RESET_CTRL1, BIT(21) },
	[SCU1_RESET_I3C6]	= { 1, SCU1_RESET_CTRL1, BIT(22) },
	[SCU1_RESET_I3C7] = { 1, SCU1_RESET_CTRL1, BIT(23) },
	[SCU1_RESET_I3C8]		= { 1, SCU1_RESET_CTRL1, BIT(24) },
	[SCU1_RESET_I3C9]	= { 1, SCU1_RESET_CTRL1, BIT(25) },
	[SCU1_RESET_I3C10]	= { 1, SCU1_RESET_CTRL1, BIT(26) },
	[SCU1_RESET_I3C11]	= { 1, SCU1_RESET_CTRL1, BIT(27) },
	[SCU1_RESET_I3C12]	= { 1, SCU1_RESET_CTRL1, BIT(28) },
	[SCU1_RESET_I3C13]	= { 1, SCU1_RESET_CTRL1, BIT(29) },
	[SCU1_RESET_I3C14]	= { 1, SCU1_RESET_CTRL1, BIT(30) },
	[SCU1_RESET_I3C15]	= { 1, SCU1_RESET_CTRL1, BIT(31) },
	[SCU1_RESET_MCU0]	= { 1, SCU1_RESET_CTRL2, BIT(0) },
	[SCU1_RESET_MCU1]	= { 1, SCU1_RESET_CTRL2, BIT(1) },
	[SCU1_RESET_H2A_SPI1]	= { 1, SCU1_RESET_CTRL2, BIT(2) },
	[SCU1_RESET_H2A_SPI2]	= { 1, SCU1_RESET_CTRL2, BIT(3) },
	[SCU1_RESET_UART0]	= { 1, SCU1_RESET_CTRL2, BIT(4) },
	[SCU1_RESET_UART1]	= { 1, SCU1_RESET_CTRL2, BIT(5) },
	[SCU1_RESET_UART2]	= { 1, SCU1_RESET_CTRL2, BIT(6) },
	[SCU1_RESET_UART3]	= { 1, SCU1_RESET_CTRL2, BIT(7) },
	[SCU1_RESET_I2C_FILTER]	= { 1, SCU1_RESET_CTRL2, BIT(8) },
	[SCU1_RESET_CALIPTRA]	= { 1, SCU1_RESET_CTRL2, BIT(9) },
	[SCU1_RESET_XDMA]	= { 1, SCU1_RESET_CTRL2, BIT(10) },
	[SCU1_RESET_FSI]	= { 1, SCU1_RESET_CTRL2, BIT(12) },
	[SCU1_RESET_CAN]	= { 1, SCU1_RESET_CTRL2, BIT(13) },
	[SCU1_RESET_MCTP]	= { 1, SCU1_RESET_CTRL2, BIT(14) },
	[SCU1_RESET_I2C]	= { 1, SCU1_RESET_CTRL2, BIT(15) },
	[SCU1_RESET_UART6]	= { 1, SCU1_RESET_CTRL2, BIT(16) },
	[SCU1_RESET_UART7]	= { 1, SCU1_RESET_CTRL2, BIT(17) },
	[SCU1_RESET_UART8]	= { 1, SCU1_RESET_CTRL2, BIT(18) },
	[SCU1_RESET_UART9]	= { 1, SCU1_RESET_CTRL2, BIT(19) },
	[SCU1_RESET_LTPI0]	= { 1, SCU1_RESET_CTRL2, BIT(20) },
	[SCU1_RESET_VGAL]	= { 1, SCU1_RESET_CTRL2, BIT(21) },
	[SCU1_RESET_LTPI1]	= { 1, SCU1_RESET_CTRL2, BIT(22) },
	[SCU1_RESET_ACE]	= { 1, SCU1_RESET_CTRL2, BIT(23) },
	[SCU1_RESET_E2M]	= { 1, SCU1_RESET_CTRL2, BIT(24) },
	[SCU1_RESET_UHCI]	= { 1, SCU1_RESET_CTRL2, BIT(25) },
	[SCU1_RESET_PORTC_USB2UART]	= { 1, SCU1_RESET_CTRL2, BIT(26) },
	[SCU1_RESET_PORTC_VHUB_EHCI]	= { 1, SCU1_RESET_CTRL2, BIT(27) },
	[SCU1_RESET_PORTD_USB2UART]	= { 1, SCU1_RESET_CTRL2, BIT(28) },
	[SCU1_RESET_PORTD_VHUB_EHCI]	= { 1, SCU1_RESET_CTRL2, BIT(29) },
	[SCU1_RESET_H2X]	= { 1, SCU1_RESET_CTRL2, BIT(30) },
	[SCU1_RESET_I3CDMA]	= { 1, SCU1_RESET_CTRL2, BIT(31) },
	[SCU1_RESET_PCIE2RST]	= { 0, SCU1_PCIE3_CTRL, BIT(0) },
};

#define to_aspeed_reset(p) container_of(p, struct aspeed_reset, rcdev)

static int aspeed_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);
	unsigned long flags;

	if (rc->info->signal[id].dedicated_clr) {
		writel(rc->info->signal[id].bit, rc->base);
	} else {
		spin_lock_irqsave(&rc->lock, flags);
		writel(readl(rc->base) & ~rc->info->signal[id].bit, rc->base);
		spin_unlock_irqrestore(&rc->lock, flags);
	}

	return 0;
}

static int aspeed_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);
	unsigned long flags;

	if (rc->info->signal[id].dedicated_clr) {
		writel(rc->info->signal[id].bit, rc->base + 0x04);
	} else {
		spin_lock_irqsave(&rc->lock, flags);
		writel(readl(rc->base) | rc->info->signal[id].bit, rc->base);
		spin_unlock_irqrestore(&rc->lock, flags);
	}

	return 0;
}

static int aspeed_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct aspeed_reset *rc = to_aspeed_reset(rcdev);

	if (readl(rc->base) & rc->info->signal[id].bit)
		return 1;
	else
		return 0;
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
	struct aspeed_reset *reset;
	struct device *dev = &adev->dev;

	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	spin_lock_init(&reset->lock);

	reset->info	= (struct aspeed_reset_info *)(id->driver_data);
	reset->rcdev.owner     = THIS_MODULE;
	reset->rcdev.nr_resets = reset->info->nr_resets;
	reset->rcdev.ops       = &aspeed_reset_ops;
	reset->rcdev.of_node   = dev->parent->of_node;
	reset->rcdev.dev	      = dev;
	reset->rcdev.of_reset_n_cells = 1;
	reset->base            = (void __iomem *)adev->dev.platform_data;
	if (!reset->base)
		return -ENOMEM;

	dev_set_drvdata(dev, reset);
	if (reset->info->reset_init)
		reset->info->reset_init(reset);

	return devm_reset_controller_register(dev, &reset->rcdev);
}

static const struct aspeed_reset_info ast2700_reset0_info = {
	.nr_resets = ARRAY_SIZE(ast2700_reset0_signals),
	.signal = ast2700_reset0_signals,
};

static const struct aspeed_reset_info ast2700_reset1_info = {
	.nr_resets = ARRAY_SIZE(ast2700_reset1_signals),
	.signal = ast2700_reset1_signals,
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
