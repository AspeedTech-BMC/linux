// SPDX-License-Identifier: GPL-2.0-or-later
/* Copyright (C) 2019 ASPEED Technology Inc. */
/* Copyright (C) 2019 IBM Corp. */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#include "sdhci-pltfm.h"

#define ASPEED_SDC_INFO		0x00
#define   ASPEED_SDC_S1MMC8	BIT(25)
#define   ASPEED_SDC_S0MMC8	BIT(24)
#define ASPEED_SDC_PHASE		0xf4
#define   ASPEED_SDC_S1_PHASE_IN	GENMASK(25, 21)
#define   ASPEED_SDC_S0_PHASE_IN	GENMASK(20, 16)
#define	  ASPEED_SDC_S0_PHASE_IN_SHIFT	16
#define   ASPEED_SDC_S1_PHASE_OUT	GENMASK(15, 11)
#define   ASPEED_SDC_S1_PHASE_IN_EN	BIT(10)
#define   ASPEED_SDC_S1_PHASE_OUT_EN	GENMASK(9, 8)
#define   ASPEED_SDC_S0_PHASE_OUT	GENMASK(7, 3)
#define   ASPEED_SDC_S0_PHASE_IN_EN	BIT(2)
#define   ASPEED_SDC_S0_PHASE_OUT_EN	GENMASK(1, 0)
#define   ASPEED_SDC_PHASE_MAX		31

#define TIMING_PHASE_OFFSET 0xF4

#define PROBE_AFTER_ASSET_DEASSERT 0x1

#define ASPEED_SDHCI_TAP_PARAM_INVERT_CLK       BIT(4)
#define ASPEED_SDHCI_NR_TAPS            15

struct aspeed_sdc_info {
	uint32_t flag;
};

struct aspeed_sdc {
	struct clk *clk;
	struct resource *res;
	struct reset_control *rst;

	spinlock_t lock;
	void __iomem *regs;
};

struct aspeed_sdhci {
	struct aspeed_sdc *parent;
	u32 width_mask;
	int	pwr_pin;
	int	pwr_sw_pin;
};

struct aspeed_sdc_info ast2600_sdc_info = {
	.flag = PROBE_AFTER_ASSET_DEASSERT
};

#ifdef CONFIG_MACH_ASPEED_G6
static void aspeed_sdc_configure_8bit_mode(struct aspeed_sdc *sdc,
					   struct aspeed_sdhci *sdhci,
					   bool bus8)
{
	u32 info;

	/* Set/clear 8 bit mode */
	spin_lock(&sdc->lock);
	info = readl(sdc->regs + ASPEED_SDC_INFO);
	if (bus8)
		info |= sdhci->width_mask;
	else
		info &= ~sdhci->width_mask;
	writel(info, sdc->regs + ASPEED_SDC_INFO);
	spin_unlock(&sdc->lock);
}

static void aspeed_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
#ifdef CONFIG_MACH_ASPEED_G6
	sdhci_set_clock(host, clock);
#else
	struct sdhci_pltfm_host *pltfm_host;
	unsigned long parent;
	int div;
	u16 clk;

	pltfm_host = sdhci_priv(host);
	parent = clk_get_rate(pltfm_host->clk);
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	if (WARN_ON(clock > host->max_clk))
		clock = host->max_clk;

	for (div = 1; div < 256; div *= 2) {
		if ((parent / div) <= clock)
			break;
	}
	div >>= 1;

	//Issue : For ast2300, ast2400 couldn't set div = 0 means /1 , so set source is ~50Mhz up
	clk = div << SDHCI_DIVIDER_SHIFT;

	sdhci_enable_clk(host, clk);
#endif
}

static void aspeed_sdhci_set_bus_width(struct sdhci_host *host, int width)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *aspeed_sdhci;
	struct aspeed_sdc *aspeed_sdc;
	u8 ctrl;

	pltfm_priv = sdhci_priv(host);
	aspeed_sdhci = sdhci_pltfm_priv(pltfm_priv);
	aspeed_sdc = aspeed_sdhci->parent;

	/* Set/clear 8-bit mode */
	aspeed_sdc_configure_8bit_mode(aspeed_sdc, aspeed_sdhci,
				       width == MMC_BUS_WIDTH_8);

	/* Set/clear 1 or 4 bit mode */
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void sdhci_aspeed_set_power(struct sdhci_host *host, unsigned char mode,
						   unsigned short vdd)
{
	struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);
	struct aspeed_sdhci *dev = sdhci_pltfm_priv(pltfm_priv);
	u8 pwr = 0;


	if (dev->pwr_pin <= 0)
		return sdhci_set_power(host, mode, vdd);

	if (mode != MMC_POWER_OFF) {
			switch (1 << vdd) {
			case MMC_VDD_165_195:
			/*
			 * Without a regulator, SDHCI does not support 2.0v
			 * so we only get here if the driver deliberately
			 * added the 2.0v range to ocr_avail. Map it to 1.8v
			 * for the purpose of turning on the power.
			 */
			case MMC_VDD_20_21:
					pwr = SDHCI_POWER_180;
					break;
			case MMC_VDD_29_30:
			case MMC_VDD_30_31:
					pwr = SDHCI_POWER_300;
					break;
			case MMC_VDD_32_33:
			case MMC_VDD_33_34:
					pwr = SDHCI_POWER_330;
					break;
			default:
					WARN(1, "%s: Invalid vdd %#x\n",
						 mmc_hostname(host->mmc), vdd);
					break;
			}
	}

	if (host->pwr == pwr)
		return;

    host->pwr = pwr;

	if (pwr == 0) {
		if (gpio_is_valid(dev->pwr_pin))
			gpio_set_value(dev->pwr_pin, 0);

		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
	} else {
		pwr |= SDHCI_POWER_ON;

		if (pwr & SDHCI_POWER_ON) {
			if (gpio_is_valid(dev->pwr_pin))
				gpio_set_value(dev->pwr_pin, 1);
		}

		if (pwr & SDHCI_POWER_330) {
			if (gpio_is_valid(dev->pwr_sw_pin))
				gpio_set_value(dev->pwr_sw_pin, 1);
		} else if (pwr & SDHCI_POWER_180) {
			if (gpio_is_valid(dev->pwr_sw_pin))
				gpio_set_value(dev->pwr_sw_pin, 0);
		} else {
			dev_err(&host->mmc->class_dev, "todo fail check ~~\n");
		}

		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);
	}
}

static void aspeed_sdhci_voltage_switch(struct sdhci_host *host)
{
    struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);
	struct aspeed_sdhci *dev = sdhci_pltfm_priv(pltfm_priv);

	if (dev->pwr_sw_pin <= 0) {
		return;
	}

	if (gpio_is_valid(dev->pwr_sw_pin))
		gpio_set_value(dev->pwr_sw_pin, 0);
}

static void aspeed_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *aspeed_sdhci;
	struct aspeed_sdc *aspeed_sdc;
	u32 save_array[8];
	u32 reg_array[] = {SDHCI_DMA_ADDRESS,
			SDHCI_BLOCK_SIZE,
			SDHCI_ARGUMENT,
			SDHCI_HOST_CONTROL,
			SDHCI_CLOCK_CONTROL,
			SDHCI_INT_ENABLE,
			SDHCI_SIGNAL_ENABLE,
			SDHCI_AUTO_CMD_STATUS};
	int i;
	u16 tran_mode;
	u32 mmc8_mode;
	u32 clk_phase;

	pltfm_priv = sdhci_priv(host);
	aspeed_sdhci = sdhci_pltfm_priv(pltfm_priv);
	aspeed_sdc = aspeed_sdhci->parent;

	if (!IS_ERR(aspeed_sdc->rst)) {
		for (i = 0; i < ARRAY_SIZE(reg_array); i++)
			save_array[i] = sdhci_readl(host, reg_array[i]);

		tran_mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
		mmc8_mode = readl(aspeed_sdc->regs);
		clk_phase = readl(aspeed_sdc->regs + ASPEED_SDC_PHASE);

		reset_control_assert(aspeed_sdc->rst);
		mdelay(1);
		reset_control_deassert(aspeed_sdc->rst);
		mdelay(1);

		for (i = 0; i < ARRAY_SIZE(reg_array); i++)
			sdhci_writel(host, save_array[i], reg_array[i]);

		sdhci_writew(host, tran_mode, SDHCI_TRANSFER_MODE);
		writel(mmc8_mode, aspeed_sdc->regs);
		writel(clk_phase, aspeed_sdc->regs + ASPEED_SDC_PHASE);

		aspeed_sdhci_set_clock(host, host->clock);
	}

	sdhci_reset(host, mask);
}

static int aspeed_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_pltfm_host *pltfm_priv;
	struct aspeed_sdhci *sdhci;
	struct aspeed_sdc *sdc;
	struct device *dev;

	u32 val, left, right, edge;
	u32 window, oldwindow = 0, center;
	u32 in_phase, out_phase, enable_mask, inverted = 0;

	dev = mmc_dev(host->mmc);
	pltfm_priv = sdhci_priv(host);
	sdhci = sdhci_pltfm_priv(pltfm_priv);
	sdc = sdhci->parent;

	out_phase = readl(sdc->regs + ASPEED_SDC_PHASE) & ASPEED_SDC_S0_PHASE_OUT;

	enable_mask = ASPEED_SDC_S0_PHASE_OUT_EN | ASPEED_SDC_S0_PHASE_IN_EN;

	/*
	 * There are two window upon clock rising and falling edge.
	 * Iterate each tap delay to find the valid window and choose the
	 * bigger one, set the tap delay at the middle of window.
	 */
	for (edge = 0; edge < 2; edge++) {
		if (edge == 1)
			inverted = ASPEED_SDHCI_TAP_PARAM_INVERT_CLK;

		val = (out_phase | enable_mask | (inverted << ASPEED_SDC_S0_PHASE_IN_SHIFT));

		/* find the left boundary */
		for (left = 0; left < ASPEED_SDHCI_NR_TAPS + 1; left++) {
			in_phase = val | (left << ASPEED_SDC_S0_PHASE_IN_SHIFT);
			writel(in_phase, sdc->regs + ASPEED_SDC_PHASE);

			if (!mmc_send_tuning(host->mmc, opcode, NULL))
				break;
		}

		/* find the right boundary */
		for (right = left + 1; right < ASPEED_SDHCI_NR_TAPS + 1; right++) {
			in_phase = val | (right << ASPEED_SDC_S0_PHASE_IN_SHIFT);
			writel(in_phase, sdc->regs + ASPEED_SDC_PHASE);

			if (mmc_send_tuning(host->mmc, opcode, NULL))
				break;
		}

		window = right - left;

		if (window > oldwindow) {
			oldwindow = window;
			center = (((right - 1) + left) / 2) | inverted;
		}
	}

	val = (out_phase | enable_mask | (center << ASPEED_SDC_S0_PHASE_IN_SHIFT));
	writel(val, sdc->regs + ASPEED_SDC_PHASE);

	return mmc_send_tuning(host->mmc, opcode, NULL);
}

/*
	AST2300/AST2400 : SDMA/PIO
	AST2500 : ADMA/SDMA/PIO
*/
static struct sdhci_ops aspeed_sdhci_ops = {
	.set_power = sdhci_aspeed_set_power,
	.voltage_switch = aspeed_sdhci_voltage_switch,
	.set_clock = aspeed_sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = aspeed_sdhci_set_bus_width,
	.get_timeout_clock = sdhci_pltfm_clk_get_max_clock,
	.reset = aspeed_sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.platform_execute_tuning = aspeed_sdhci_execute_tuning,
};
#endif

static struct sdhci_pltfm_data aspeed_sdhci_pdata = {
#ifndef CONFIG_MACH_ASPEED_G6
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN | SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
#else
	.ops = &aspeed_sdhci_ops,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
#endif
};

static inline int aspeed_sdhci_calculate_slot(struct aspeed_sdhci *dev,
					      struct resource *res)
{
	resource_size_t delta;

	if (!res || resource_type(res) != IORESOURCE_MEM)
		return -EINVAL;

	if (res->start < dev->parent->res->start)
		return -EINVAL;

	delta = res->start - dev->parent->res->start;
	if (delta & (0x100 - 1))
		return -EINVAL;

	return (delta / 0x100) - 1;
}

static int aspeed_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct device_node *np = pdev->dev.of_node;
	struct aspeed_sdhci *dev;
	struct sdhci_host *host;
	struct resource *res;
	uint32_t reg_val;
	int slot;
	int ret;

	host = sdhci_pltfm_init(pdev, &aspeed_sdhci_pdata, sizeof(*dev));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	dev = sdhci_pltfm_priv(pltfm_host);
	dev->parent = dev_get_drvdata(pdev->dev.parent);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	slot = aspeed_sdhci_calculate_slot(dev, res);

	if (slot < 0)
		return slot;
	else if (slot >= 2)
		return -EINVAL;

	dev_info(&pdev->dev, "Configuring for slot %d\n", slot);
	dev->width_mask = !slot ? ASPEED_SDC_S0MMC8 : ASPEED_SDC_S1MMC8;

	sdhci_get_of_property(pdev);

	if (of_property_read_bool(pdev->dev.parent->of_node, "mmc-hs200-1_8v") ||
		of_property_read_bool(pdev->dev.parent->of_node, "sd-uhs-sdr104")) {
		reg_val = readl(host->ioaddr + 0x40);
		/* support 1.8V */
		reg_val |= BIT(26);
		/* write to sdhci140 or sdhci240 mirror register */
		writel(reg_val, dev->parent->regs + (0x10 * (slot + 1)));
	}

	if (of_property_read_bool(pdev->dev.parent->of_node, "sd-uhs-sdr104")) {
		reg_val = readl(host->ioaddr + 0x44);
		/* SDR104 */
		reg_val |= BIT(1);
		/* write to sdhci144 or sdhci244 mirror register */
		writel(reg_val, dev->parent->regs + (0x04 + (slot + 1) * 0x10));
	}

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pltfm_host->clk))
		return PTR_ERR(pltfm_host->clk);

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SDIO clock\n");
		goto err_pltfm_free;
	}

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	dev->pwr_pin = of_get_named_gpio(np, "power-gpio", 0);
	if(dev->pwr_pin >= 0) {
		if (gpio_is_valid(dev->pwr_pin)) {
			if (devm_gpio_request(&pdev->dev, dev->pwr_pin,
								  "mmc_pwr")) {
				dev_err(&pdev->dev, "devm_gpio_request pwr fail\n");
			}

			gpio_direction_output(dev->pwr_pin, 1);
		}
	}

	dev->pwr_sw_pin = of_get_named_gpio(np, "power-switch-gpio", 0);

	if(dev->pwr_sw_pin >= 0) {
		if (gpio_is_valid(dev->pwr_sw_pin)) {
			if (devm_gpio_request(&pdev->dev, dev->pwr_sw_pin,
							  "mmc_pwr_sw")) {
				dev_err(&pdev->dev, "devm_gpio_request pwr sw fail\n");
			}

			gpio_direction_output(dev->pwr_sw_pin, 1);
		}
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
err_pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int aspeed_sdhci_remove(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	int dead = 0;

	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);

	sdhci_remove_host(host, dead);

	clk_disable_unprepare(pltfm_host->clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id aspeed_sdhci_of_match[] = {
	{ .compatible = "aspeed,ast2400-sdhci", },
	{ .compatible = "aspeed,ast2500-sdhci", },
	{ .compatible = "aspeed,ast2600-sdhci", },
	{ }
};

static struct platform_driver aspeed_sdhci_driver = {
	.driver		= {
		.name	= "sdhci-aspeed",
		.of_match_table = aspeed_sdhci_of_match,
	},
	.probe		= aspeed_sdhci_probe,
	.remove		= aspeed_sdhci_remove,
};

static const struct of_device_id aspeed_sdc_of_match[] = {
	{ .compatible = "aspeed,ast2400-sd-controller", },
	{ .compatible = "aspeed,ast2500-sd-controller", },
	{ .compatible = "aspeed,ast2600-sd-controller", .data = &ast2600_sdc_info},
	{ }
};

MODULE_DEVICE_TABLE(of, aspeed_sdc_of_match);

static int aspeed_sdc_probe(struct platform_device *pdev)

{
	struct device_node *parent, *child;
	struct aspeed_sdc *sdc;
	const struct of_device_id *match = NULL;
	const struct aspeed_sdc_info *info = NULL;
	int ret;
	u32 timing_phase;

	sdc = devm_kzalloc(&pdev->dev, sizeof(*sdc), GFP_KERNEL);
	if (!sdc)
		return -ENOMEM;

	spin_lock_init(&sdc->lock);

	match = of_match_device(aspeed_sdc_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	if (match->data)
		info = match->data;

	if (info) {
		if (info->flag & PROBE_AFTER_ASSET_DEASSERT) {
			sdc->rst = devm_reset_control_get(&pdev->dev, NULL);
			if (!IS_ERR(sdc->rst)) {
				reset_control_assert(sdc->rst);
				reset_control_deassert(sdc->rst);
			}
		}
	}

	sdc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdc->clk))
		return PTR_ERR(sdc->clk);

	ret = clk_prepare_enable(sdc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SDCLK\n");
		return ret;
	}

	sdc->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sdc->regs = devm_ioremap_resource(&pdev->dev, sdc->res);
	if (IS_ERR(sdc->regs)) {
		ret = PTR_ERR(sdc->regs);
		goto err_clk;
	}

	if (!of_property_read_u32(pdev->dev.of_node, \
		"timing-phase", &timing_phase))
		writel(timing_phase, sdc->regs + TIMING_PHASE_OFFSET);

	dev_set_drvdata(&pdev->dev, sdc);

	parent = pdev->dev.of_node;
	for_each_available_child_of_node(parent, child) {
		struct platform_device *cpdev;

		cpdev = of_platform_device_create(child, NULL, &pdev->dev);
		if (!cpdev) {
			of_node_put(child);
			ret = -ENODEV;
			goto err_clk;
		}
	}

	return 0;

err_clk:
	clk_disable_unprepare(sdc->clk);
	return ret;
}

static int aspeed_sdc_remove(struct platform_device *pdev)
{
	struct aspeed_sdc *sdc = dev_get_drvdata(&pdev->dev);

	clk_disable_unprepare(sdc->clk);

	return 0;
}

static struct platform_driver aspeed_sdc_driver = {
	.driver		= {
		.name	= "sd-controller-aspeed",
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = aspeed_sdc_of_match,
	},
	.probe		= aspeed_sdc_probe,
	.remove		= aspeed_sdc_remove,
};

static int __init aspeed_sdc_init(void)
{
	int rc;

	rc = platform_driver_register(&aspeed_sdhci_driver);
	if (rc < 0)
		return rc;

	rc = platform_driver_register(&aspeed_sdc_driver);
	if (rc < 0)
		platform_driver_unregister(&aspeed_sdhci_driver);

	return rc;
}
module_init(aspeed_sdc_init);

static void __exit aspeed_sdc_exit(void)
{
	platform_driver_unregister(&aspeed_sdc_driver);
	platform_driver_unregister(&aspeed_sdhci_driver);
}
module_exit(aspeed_sdc_exit);

MODULE_DESCRIPTION("Driver for the ASPEED SD/SDIO/SDHCI Controllers");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_AUTHOR("Andrew Jeffery <andrew@aj.id.au>");
MODULE_LICENSE("GPL");
