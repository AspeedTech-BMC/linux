// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>

#define DEVICE_NAME	"aspeed-disp-intf"

#define AST2700_SCU_CHIP_ID		0x0
#define  SCU_CPU_REVISION_ID_HW		GENMASK(23, 16)

#define AST2700_SCU_PIN_SEL		0x414
#define  AST2700_SCU_D1PLL_SEL		GENMASK(13, 12)
#define  AST2700_SCU_DAC_SRC_SEL	GENMASK(11, 10)
#define  AST2700_SCU_DP_SRC_SEL		GENMASK(9, 8)

#define AST2600_SCU_PIN_SEL		0x0C0
#define  AST2600_SCU_DP_SRC_SEL		BIT(18)
#define  AST2600_SCU_DAC_SRC_SEL	BIT(16)

struct aspeed_disp_intf_config {
	u8 version;
	u32 dac_src_sel;
	u32 dac_src_max;
	u32 dac_src_min;
	u32 dp_src_sel;
	u32 dp_src_max;
	u32 dp_src_min;
};

struct aspeed_disp_intf {
	struct device *dev;
	struct miscdevice miscdev;
	struct regmap *scu;
	const struct aspeed_disp_intf_config *config;
};

static int dac_src, dp_src;

static const struct aspeed_disp_intf_config ast2600_config = {
	.version = 6,
	.dac_src_sel = AST2600_SCU_PIN_SEL,
	.dac_src_max = 1,
	.dac_src_min = 0,
	.dp_src_sel = AST2600_SCU_PIN_SEL,
	.dp_src_max = 1,
	.dp_src_min = 0,
};

static const struct aspeed_disp_intf_config ast2700_config = {
	.version = 7,
	.dac_src_sel = AST2700_SCU_PIN_SEL,
	.dac_src_max = 2,
	.dac_src_min = 0,
	.dp_src_sel = AST2700_SCU_PIN_SEL,
	.dp_src_max = 2,
	.dp_src_min = 0,
};

static ssize_t dac_src_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct aspeed_disp_intf *intf = dev_get_drvdata(dev);
	const struct aspeed_disp_intf_config *config = intf->config;
	u32 val;

	regmap_read(intf->scu, config->dac_src_sel, &val);
	dac_src = (config->version == 6)
		? FIELD_GET(AST2600_SCU_DAC_SRC_SEL, val)
		: FIELD_GET(AST2700_SCU_DAC_SRC_SEL, val);
	return sysfs_emit(buf, "%d\n", dac_src);
}

static ssize_t dac_src_store(struct device *dev,
			     struct device_attribute *attr, const char *buf, size_t count)
{
	struct aspeed_disp_intf *intf = dev_get_drvdata(dev);
	const struct aspeed_disp_intf_config *config = intf->config;
	int src, res;

	res = kstrtoint(buf, 0, &src);
	if (res)
		return res;

	if (src < config->dac_src_min || src > config->dac_src_max) {
		dev_err(intf->dev, "Invalid dac_src(max:%d, min:%d)\n",
			config->dac_src_max, config->dac_src_min);
		return -1;
	}

	dac_src = src;
	if (config->version == 6)
		regmap_update_bits(intf->scu, config->dac_src_sel, AST2600_SCU_DAC_SRC_SEL,
				   FIELD_PREP(AST2600_SCU_DAC_SRC_SEL, src));
	else
		regmap_update_bits(intf->scu, config->dac_src_sel, AST2700_SCU_DAC_SRC_SEL,
				   FIELD_PREP(AST2700_SCU_DAC_SRC_SEL, src));

	return count;
}

static DEVICE_ATTR_RW(dac_src);

static ssize_t dp_src_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct aspeed_disp_intf *intf = dev_get_drvdata(dev);
	const struct aspeed_disp_intf_config *config = intf->config;
	u32 val;

	regmap_read(intf->scu, config->dp_src_sel, &val);
	dp_src = (config->version == 6)
		? FIELD_GET(AST2600_SCU_DP_SRC_SEL, val)
		: FIELD_GET(AST2700_SCU_DP_SRC_SEL, val);
	return sysfs_emit(buf, "%d\n", dp_src);
}

static ssize_t dp_src_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct aspeed_disp_intf *intf = dev_get_drvdata(dev);
	const struct aspeed_disp_intf_config *config = intf->config;
	int src, res;

	res = kstrtoint(buf, 0, &src);
	if (res)
		return res;

	if (src < config->dp_src_min || src > config->dp_src_max) {
		dev_err(intf->dev, "Invalid dp_src(max:%d, min:%d)\n",
			config->dp_src_max, config->dp_src_min);
		return -1;
	}

	dp_src = src;
	if (config->version == 6) {
		regmap_update_bits(intf->scu, config->dp_src_sel, AST2600_SCU_DP_SRC_SEL,
				   FIELD_PREP(AST2600_SCU_DP_SRC_SEL, src));
	} else {
		u32 val;

		regmap_update_bits(intf->scu, config->dp_src_sel, AST2700_SCU_DP_SRC_SEL,
				   FIELD_PREP(AST2700_SCU_DP_SRC_SEL, src));

		// D1PLL used in A0 only
		regmap_read(intf->scu, AST2700_SCU_CHIP_ID, &val);
		if (FIELD_GET(SCU_CPU_REVISION_ID_HW, val) == 0) {
			regmap_update_bits(intf->scu, config->dp_src_sel, AST2700_SCU_D1PLL_SEL,
					   FIELD_PREP(AST2700_SCU_D1PLL_SEL, src));
		}
	}

	return count;
}

static DEVICE_ATTR_RW(dp_src);

static struct attribute *aspeed_disp_intf_attrs[] = {
	&dev_attr_dac_src.attr,
	&dev_attr_dp_src.attr,
	NULL,
};

static const struct attribute_group aspeed_disp_intf_attgrp = {
	.name = NULL,
	.attrs = aspeed_disp_intf_attrs,
};

static int aspeed_disp_intf_probe(struct platform_device *pdev)
{
	struct aspeed_disp_intf *intf;
	struct device *dev = &pdev->dev;
	int ret;

	intf = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_disp_intf), GFP_KERNEL);
	if (!intf)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, intf);

	intf->config = of_device_get_match_data(&pdev->dev);
	if (!intf->config)
		return -ENODEV;

	intf->dev = dev;
	intf->scu = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(intf->scu)) {
		dev_err(dev, "failed to find SCU regmap\n");
		return PTR_ERR(intf->scu);
	}

	intf->miscdev.minor = MISC_DYNAMIC_MINOR;
	intf->miscdev.name = DEVICE_NAME;
	intf->miscdev.parent = dev;
	ret = misc_register(&intf->miscdev);
	if (ret) {
		dev_err(dev, "Unable to register device\n");
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &aspeed_disp_intf_attgrp);
	if (ret != 0)
		dev_warn(dev, "failed to register attributes\n");

	return 0;
}

static void aspeed_disp_intf_remove(struct platform_device *pdev)
{
	struct aspeed_disp_intf *intf = platform_get_drvdata(pdev);

	sysfs_remove_group(&intf->dev->kobj, &aspeed_disp_intf_attgrp);
	misc_deregister(&intf->miscdev);
	devm_kfree(&pdev->dev, intf);
}

static const struct of_device_id aspeed_disp_intf_of_matches[] = {
	{ .compatible = "aspeed,ast2600-disp-intf", .data = &ast2600_config },
	{ .compatible = "aspeed,ast2700-disp-intf", .data = &ast2700_config },
	{},
};

static struct platform_driver aspeed_disp_intf_driver = {
	.probe		= aspeed_disp_intf_probe,
	.remove_new	= aspeed_disp_intf_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.of_match_table = aspeed_disp_intf_of_matches,
	},
};

module_platform_driver(aspeed_disp_intf_driver);

MODULE_DEVICE_TABLE(of, aspeed_disp_intf_of_matches);
MODULE_AUTHOR("Jammy Huang <jammy_huang@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Display Interface Driver");
MODULE_LICENSE("GPL");
