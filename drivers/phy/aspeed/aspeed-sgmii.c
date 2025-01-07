// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Aspeed Technology Inc.
 */
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/ethtool.h>

#define SGMII_CFG		0x00
#define SGMII_PHY_PIPE_CTL	0x20
#define SGMII_MODE		0x30

#define SGMII_CFG_FIFO_MODE		BIT(0)
#define SGMII_CFG_SPEED_10M		0
#define SGMII_CFG_SPEED_100M		BIT(4)
#define SGMII_CFG_SPEED_1G		BIT(5)
#define SGMII_CFG_PWR_DOWN		BIT(11)
#define SGMII_CFG_AN_ENABLE		BIT(12)
#define SGMII_CFG_SW_RESET		BIT(15)
#define SGMII_PCTL_TX_DEEMPH_3_5DB	BIT(6)
#define SGMII_MODE_ENABLE		BIT(0)
#define SGMII_MODE_USE_LOCAL_CONFIG	BIT(2)

#define PLDA_CLK		0x268

#define PLDA_CLK_SEL_INTERNAL_25M	BIT(8)
#define PLDA_CLK_FREQ_MULTI		GENMASK(7, 0)

struct aspeed_sgmii {
	struct device *dev;
	void __iomem *regs;
	struct regmap *plda_regmap;
};

static void aspeed_sgmii_set_nway(struct phy *phy)
{
	struct aspeed_sgmii *sgmii = phy_get_drvdata(phy);
	u32 reg;

	/*
	 * The PLDA frequency multiplication is X xor 0x19.
	 * (X xor 0x19) * clock source = data rate.
	 * SGMII data rate is 1.25G, so (0x2b xor 0x19) * 25MHz is equal 1.25G.
	 */
	reg = PLDA_CLK_SEL_INTERNAL_25M | FIELD_PREP(PLDA_CLK_FREQ_MULTI, 0x2b);
	regmap_write(sgmii->plda_regmap, PLDA_CLK, reg);

	writel(0, sgmii->regs + SGMII_MODE);

	writel(0, sgmii->regs + SGMII_CFG);
	reg = SGMII_CFG_SW_RESET | SGMII_CFG_PWR_DOWN;
	writel(reg, sgmii->regs + SGMII_CFG);

	reg = SGMII_CFG_AN_ENABLE;
	writel(reg, sgmii->regs + SGMII_CFG);

	writel(SGMII_PCTL_TX_DEEMPH_3_5DB, sgmii->regs + SGMII_PHY_PIPE_CTL);
	reg = SGMII_MODE_ENABLE;
	writel(reg, sgmii->regs + SGMII_MODE);
}

static void aspeed_sgmii_set_2_5g(struct phy *phy)
{
	struct aspeed_sgmii *sgmii = phy_get_drvdata(phy);
	u32 reg;

	/* For HiSGMII 2.5G speed */
	reg = PLDA_CLK_SEL_INTERNAL_25M | FIELD_PREP(PLDA_CLK_FREQ_MULTI, 0x64);
	regmap_write(sgmii->plda_regmap, PLDA_CLK, reg);

	writel(0, sgmii->regs + SGMII_MODE);

	writel(0, sgmii->regs + SGMII_CFG);
	reg = SGMII_CFG_SW_RESET | SGMII_CFG_PWR_DOWN;
	writel(reg, sgmii->regs + SGMII_CFG);

	reg = SGMII_CFG_SPEED_1G;
	writel(reg, sgmii->regs + SGMII_CFG);

	writel(SGMII_PCTL_TX_DEEMPH_3_5DB, sgmii->regs + SGMII_PHY_PIPE_CTL);
	reg = SGMII_MODE_ENABLE;
	writel(reg, sgmii->regs + SGMII_MODE);
}

static int aspeed_sgmii_phy_init(struct phy *phy)
{
	aspeed_sgmii_set_nway(phy);

	return 0;
}

static int aspeed_sgmii_phy_exit(struct phy *phy)
{
	struct aspeed_sgmii *sgmii = phy_get_drvdata(phy);

	/* Disable SGMII controller */
	writel(0, sgmii->regs + SGMII_MODE);

	return 0;
}

static int aspeed_sgmii_phy_set_speed(struct phy *phy, int speed)
{
	if (speed == SPEED_2500)
		aspeed_sgmii_set_2_5g(phy);
	else
		aspeed_sgmii_set_nway(phy);

	return 0;
}

static const struct phy_ops aspeed_sgmii_phyops = {
	.init		= aspeed_sgmii_phy_init,
	.exit		= aspeed_sgmii_phy_exit,
	.set_speed	= aspeed_sgmii_phy_set_speed,
	.owner		= THIS_MODULE,
};

static int aspeed_sgmii_probe(struct platform_device *pdev)
{
	struct aspeed_sgmii *sgmii;
	struct resource *res;
	struct device *dev;
	struct device_node *np;
	struct phy_provider *provider;
	struct phy *phy;

	dev = &pdev->dev;

	sgmii = devm_kzalloc(dev, sizeof(*sgmii), GFP_KERNEL);
	if (!sgmii)
		return -ENOMEM;

	sgmii->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot get resource\n");
		return -ENODEV;
	}

	sgmii->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(sgmii->regs)) {
		dev_err(dev, "cannot map registers\n");
		return PTR_ERR(sgmii->regs);
	}

	np = pdev->dev.of_node;
	sgmii->plda_regmap = syscon_regmap_lookup_by_phandle(np, "aspeed,plda");
	if (IS_ERR(sgmii->plda_regmap)) {
		dev_err(sgmii->dev, "Unable to find plda regmap (%ld)\n",
			PTR_ERR(sgmii->plda_regmap));
		return PTR_ERR(sgmii->plda_regmap);
	}

	phy = devm_phy_create(dev, NULL, &aspeed_sgmii_phyops);
	if (IS_ERR(phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	phy_set_drvdata(phy, sgmii);

	dev_info(dev, "module loaded\n");

	return 0;
}

static const struct of_device_id aspeed_sgmii_of_matches[] = {
	{ .compatible = "aspeed,ast2700-sgmii" },
	{ },
};

static struct platform_driver aspeed_sgmii_driver = {
	.probe = aspeed_sgmii_probe,
	.driver = {
		.name = "aspeed-sgmii",
		.of_match_table = aspeed_sgmii_of_matches,
	},
};

module_platform_driver(aspeed_sgmii_driver);

MODULE_AUTHOR("Jacky Chou <jacky_chou@aspeedtech.com>");
MODULE_DESCRIPTION("Control of ASPEED SGMII Device");
MODULE_LICENSE("GPL");
