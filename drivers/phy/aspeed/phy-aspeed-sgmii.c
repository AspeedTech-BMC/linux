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

#define SCU_HW_REVISION_ID	GENMASK(23, 16)

#define SGMII_CFG			0x00
#define   SGMII_CFG_FIFO_MODE			BIT(0)
#define   SGMII_CFG_SPEED_SEL_MASK		GENMASK(5, 4)
#define   SGMII_CFG_SPEED_SEL(x)		FIELD_PREP(SGMII_CFG_SPEED_SEL_MASK, (x))
#define   SGMII_CFG_PWR_DOWN			BIT(11)
#define   SGMII_CFG_AN_ENABLE			BIT(12)
#define  SGMII_CFG_SW_RESET			BIT(15)
#define SGMII_LINK_TIMER		0x08
#define SGMII_NWAY_ACK			0x0c
#define SGMII_PHY_CFG1			0x18
#define   SGMII_PHY_SPEED_MASK			GENMASK(3, 2)
#define   SGMII_PHY_SPEED(x)			FIELD_PREP(SGMII_PHY_SPEED_MASK, (x))
#define SGMII_PHY_PIPE_CTL		0x20
#define   SGMII_PCTL_TX_NO_DEEMPH		BIT(7)
#define SGMII_FIFO_DELAY_THREHOLD	0x28
#define SGMII_MODE			0x30
#define   SGMII_MODE_ENABLE			BIT(0)
#define   SGMII_MODE_USE_LOCAL_CONFIG		BIT(2)

#define PEHR280				0x280
#define   SGMII_INTERNAL_CLK_EN			BIT(26)
#define PCIEPHY_CLK			0x268
#define   PCIEPHY_CLK_FREQ_MULTI_MASK		GENMASK(7, 0)
#define   PCIEPHY_CLK_FREQ_MULTI(x)		FIELD_PREP(PCIEPHY_CLK_FREQ_MULTI_MASK, (x))
#define   PCIEPHY_CLK_SEL_INTERNAL_25M		BIT(8)

#define SGMII_SPEED_10M		0x00
#define SGMII_SPEED_100M	0x01
#define SGMII_SPEED_1G		0x02

struct aspeed_sgmii {
	struct device *dev;
	void __iomem *regs;
	struct regmap *pcie_phy_regmap;
	u8 revision;
};

static int aspeed_sgmii_conf(struct phy *phy, bool nway, int speed)
{
	struct aspeed_sgmii *sgmii = phy_get_drvdata(phy);
	u32 cfg;

	writel(0, sgmii->regs + SGMII_MODE);

	writel(0, sgmii->regs + SGMII_CFG);
	writel(SGMII_CFG_SW_RESET | SGMII_CFG_PWR_DOWN, sgmii->regs + SGMII_CFG);
	if (nway) {
		if (sgmii->revision == 1)
			writel(SGMII_CFG_AN_ENABLE, sgmii->regs + SGMII_CFG);
		else
			writel(SGMII_CFG_AN_ENABLE | SGMII_CFG_FIFO_MODE,
			       sgmii->regs + SGMII_CFG);
	} else {
		switch (speed) {
		case SPEED_10:
			cfg = SGMII_SPEED_10M;
			break;
		case SPEED_100:
			cfg = SGMII_SPEED_100M;
			break;
		case SPEED_1000:
			cfg = SGMII_SPEED_1G;
			break;
		default:
			return -EINVAL;
		}
		writel(SGMII_PHY_SPEED(cfg), sgmii->regs + SGMII_PHY_CFG1);
		if (sgmii->revision == 1)
			writel(SGMII_CFG_SPEED_SEL(cfg),
			       sgmii->regs + SGMII_CFG);
		else
			writel(SGMII_CFG_SPEED_SEL(cfg) | SGMII_CFG_FIFO_MODE,
			       sgmii->regs + SGMII_CFG);
	}

	if (sgmii->revision == 1)
		writel(0x0c, sgmii->regs + SGMII_FIFO_DELAY_THREHOLD);
	else
		writel(0x0e, sgmii->regs + SGMII_FIFO_DELAY_THREHOLD);
	writel(SGMII_PCTL_TX_NO_DEEMPH, sgmii->regs + SGMII_PHY_PIPE_CTL);

	/* Set link timer for state change */
	writel(0x100, sgmii->regs + SGMII_LINK_TIMER);

	/* Bit 0 always sets to 1 in ACK message */
	writel(0x1, sgmii->regs + SGMII_NWAY_ACK);

	cfg = SGMII_MODE_ENABLE;
	if (!nway)
		cfg |= SGMII_MODE_USE_LOCAL_CONFIG;
	writel(cfg, sgmii->regs + SGMII_MODE);

	return 0;
}

static int aspeed_sgmii_phy_init(struct phy *phy)
{
	/* Default to enable Nway, not need configure speed */
	return aspeed_sgmii_conf(phy, true, 0);
}

static int aspeed_sgmii_phy_set_speed(struct phy *phy, int speed)
{
	return aspeed_sgmii_conf(phy, false, speed);
}

static int aspeed_sgmii_phy_exit(struct phy *phy)
{
	struct aspeed_sgmii *sgmii = phy_get_drvdata(phy);

	/* Disable SGMII controller */
	writel(0, sgmii->regs + SGMII_MODE);

	return 0;
}

static const struct phy_ops aspeed_sgmii_phyops = {
	.init		= aspeed_sgmii_phy_init,
	.set_speed	= aspeed_sgmii_phy_set_speed,
	.exit		= aspeed_sgmii_phy_exit,
	.owner		= THIS_MODULE,
};

static int aspeed_sgmii_probe(struct platform_device *pdev)
{
	struct phy_provider *provider;
	struct aspeed_sgmii *sgmii;
	struct regmap *scu_regmap;
	struct device_node *np;
	struct resource *res;
	struct device *dev;
	struct phy *phy;
	u32 reg;

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
	sgmii->pcie_phy_regmap = syscon_regmap_lookup_by_phandle(np, "phys");
	if (IS_ERR(sgmii->pcie_phy_regmap)) {
		dev_err(sgmii->dev, "Unable to find phys regmap (%ld)\n",
			PTR_ERR(sgmii->pcie_phy_regmap));
		return PTR_ERR(sgmii->pcie_phy_regmap);
	}

	scu_regmap = syscon_regmap_lookup_by_phandle(np, "aspeed,scu");
	if (IS_ERR(scu_regmap)) {
		dev_err(sgmii->dev, "Unable to find SCU regmap (%ld)\n",
			PTR_ERR(scu_regmap));
		return PTR_ERR(scu_regmap);
	}

	regmap_read(scu_regmap, 0x00, &reg);
	sgmii->revision = FIELD_GET(SCU_HW_REVISION_ID, reg);

	phy = devm_phy_create(dev, NULL, &aspeed_sgmii_phyops);
	if (IS_ERR(phy)) {
		dev_err(&pdev->dev, "failed to create PHY\n");
		return PTR_ERR(phy);
	}

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	phy_set_drvdata(phy, sgmii);

	/*
	 * The PLDA frequency multiplication is X xor 0x19.
	 * (X xor 0x19) * clock source = data rate.
	 * SGMII data rate is 1.25G, so (0x2b xor 0x19) * 25MHz is equal 1.25G.
	 */
	reg = PCIEPHY_CLK_SEL_INTERNAL_25M | PCIEPHY_CLK_FREQ_MULTI(0x2b);
	regmap_write(sgmii->pcie_phy_regmap, PCIEPHY_CLK, reg);
	if (sgmii->revision > 1) {
		regmap_read(sgmii->pcie_phy_regmap, PEHR280, &reg);
		reg |= SGMII_INTERNAL_CLK_EN;
		regmap_write(sgmii->pcie_phy_regmap, PEHR280, reg);
	}

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
MODULE_DESCRIPTION("ASPEED SGMII Serdes/PHY controller");
MODULE_LICENSE("GPL");
