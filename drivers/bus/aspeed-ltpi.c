// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright ASPEED Technology

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/io.h>

#define LTPI_AUTO_CAP_LOW			0x24
#define   LTPI_I2C_IO_FRAME_EN			GENMASK(29, 24)
#define LTPI_AUTO_CAP_HIGH			0x28
#define   LTPI_UART_IO_FRAME_EN			GENMASK(14, 13)

#define LTPI_LINK_CONTROLL			0x80
#define   LTPI_AUTO_CONFIG			BIT(10)

#define LTPI_INTR_STATUS			0x100
#define LTPI_INTR_EN				0x104
#define   LTPI_INTR_EN_OP_LINK_LOST		BIT(4)
#define LTPI_LINK_MANAGE_ST			0x108
#define   LTPI_LINK_PARTNER_FLAG		BIT(24)

#define LTPI_MANUAL_CAP_LOW			0x118
#define LTPI_MANUAL_CAP_HIGH			0x11c

#define LTPI_I2C_TIMING_0			0x134
#define LTPI_I2C_TIMING_1			0x138

#define LTPI_I2C_100K_0			0x3535352f
#define LTPI_I2C_100K_1			0x09353535

#define LTPI_I2C_400K_0			0x06060d06
#define LTPI_I2C_400K_1			0x090d0a06

#define SCU_IO_PINS_TRAP1			0x10
#define SCU_IO_PINS_TRAP1_CLEAR			0x14
#define   SCU_IO_PINS_TRAP_LTPI			GENMASK(2, 0)
#define SCU_IO_OTP_TRAP1			0xa00
#define SCU_IO_OTP_TRAP1_CLEAR			0xa04
#define SCU_IO_OTP_TRAP2			0xa20
#define SCU_IO_OTP_TRAP2_CLEAR			0xa24

#define MAX_I2C_IN_LTPI				6
#define MAX_UART_IN_LTPI			2

enum chip_version {
	AST2700,
	AST1700,
};

struct aspeed_ltpi_priv {
	struct device *dev;
	void __iomem *regs;
	struct clk *ltpi_clk;
	struct clk *ltpi_phyclk;
	struct reset_control *ltpi_rst;
	struct regmap *scu;
	u32 version;
	u32 i2c_tunneling;
	u32 i2c_timing_0;
	u32 i2c_timing_1;
	u32 uart_tunneling;
};

static irqreturn_t aspeed_ltpi_irq_handler(int irq, void *dev_id)
{
	struct aspeed_ltpi_priv *priv = dev_id;
	u32 status = readl(priv->regs + LTPI_INTR_STATUS);

	if (status & LTPI_INTR_EN_OP_LINK_LOST) {
		writel(0, priv->regs + LTPI_INTR_EN);
		writel(status, priv->regs + LTPI_INTR_STATUS);
		panic("LTPI link lost!\n");
		/* Will not return */
	}

	writel(status, priv->regs + LTPI_INTR_STATUS);

	return IRQ_HANDLED;
}

static int aspeed_ltpi_init_mux(struct aspeed_ltpi_priv *priv)
{
	u32 reg, i2c_en, uart_en, i;

	reg = readl(priv->regs + LTPI_AUTO_CAP_LOW);

	i2c_en = FIELD_GET(LTPI_I2C_IO_FRAME_EN, reg);
	i2c_en &= priv->i2c_tunneling;

	reg &= ~LTPI_I2C_IO_FRAME_EN;
	reg |= FIELD_PREP(LTPI_I2C_IO_FRAME_EN, i2c_en);
	writel(reg, priv->regs + LTPI_MANUAL_CAP_LOW);

	reg = readl(priv->regs + LTPI_AUTO_CAP_HIGH);

	uart_en = FIELD_GET(LTPI_UART_IO_FRAME_EN, reg);
	uart_en &= priv->uart_tunneling;

	reg &= ~LTPI_UART_IO_FRAME_EN;
	reg |= FIELD_PREP(LTPI_UART_IO_FRAME_EN, uart_en);

	writel(reg, priv->regs + LTPI_MANUAL_CAP_HIGH);

	/* Apply LTPI manual configuration */
	reg = readl(priv->regs + LTPI_LINK_CONTROLL);
	reg &= ~LTPI_AUTO_CONFIG;
	writel(reg, priv->regs + LTPI_LINK_CONTROLL);

	/* Set the AST1700 i2c ac-timing */
	if (priv->version == AST1700) {
		/* Apply i2c timing with i2c tunneling setting */
		for (i = 0; i < MAX_I2C_IN_LTPI; i++) {
			if ((priv->i2c_tunneling >> i) & 0x1) {
				writel(priv->i2c_timing_0,
				       priv->regs + LTPI_I2C_TIMING_0 + (0x8 * i));
				writel(priv->i2c_timing_1,
				       priv->regs + LTPI_I2C_TIMING_1 + (0x8 * i));
			}
		}
	}

	return 0;
}

static int aspeed_ltpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_dev_auxdata *lookup = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	struct aspeed_ltpi_priv *priv;
	int irq, ret;

	match = of_match_device(dev->driver->of_match_table, dev);

	if (match) {
		if (of_property_match_string(np, "compatible", match->compatible) < 0)
			return -ENODEV;
	} else {
		return -ENODEV;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->version = (enum chip_version)device_get_match_data(dev);

	priv->ltpi_clk = devm_clk_get(&pdev->dev, "ltpi");
	if (IS_ERR(priv->ltpi_clk)) {
		priv->ltpi_clk = devm_clk_get(&pdev->dev, "ahb");
		if (IS_ERR(priv->ltpi_clk))
			return PTR_ERR(priv->ltpi_clk);

		clk_prepare_enable(priv->ltpi_clk);

		priv->ltpi_phyclk = devm_clk_get(&pdev->dev, "phy");
		if (IS_ERR(priv->ltpi_phyclk))
			return PTR_ERR(priv->ltpi_phyclk);

		clk_prepare_enable(priv->ltpi_phyclk);
	} else {
		priv->ltpi_phyclk = NULL;
	}

	priv->ltpi_rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(priv->ltpi_rst))
		return PTR_ERR(priv->ltpi_rst);

	reset_control_deassert(priv->ltpi_rst);

	priv->i2c_tunneling = GENMASK(MAX_I2C_IN_LTPI - 1, 0);
	if (!of_property_read_u32(np, "i2c-tunneling", &ret))
		priv->i2c_tunneling = ret;

	priv->i2c_timing_0 = LTPI_I2C_100K_0;
	priv->i2c_timing_1 = LTPI_I2C_100K_1;
	if (!of_property_read_u32(np, "i2c-tunneling-timing", &ret)) {
		if (ret == 400) {
			priv->i2c_timing_0 = LTPI_I2C_400K_0;
			priv->i2c_timing_1 = LTPI_I2C_400K_1;
		}
	}
	priv->uart_tunneling = GENMASK(MAX_UART_IN_LTPI - 1, 0);
	if (!of_property_read_u32(np, "uart-tunneling", &ret))
		priv->uart_tunneling = ret;

	priv->scu = syscon_regmap_lookup_by_phandle(np, "aspeed,scu");
	if (of_get_property(np, "remote-controller", NULL)) {
		u32 reg;

		/* Clear all the pins/otp strap but LTPI related settings for AST1700 */
		regmap_read(priv->scu, SCU_IO_PINS_TRAP1, &reg);
		reg &= ~SCU_IO_PINS_TRAP_LTPI;
		regmap_write(priv->scu, SCU_IO_PINS_TRAP1_CLEAR, reg);

		regmap_read(priv->scu, SCU_IO_OTP_TRAP1, &reg);
		regmap_write(priv->scu, SCU_IO_OTP_TRAP1_CLEAR, reg);

		regmap_read(priv->scu, SCU_IO_OTP_TRAP2, &reg);
		regmap_write(priv->scu, SCU_IO_OTP_TRAP2_CLEAR, reg);
	} else {
		irq = platform_get_irq(pdev, 0);
		ret = devm_request_irq(priv->dev, irq, aspeed_ltpi_irq_handler,
				       0, dev_name(priv->dev), priv);
		if (ret) {
			dev_err(priv->dev, "failed to request irq\n");
			reset_control_assert(priv->ltpi_rst);
			clk_disable_unprepare(priv->ltpi_phyclk);
			clk_disable_unprepare(priv->ltpi_clk);
			return ret;
		}

		writel(LTPI_INTR_EN_OP_LINK_LOST, priv->regs + LTPI_INTR_STATUS);
		writel(LTPI_INTR_EN_OP_LINK_LOST, priv->regs + LTPI_INTR_EN);
	}

	aspeed_ltpi_init_mux(priv);

	platform_set_drvdata(pdev, priv);
	if (np)
		of_platform_populate(np, NULL, lookup, priv->dev);

	return 0;
}

static int aspeed_ltpi_remove(struct platform_device *pdev)
{
	struct aspeed_ltpi_priv *priv;

	priv = platform_get_drvdata(pdev);
	reset_control_assert(priv->ltpi_rst);
	clk_disable_unprepare(priv->ltpi_phyclk);
	clk_disable_unprepare(priv->ltpi_clk);

	return 0;
}

static const struct of_device_id aspeed_ltpi_of_match[] = {
	{ .compatible = "aspeed-ltpi", .data = (const void *)AST2700,},
	{ .compatible = "aspeed-ast1700-ltpi", .data = (const void *)AST1700,},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aspeed_ltpi_of_match);

static struct platform_driver aspeed_ltpi_driver = {
	.probe = aspeed_ltpi_probe,
	.remove = aspeed_ltpi_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_ltpi_of_match,
	},
};

module_platform_driver(aspeed_ltpi_driver);

MODULE_DESCRIPTION("LVDS Tunneling Protocol and Interface Bus Driver");
MODULE_AUTHOR("Dylan Hung <dylan_hung@aspeedtech.com>");
MODULE_LICENSE("GPL");
