// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Aspeed Technology Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/reset.h>

#define PHY3P00_DEFAULT		0xCE70000F	/* PHY PCS Protocol Setting #1 default value */
#define PHY3P04_DEFAULT		0x49C00014	/* PHY PCS Protocol Setting #2 default value */
#define PHY3P08_DEFAULT		0x5E406825	/* PHY PCS Protocol Setting #3 default value */
#define PHY3P0C_DEFAULT		0x00000001	/* PHY PCS Protocol Setting #4 default value */

#define DWC_CRTL_NUM	2

#define USB_PHY3_INIT_DONE	BIT(15)	/* BIT15: USB3.1 Phy internal SRAM iniitalization done */
#define USB_PHY3_SRAM_BYPASS	BIT(7)	/* USB3.1 Phy SRAM bypass */
#define USB_PHY3_SRAM_EXT_LOAD	BIT(6)	/* USB3.1 Phy SRAM external load done */

struct usb_dwc3_ctrl {
	u32 offset;
	u32 value;
};

struct aspeed_usb_phy3_model {
	/* offsets to the PHY3 registers */
	unsigned int phy3s00;	/* PHY SRAM Control/Status #1 */
	unsigned int phy3s04;	/* PHY SRAM Control/Status #2 */
	unsigned int phy3c00;	/* PHY PCS Control/Status #1 */
	unsigned int phy3c04;	/* PHY PCS Control/Status #2 */
	unsigned int phy3p00;	/* PHY PCS Protocol Setting #1 */
	unsigned int phy3p04;	/* PHY PCS Protocol Setting #2 */
	unsigned int phy3p08;	/* PHY PCS Protocol Setting #3 */
	unsigned int phy3p0c;	/* PHY PCS Protocol Setting #4 */
	unsigned int dwc_cmd;	/* DWC3 Commands base address offest */
};

static struct usb_dwc3_ctrl ctrl_data[DWC_CRTL_NUM] = {
	{0xc12c, 0x0c854802},	/* Set DWC3 GUCTL for ref_clk */
	{0xc630, 0x0c800020},	/* Set DWC3 GLADJ for ref_clk */
};

static const struct aspeed_usb_phy3_model ast2700a0_model = {
	.phy3s00 = 0x800,
	.phy3s04 = 0x804,
	.phy3c00 = 0x808,
	.phy3c04 = 0x80C,
	.phy3p00 = 0x810,
	.phy3p04 = 0x814,
	.phy3p08 = 0x818,
	.phy3p0c = 0x81C,
	.dwc_cmd = 0xB80,
};

static const struct aspeed_usb_phy3_model ast2700_model = {
	.phy3s00 = 0x00,
	.phy3s04 = 0x04,
	.phy3c00 = 0x08,
	.phy3c04 = 0x0C,
	.phy3p00 = 0x10,
	.phy3p04 = 0x14,
	.phy3p08 = 0x18,
	.phy3p0c = 0x1C,
	.dwc_cmd = 0x40,
};

static const struct of_device_id aspeed_usb_phy3_dt_ids[] = {
	{
		.compatible = "aspeed,ast2700-a0-phy3a",
		.data = &ast2700a0_model
	},
	{
		.compatible = "aspeed,ast2700-a0-phy3b",
		.data = &ast2700a0_model
	},
	{
		.compatible = "aspeed,ast2700-phy3a",
		.data = &ast2700_model
	},
	{
		.compatible = "aspeed,ast2700-phy3b",
		.data = &ast2700_model
	},
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_usb_phy3_dt_ids);

static int aspeed_usb_phy3_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct aspeed_usb_phy3_model *model;
	void __iomem *reg_base;
	u32 val;
	bool phy_ext_load_quirk;
	struct clk				*clk;
	struct reset_control	*rst;
	int timeout = 100;
	int rc = 0;
	int i, j;

	model = of_device_get_match_data(&pdev->dev);
	if (IS_ERR(model)) {
		dev_err(&pdev->dev, "Couldn't get model data\n");
		return -ENODEV;
	}

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rc = clk_prepare_enable(clk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to enable clock (%d)\n", rc);
		return rc;
	}

	rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		rc = PTR_ERR(rst);
		goto err;
	}
	rc = reset_control_deassert(rst);
	if (rc)
		goto err;

	reg_base = of_iomap(node, 0);

	while ((readl(reg_base + model->phy3s00) & USB_PHY3_INIT_DONE)
			!= USB_PHY3_INIT_DONE) {
		usleep_range(100, 110);
		if (--timeout == 0) {
			dev_err(&pdev->dev, "Wait phy3 init timed out\n");
			rc = -ETIMEDOUT;
			goto err;
		}
	}

	phy_ext_load_quirk =
		device_property_read_bool(&pdev->dev, "aspeed,phy_ext_load_quirk");

	val = readl(reg_base + model->phy3s00);

	if (phy_ext_load_quirk)
		val |= USB_PHY3_SRAM_EXT_LOAD;
	else
		val |= USB_PHY3_SRAM_BYPASS;
	writel(val, reg_base + model->phy3s00);

	/* Set protocol1_ext signals as default PHY3 settings based on SNPS documents.
	 * Including PCFGI[54]: protocol1_ext_rx_los_lfps_en for better compatibility
	 */
	writel(PHY3P00_DEFAULT, reg_base + model->phy3p00);
	writel(PHY3P04_DEFAULT, reg_base + model->phy3p04);
	writel(PHY3P08_DEFAULT, reg_base + model->phy3p08);
	writel(PHY3P0C_DEFAULT, reg_base + model->phy3p0c);

	/* xHCI DWC specific command initially set when PCIe xHCI enable */
	for (i = 0, j = model->dwc_cmd; i < DWC_CRTL_NUM; i++) {
		/* 48-bits Command:
		 * CMD1: Data -> DWC CMD [31:0], Address -> DWC CMD [47:32]
		 * CMD2: Data -> DWC CMD [79:48], Address -> DWC CMD [95:80]
		 * ... and etc.
		 */
		if (i % 2 == 0) {
			writel(ctrl_data[i].value, reg_base + j);
			j += 4;

			writel(ctrl_data[i].offset & 0xFFFF, reg_base + j);
		} else {
			val = readl(reg_base + j) & 0xFFFF;
			val |= ((ctrl_data[i].value & 0xFFFF) << 16);
			writel(val, reg_base + j);
			j += 4;

			val = (ctrl_data[i].offset << 16) | (ctrl_data[i].value >> 16);
			writel(val, reg_base + j);
			j += 4;
		}
	}

	dev_info(&pdev->dev, "Initialized USB PHY3\n");

	return 0;

err:
	if (clk)
		clk_disable_unprepare(clk);
	return rc;
}

static int aspeed_usb_phy3_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver aspeed_usb_phy3_driver = {
	.probe		= aspeed_usb_phy3_probe,
	.remove		= aspeed_usb_phy3_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table	= aspeed_usb_phy3_dt_ids,
	},
};
module_platform_driver(aspeed_usb_phy3_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joe Wang <joe_wang@aspeedtech.com>");
