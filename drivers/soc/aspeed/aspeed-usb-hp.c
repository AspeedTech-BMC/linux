// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Aspeed Technology Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>

#define	USB_HP_BEHCI84		0x84	/* Controller Fine-tune Register */
#define	USB_HP_BEHCI88		0x88	/* Frame Timing Adjustment */

#define	BEHCI_64BIT		BIT(11)		/* Enable support 64 bit address mode */
#define BEHCI_PRE_EOF1(x)	((x) << 12)
#define BEHCI_PRE_EOF2(x)	((x) << 22)
/*
 * Set preEOF1 to 0x100 and preEOF2 to 0x140
 * (i.e. preEOF1 + MPS 0x40) as the workaround.
 */
#define BEHCI_EOF1_EOF2_TIMING \
	(BEHCI_PRE_EOF1(0x100) | BEHCI_PRE_EOF2(0x140))

static const struct of_device_id aspeed_usb_hp_dt_ids[] = {
	{
		.compatible = "aspeed,ast2600-usb2ahp",
	},
	{
		.compatible = "aspeed,ast2700-usb3hp",
	},
	{
		.compatible = "aspeed,ast2700-usb2hp",
	},
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_usb_hp_dt_ids);

static int aspeed_usb_hp_probe(struct platform_device *pdev)
{
	void __iomem		*regs;
	u32			val;
	struct clk		*clk;
	struct reset_control	*rst;
	struct regmap		*device;
	struct phy		*usb3_phy;
	bool is_pcie_xhci;
	int rc = 0;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "aspeed,ast2600-usb2ahp")) {
		dev_info(&pdev->dev, "Initialized AST2600 USB2AHP\n");
		return 0;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "aspeed,ast2700-usb3hp"))
		is_pcie_xhci = true;
	else if (of_device_is_compatible(pdev->dev.of_node,
					   "aspeed,ast2700-usb2hp"))
		is_pcie_xhci = false;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rc = clk_prepare_enable(clk);
	if (rc) {
		dev_err(&pdev->dev, "Unable to enable clock (%d)\n", rc);
		return rc;
	}

	rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		rc = PTR_ERR(rst);
		goto err;
	}
	rc = reset_control_deassert(rst);
	if (rc)
		goto err;

	device = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "aspeed,device");
	if (IS_ERR(device)) {
		dev_err(&pdev->dev, "failed to find device regmap\n");
		goto err;
	}

	if (is_pcie_xhci) {
		usb3_phy = devm_phy_get(&pdev->dev, "usb3-phy");
		if (IS_ERR(usb3_phy)) {
			rc = dev_err_probe(&pdev->dev, PTR_ERR(usb3_phy),
					   "failed to get usb3 phy\n");
			goto err;
		}
		rc = phy_init(usb3_phy);
		if (rc < 0) {
			dev_err(&pdev->dev, "failed to init usb3 phy\n");
			goto err;
		}
		//EnPCIaMSI_EnPCIaIntA_EnPCIaMst_EnPCIaDev
		/* Turn on PCIe xHCI without MSI */
		regmap_update_bits(device, 0x70,
				   BIT(19) | BIT(11) | BIT(3),
				   BIT(19) | BIT(11) | BIT(3));
	} else {
		regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(regs)) {
			dev_err(&pdev->dev, "Failed to map resources\n");
			rc = PTR_ERR(regs);
			goto err;
		}

		if (device_property_read_bool(&pdev->dev, "aspeed,ehci_32bits_cap")) {
			val = readl(regs + USB_HP_BEHCI84) & ~BEHCI_64BIT;
			writel(val, regs + USB_HP_BEHCI84);
		}

		/* Adjust PCIe-EHCI EOF1/EOF2 timing to workaround the DMA termination bug. */
		writel(BEHCI_EOF1_EOF2_TIMING, regs + USB_HP_BEHCI88);

		//EnPCIaMSI_EnPCIaIntA_EnPCIaMst_EnPCIaDev
		/* Turn on PCIe EHCI without MSI */
		regmap_update_bits(device, 0x70,
				   BIT(18) | BIT(10) | BIT(2),
				   BIT(18) | BIT(10) | BIT(2));
	}
	dev_info(&pdev->dev, "Initialized AST2700 USB Host PCIe %s\n", is_pcie_xhci ? "xHCI" : "EHCI");
	return 0;
err:
	if (clk)
		clk_disable_unprepare(clk);
	return rc;
}

static void aspeed_usb_hp_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Remove USB Host PCIe\n");
}

static struct platform_driver aspeed_usb_hp_driver = {
	.probe		= aspeed_usb_hp_probe,
	.remove		= aspeed_usb_hp_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table	= aspeed_usb_hp_dt_ids,
	},
};
module_platform_driver(aspeed_usb_hp_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neal Liu <neal_liu@aspeedtech.com>");
