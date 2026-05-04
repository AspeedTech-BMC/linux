// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/string.h>

#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>

#include "aspeed-pcie-mmbi.h"
#include "aspeed-mmbi.h"

/* AST2700 E2M */
#define ASPEED_E2M_EVENT 0x0D0
#define ASPEED_E2M_EVENT_SET 0x0D4
#define ASPEED_E2M_EVENT_CLR 0x0D8
#define ASPEED_E2M_EVENT_EN 0x0DC
#define ASPEED_E2M_ADRMAP00 0x100
#define ASPEED_E2M_WIRQA0 0x180
#define ASPEED_E2M_WIRQV0 0x1C0
#define ASPEED_E2M_SPROT_SIDG0 0x210
#define ASPEED_E2M_SPROT_CTL0 0x280
#define ASPEED_E2M_SPROT_ADR0 0x2C0
/* Value for triggering interrupt */
#define ASPEED_E2M_INT_VAL 0xC6

/* AST2700 SCU */
#define ASPEED_SCU_DECODE_DEV BIT(18)
#define ASPEED_SCU_INT_EN BIT(23)
struct aspeed_platform {
	int (*mmbi_init)(struct platform_device *pdev);
};

#define ASPEED_MMBI_APP_INTF_PROP "aspeed,mmbi-interface"

struct aspeed_pcie_mmbi {
	struct device *dev;
	struct regmap *device;
	struct regmap *e2m;
	int irq;
	const struct aspeed_platform *platform;
	/* E2M index */
	int id;
	int pid;
	int msi_index;
	int scu_bar_offset;
	int e2m_index;
	int e2m_h2b_int;
	int num_of_channels;
	bool host_int_en;

	/* Memory Mapping */
	void __iomem *mem_virt;
	dma_addr_t mem_phy;
	phys_addr_t mem_size;

	struct mmbi_ins_desc mmbi_desc;
};

static const char *
aspeed_pcie_mmbi_app_interface_name(enum mmbi_app_interface app_interface)
{
	switch (app_interface) {
	case MMBI_APP_INTF_MCTP_NETDEV:
		return "mctp-netdev";
	case MMBI_APP_INTF_IOCTL:
	default:
		return "ioctl";
	}
}

static int aspeed_pcie_mmbi_parse_app_interface_string(struct device *dev,
						       const char *interface,
						       enum mmbi_app_interface *app_interface)
{
	if (!strcmp(interface, "ioctl")) {
		*app_interface = MMBI_APP_INTF_IOCTL;
		return 0;
	}

	if (!strcmp(interface, "mctp-netdev")) {
		*app_interface = MMBI_APP_INTF_MCTP_NETDEV;
		return 0;
	}

	dev_err(dev, "invalid %s value '%s'\n",
		ASPEED_MMBI_APP_INTF_PROP, interface);
	return -EINVAL;
}

static int aspeed_pcie_mmbi_parse_app_interface(struct device *dev,
						struct mmbi_ins_desc *mmbi_desc)
{
	int count, i, ret;

	for (i = 0; i < mmbi_desc->num_of_channels; i++)
		mmbi_desc->chan_desc[i].app_interface = MMBI_APP_INTF_IOCTL;

	if (!of_find_property(dev->of_node, ASPEED_MMBI_APP_INTF_PROP, NULL))
		return 0;

	count = of_property_count_strings(dev->of_node, ASPEED_MMBI_APP_INTF_PROP);
	if (count < 0) {
		dev_err(dev, "invalid %s property\n", ASPEED_MMBI_APP_INTF_PROP);
		return count;
	}

	if (count != 1 && count != mmbi_desc->num_of_channels) {
		dev_err(dev, "%s must contain either 1 or %u strings\n",
			ASPEED_MMBI_APP_INTF_PROP,
			(u32)mmbi_desc->num_of_channels);
		return -EINVAL;
	}

	for (i = 0; i < mmbi_desc->num_of_channels; i++) {
		const char *interface;
		int index = count == 1 ? 0 : i;

		ret = of_property_read_string_index(dev->of_node,
						    ASPEED_MMBI_APP_INTF_PROP,
						    index, &interface);
		if (ret)
			return ret;

		ret = aspeed_pcie_mmbi_parse_app_interface_string(dev, interface,
								  &mmbi_desc->chan_desc[i].app_interface);
		if (ret)
			return ret;
	}

	return 0;
}

static irqreturn_t aspeed_pcie_mmbi_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

/**
 * Detailed Buffer Layout:
 *
 * +---------------------------------------+
 * |     HOST READ ONLY REGION (50%)       |
 * +---------------------------------------+
 * | mmbi_desc                             |
 * |   - Size: 32 + 32*noi bytes           |
 * +---------------------------------------+
 * | Interrupt Region (8 bytes)            |
 * |   - trigger_int (1 byte)              |
 * |   - reserved (7 bytes)                |
 * +---------------------------------------+
 * | Channel 0:                            |
 * |   hos_ro (8 bytes)                    |
 * |     - First 8 bytes after interrupt   |
 * |   b2h_buffer                          |
 * |     - Channel 0 B2H buffer space      |
 * +---------------------------------------+
 * | Channel 1:                            |
 * |   hos_ro (8 bytes)                    |
 * |   b2h_buffer                          |
 * |     - Channel 1 B2H buffer space      |
 * +---------------------------------------+
 * | ...                                   |
 * +---------------------------------------+
 * | Channel 6:                            |
 * |   hos_ro (8 bytes)                    |
 * |   b2h_buffer                          |
 * |     - Channel 6 B2H buffer space      |
 * +---------------------------------------+
 * |   HOST READ/WRITE REGION (50%)        |
 * +---------------------------------------+
 * | Channel 0:                            |
 * |   hos_rw (8 bytes)                    |
 * |     - First 8 bytes of this region    |
 * |   h2b_buffer                          |
 * |     - Channel 0 H2B buffer space      |
 * +---------------------------------------+
 * | Channel 1:                            |
 * |   hos_rw (8 bytes)                    |
 * |   h2b_buffer                          |
 * |     - Channel 1 H2B buffer space      |
 * +---------------------------------------+
 * | ...                                   |
 * +---------------------------------------+
 * | Channel 6:                            |
 * |   hos_rw (8 bytes)                    |
 * |   h2b_buffer                          |
 * |     - Channel 6 H2B buffer space      |
 * +---------------------------------------+
 */
static int aspeed_pcie_mmbi_init(struct aspeed_pcie_mmbi *mmbi)
{
	struct mmbi_ins_desc *mmbi_desc = &mmbi->mmbi_desc;
	struct mmbi_buf_vpscb *vpscb;
	struct device *dev = mmbi->dev;
	u32 desc_size, b2h_buf_size, h2b_buf_size;
	int i, ret;

	desc_size = MMBI_DESC_SIZE_PREFIX + MMBI_DESC_SIZE_INTERRUPT +
		    MMBI_DESC_SIZE_CHANNEL * mmbi->num_of_channels + 8;

	if ((mmbi->mem_size >> 1) <= desc_size) {
		dev_err(dev, "MMBI memory size too small for B2H buffer\n");
		return -EINVAL;
	}

	b2h_buf_size =
		rounddown_pow_of_two(((mmbi->mem_size >> 1) - desc_size) /
				     mmbi->num_of_channels);

	h2b_buf_size = rounddown_pow_of_two((mmbi->mem_size >> 1) / mmbi->num_of_channels);

	mmbi_desc->desc_virt = mmbi->mem_virt;
	mmbi_desc->mmbi_version = MMBI_VERSION_1_1;
	mmbi_desc->os_use = 1;
	mmbi_desc->num_of_channels = mmbi->num_of_channels;
	mmbi_desc->host_int_type = MMBI_HOST_INT_PCIE;
	mmbi_desc->host_int_location = mmbi->msi_index;
	mmbi_desc->host_int_value = 0; /* cleared to 0 */
	mmbi_desc->bmc_int_type = MMBI_BMC_INT_INBAND;
	mmbi_desc->bmc_int_location = 0; /* reserved */
	mmbi_desc->bmc_int_value = 0; /* cleared to 0 */
	mmbi_desc->dev = dev;
	mmbi_desc->role = MMBI_ROLE_BMC;

	for (i = 0; i < mmbi_desc->num_of_channels; i++) {
		vpscb = &mmbi_desc->chan_desc[i].buffer_desc;
		vpscb->h_ros_p = desc_size + i * b2h_buf_size;
		vpscb->h_rws_p = (mmbi->mem_size >> 1) + i * h2b_buf_size;
		vpscb->host_int_val_ch = 0;
		vpscb->bmc_int_val_ch = 0;

		mmbi_desc->chan_desc[i].b2h_ba_offset = vpscb->h_ros_p + 8;
		mmbi_desc->chan_desc[i].h2b_ba_offset = vpscb->h_rws_p + 8;
		mmbi_desc->chan_desc[i].b2h_l = b2h_buf_size - 8;
		mmbi_desc->chan_desc[i].h2b_l = h2b_buf_size - 8;
		// mmbi_desc->chan_desc[i].h2b_ba_offset = vpscb->h_rws_p + 16;
		// mmbi_desc->chan_desc[i].b2h_l = 0x100;
		// mmbi_desc->chan_desc[i].h2b_l = 0x100;
		mmbi_desc->chan_desc[i].buffer_type = MMBI_BUFFER_TYPE_VPSCB;
	}
	ret = mmbi_instance_init(mmbi_desc);
	if (ret)
		dev_err(dev, "MMBI instance init failed: %d\n", ret);

	return ret;
}

/*
 * AST2700 PCIe MMBI (SCU & E2M)
 * SoC         |    0                                    |    1                          |
 * PCI class   |    MFD (0xFF_00_00)                     |    MMBI (0x0C_0C_00)          |
 * Node        |    0                   1                |    0                          |
 * PID         |    3    4    5    6   11   12   13   14 |    2    3    4    5    6    7 |
 * E2M index   |    0    1    2    3    4    5    6    7 |    0    1    2    3    4    5 |
 * BAR index   |    2    3    4    5    2    3    4    5 |    0    1    2    3    4    5 |
 * SCU BAR     |   3c   4c   5c   6c   3c   4c   5c   6c |   1c   50   3c   4c   5c   6c |
 * E2M H2B Int |    0    1    2    3    0    1    2    3 |    0    1    2    3    4    5 | (bit)
 */
static int aspeed_ast2700_pcie_mmbi_init(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 value, e2m_index, pid;
	struct resource res;
	int ret, i;

	/* Get register map*/
	mmbi->e2m = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(mmbi->e2m)) {
		dev_err(dev, "failed to find e2m regmap\n");
		return PTR_ERR(mmbi->e2m);
	}
	if (of_address_to_resource(dev->of_node->parent, 0, &res)) {
		dev_err(dev, "Failed to get e2m resource\n");
		return -EINVAL;
	}
	if (res.start == 0x14c1d000)
		mmbi->id = 2;
	else if (res.start == 0x12c22000)
		mmbi->id = 1;
	else
		mmbi->id = 0; /* 0x12c21000 */

	mmbi->device = syscon_regmap_lookup_by_phandle(dev->of_node->parent, "aspeed,device");
	if (IS_ERR(mmbi->device)) {
		dev_err(dev, "failed to find device regmap\n");
		return PTR_ERR(mmbi->device);
	}

	ret = of_property_read_u32(dev->of_node, "index", &mmbi->e2m_index);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi index value\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "pid", &mmbi->pid);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi pid value\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "bar", &mmbi->scu_bar_offset);
	if (ret < 0) {
		dev_err(dev, "cannot get mmbi bar value\n");
		return ret;
	}

	e2m_index = mmbi->e2m_index;
	pid = mmbi->pid;
	mmbi->e2m_h2b_int += mmbi->e2m_index;
	if (mmbi->id < 2) {
		/* PCIe device class, sub-class, protocol and reversion */
		regmap_write(mmbi->device, 0x18, 0x0C0C0027);
	} else {
		regmap_write(mmbi->device, 0x18, 0x0C0C0027);
		regmap_write(mmbi->device, 0x78, ASPEED_SCU_INT_EN | ASPEED_SCU_DECODE_DEV);
	}

	/* MSI */
	regmap_update_bits(mmbi->device, 0x74, GENMASK(7, 4), BIT(7) | (5 << 4));

	regmap_update_bits(mmbi->device, 0x70, BIT(25) | BIT(17) | BIT(9) | BIT(1),
			   BIT(25) | BIT(17) | BIT(9) | BIT(1));

	/* Calculate the BAR Size */
	for (i = 1; i < 16; i++) {
		/* bar size check for 4k align */
		if ((mmbi->mem_size / 4096) == (1 << (i - 1)))
			break;
	}
	if (i == 16) {
		i = 0;
		dev_warn(dev, "Bar size not align for 4K : %dK\n", (u32)mmbi->mem_size / 1024);
	}
	regmap_write(mmbi->device, mmbi->scu_bar_offset, (mmbi->mem_phy >> 4) | i);
	regmap_write(mmbi->e2m, ASPEED_E2M_ADRMAP00 + (4 * pid), (mmbi->mem_phy >> 4) | i);

	/* BMC Interrupt: E2M */
	value = MMBI_DESC_SIZE_PREFIX + MMBI_DESC_SIZE_INTERRUPT +
		MMBI_DESC_SIZE_CHANNEL * mmbi->num_of_channels;
	value += mmbi->mem_phy;
	regmap_write(mmbi->e2m, ASPEED_E2M_WIRQA0 + (4 * e2m_index), value);
	value = (BIT(16) << pid) | ASPEED_E2M_INT_VAL;
	regmap_write(mmbi->e2m, ASPEED_E2M_WIRQV0 + (4 * e2m_index), value);

	/* HOST Interrupt: MSI */
	regmap_read(mmbi->e2m, ASPEED_E2M_EVENT_EN, &value);
	value |= BIT(mmbi->e2m_h2b_int);
	regmap_write(mmbi->e2m, ASPEED_E2M_EVENT_EN, value);

	ret = aspeed_pcie_mmbi_init(mmbi);
	if (ret < 0) {
		dev_err(dev, "Initialize MMBI device failed.\n");
		return ret;
	}

	return 0;
}

struct aspeed_platform ast2700_platform = {
	.mmbi_init = aspeed_ast2700_pcie_mmbi_init,
};

static const struct of_device_id aspeed_pcie_mmbi_of_matches[] = {
	{ .compatible = "aspeed,ast2700-pcie-mmbi", .data = &ast2700_platform },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_pcie_mmbi_of_matches);

static int aspeed_pcie_mmbi_probe(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi;
	struct mmbi_ins_desc *mmbi_desc;
	struct device *dev = &pdev->dev;
	struct resource res;
	struct device_node *np;
	const void *md;
	int i, ret = 0;

	md = of_device_get_match_data(dev);
	if (!md)
		return -ENODEV;

	mmbi = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_pcie_mmbi), GFP_KERNEL);
	if (!mmbi)
		return -ENOMEM;
	dev_set_drvdata(dev, mmbi);

	mmbi->dev = dev;
	mmbi->platform = md;

	/* Get MMBI memory size */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np || of_address_to_resource(np, 0, &res)) {
		dev_err(dev, "Failed to find memory-region.\n");
		ret = -ENOMEM;
		goto out_region;
	}

	of_node_put(np);

	mmbi->mem_phy = res.start;
	mmbi->mem_size = resource_size(&res);
	mmbi->mem_virt = ioremap(mmbi->mem_phy, mmbi->mem_size);
	if (!mmbi->mem_virt) {
		dev_err(dev, "cannot map mmbi memory region\n");
		ret = -ENOMEM;
		goto out_region;
	}

	/* Get IRQ */
	mmbi->irq = platform_get_irq(pdev, 0);
	if (mmbi->irq < 0) {
		dev_err(&pdev->dev, "platform get of irq[=%d] failed!\n", mmbi->irq);
		ret = mmbi->irq;
		goto out_unmap;
	}
	ret = devm_request_irq(&pdev->dev, mmbi->irq, aspeed_pcie_mmbi_isr, 0, dev_name(&pdev->dev),
			       mmbi);
	if (ret) {
		dev_err(dev, "pcie mmbi unable to get IRQ");
		goto out_unmap;
	}

	mmbi_desc = &mmbi->mmbi_desc;
	memset(mmbi_desc, 0, sizeof(struct mmbi_ins_desc));

	ret = of_property_read_u32(dev->of_node, "noi", &mmbi->num_of_channels);
	if (ret || mmbi->num_of_channels == 0 ||
	    mmbi->num_of_channels > MMBI_MAX_CHANNELS) {
		dev_err(dev, "ret %d, MMBI NOI %d\n", ret, mmbi->num_of_channels);
		goto out_irq;
	}
	mmbi_desc->num_of_channels = mmbi->num_of_channels;

	ret = aspeed_pcie_mmbi_parse_app_interface(dev, mmbi_desc);
	if (ret)
		goto out_irq;

	/* B2H Interrupt */
	mmbi->host_int_en = true;
	ret = of_property_read_u32(dev->of_node, "msi", &mmbi->msi_index);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI B2H interrupt location\n");
		mmbi->host_int_en = false;
	}

	ret = mmbi->platform->mmbi_init(pdev);
	if (ret) {
		dev_err(dev, "Initialize pcie mmbi failed\n");
		goto out_irq;
	}

	for (i = 0; i < mmbi_desc->num_of_channels; i++)
		dev_info(dev, "channel %d application interface: %s\n",
			 i, aspeed_pcie_mmbi_app_interface_name(mmbi_desc->chan_desc[i].app_interface));
	dev_info(dev, "ASPEED PCIe MMBI Dev %d: driver successfully loaded.\n", mmbi->id);

	return 0;
out_irq:
	devm_free_irq(dev, mmbi->irq, mmbi);
out_unmap:
	iounmap(mmbi->mem_virt);
out_region:
	devm_kfree(dev, mmbi);
	dev_warn(dev, "aspeed pcie mmbi: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_pcie_mmbi_remove(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);

	mmbi_instance_remove(&mmbi->mmbi_desc);
	devm_free_irq(&pdev->dev, mmbi->irq, mmbi);
	iounmap(mmbi->mem_virt);
	devm_kfree(&pdev->dev, mmbi);

	return 0;
}

static struct platform_driver aspeed_pcie_mmbi_driver = {
	.probe		= aspeed_pcie_mmbi_probe,
	.remove		= aspeed_pcie_mmbi_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_pcie_mmbi_of_matches,
	},
};

module_platform_driver(aspeed_pcie_mmbi_driver);

MODULE_AUTHOR("Jacky Chou <jacky_chou@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PCI-E MMBI Driver");
MODULE_LICENSE("GPL");
