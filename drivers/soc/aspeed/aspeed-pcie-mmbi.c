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

#include "aspeed-pcie-mmbi.h"

/* AST2700 E2M */
#define ASPEED_E2M_EVENT		0x0D0
#define ASPEED_E2M_EVENT_SET		0x0D4
#define ASPEED_E2M_EVENT_CLR		0x0D8
#define ASPEED_E2M_EVENT_EN		0x0DC
#define ASPEED_E2M_ADRMAP00		0x100
#define ASPEED_E2M_WIRQA0		0x180
#define ASPEED_E2M_WIRQV0		0x1C0
#define ASPEED_E2M_SPROT_SIDG0		0x210
#define ASPEED_E2M_SPROT_CTL0		0x280
#define ASPEED_E2M_SPROT_ADR0		0x2C0

/* AST2700 SCU */
#define ASPEED_SCU_DECODE_DEV		BIT(18)
#define ASPEED_SCU_INT_EN		BIT(23)
struct aspeed_platform {
	int (*mmbi_init)(struct platform_device *pdev);
};

struct aspeed_pcie_mmbi {
	struct device *dev;
	struct regmap *device;
	struct regmap *e2m;
	int irq;
	const struct aspeed_platform *platform;
	/* E2M index */
	int id;
	int pid;
	int scu_bar_offset;
	int e2m_index;

	/* MISC */
	struct miscdevice mdev;
	wait_queue_head_t wq;

	/* Memory Mapping */
	void __iomem *mem_virt;
	dma_addr_t mem_phy;
	phys_addr_t mem_size;

	/* BMC Interrupt */
	bool bmc_int_en;
	u8 bmc_int_byte;
	u32 bmc_int_offset;
	bool bmc_int_update;
	wait_queue_head_t bmc_int_wq;
};

static struct aspeed_pcie_mmbi *file_aspeed_pcie_mmbi(struct file *file)
{
	return container_of(file->private_data, struct aspeed_pcie_mmbi, mdev);
}

static int aspeed_pcie_mmbi_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_pcie_mmbi *mmbi = file_aspeed_pcie_mmbi(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vsize) > mmbi->mem_size)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (mmbi->mem_phy >> PAGE_SHIFT) + vma->vm_pgoff, vsize, prot))
		return -EAGAIN;

	return 0;
}

static __poll_t aspeed_pcie_mmbi_poll(struct file *file, struct poll_table_struct *pt)
{
	struct aspeed_pcie_mmbi *mmbi = file_aspeed_pcie_mmbi(file);

	poll_wait(file, &mmbi->bmc_int_wq, pt);

	if (!mmbi->bmc_int_update)
		return 0;

	mmbi->bmc_int_update = false;

	return EPOLLIN;
}

static const struct file_operations aspeed_pcie_mmbi_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_pcie_mmbi_mmap,
	.poll		= aspeed_pcie_mmbi_poll,
};

static irqreturn_t aspeed_pcie_mmbi_isr(int irq, void *dev_id)
{
	struct aspeed_pcie_mmbi *mmbi = dev_id;

	mmbi->bmc_int_update = true;
	wake_up_interruptible(&mmbi->bmc_int_wq);

	return IRQ_HANDLED;
}

/*
 * AST2700 PCIe MMBI (SCU & E2M)
 * SoC      |    0                                    |    1                          |
 * PCI class|    MFD (0xFF_00_00)                     |    MMBI (0x0C_0C_00)          |
 * Node     |    0                   1                |    0                          |
 * Alias id |    0    1    2    3    4    5    6    7 |    8    9   10   11   12   13 |
 * PID      |    3    4    5    6   11   12   13   14 |    2    3    4    5    6    7 |
 * E2M index|    0    1    2    3    4    5    6    7 |    0    1    2    3    4    5 |
 * BAR index|    2    3    4    5    2    3    4    5 |    0    1    2    3    4    5 |
 * SCU BAR  |   3c   4c   5c   6c   3c   4c   5c   6c |   1c   50   3c   4c   5c   6c |
 */
static int aspeed_ast2700_pcie_mmbi_init(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 value, sprot_size, e2m_index, pid;
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
		mmbi->id = 0;	/* 0x12c21000 */

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
	if (mmbi->id < 2) {
		/* PCIe device class, sub-class, protocol and reversion */
		regmap_write(mmbi->device, 0x18, 0xFF000027);
	} else {
		regmap_write(mmbi->device, 0x18, 0x0C0C0027);
		regmap_write(mmbi->device, 0x78, ASPEED_SCU_INT_EN | ASPEED_SCU_DECODE_DEV);
	}

	/* MSI */
	regmap_update_bits(mmbi->device, 0x74, GENMASK(7, 4), BIT(7) | (5 << 4));

	regmap_update_bits(mmbi->device, 0x70,
			   BIT(25) | BIT(17) | BIT(9) | BIT(1),
			   BIT(25) | BIT(17) | BIT(9) | BIT(1));

	/* Create MISC device for MMBI */
	mmbi->mdev.parent = dev;
	mmbi->mdev.minor = MISC_DYNAMIC_MINOR;
	mmbi->mdev.name =
		devm_kasprintf(dev, GFP_KERNEL, "pcie%d-mmbi%d", mmbi->id, e2m_index);
	mmbi->mdev.fops = &aspeed_pcie_mmbi_fops;
	ret = misc_register(&mmbi->mdev);
	if (ret) {
		dev_err(dev, "cannot register device %s\n", mmbi->mdev.name);
		return ret;
	}
	init_waitqueue_head(&mmbi->wq);

	/* Calculate the BAR Size */
	for (i = 1; i < 16; i++) {
		/* bar size check for 4k align */
		if ((mmbi->mem_size / 4096) == (1 << (i - 1)))
			break;
	}
	if (i == 16) {
		i = 0;
		dev_warn(dev, "Bar size not align for 4K : %dK\n",
			 (u32)mmbi->mem_size / 1024);
	}
	regmap_write(mmbi->device, mmbi->scu_bar_offset, (mmbi->mem_phy >> 4) | i);
	regmap_write(mmbi->e2m, ASPEED_E2M_ADRMAP00 + (4 * pid), (mmbi->mem_phy >> 4) | i);

	/* BMC Interrupt */
	if (mmbi->bmc_int_en) {
		value = mmbi->mem_phy + mmbi->bmc_int_offset;
		regmap_write(mmbi->e2m, ASPEED_E2M_WIRQA0 + (4 * e2m_index), value);
		value = (BIT(16) << pid) | mmbi->bmc_int_byte;
		regmap_write(mmbi->e2m, ASPEED_E2M_WIRQV0 + (4 * e2m_index), value);
	}

	/* HOST Interrupt: MSI */
	regmap_read(mmbi->e2m, ASPEED_E2M_EVENT_EN, &value);
	value |= BIT(e2m_index);
	regmap_write(mmbi->e2m, ASPEED_E2M_EVENT_EN, value);

	/* B2H Write Protect */
	sprot_size = (mmbi->mem_size / 2) / SZ_1M;
	value = (sprot_size << 16) | (mmbi->mem_phy >> 20);
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_ADR0 + (4 * e2m_index), value);
	/* Enable read & disalbe write */
	value = 1 << (8 + e2m_index);
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_CTL0 + (4 * e2m_index), value);
	/* Set PID */
	regmap_read(mmbi->e2m, ASPEED_E2M_SPROT_SIDG0 + (4 * (e2m_index / 4)), &value);
	value |= pid << (8 * (e2m_index % 4));
	regmap_write(mmbi->e2m, ASPEED_E2M_SPROT_SIDG0 + (4 * (e2m_index / 4)), value);

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
	struct device *dev = &pdev->dev;
	struct resource res;
	struct device_node *np;
	const void *md;
	int ret = 0;

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
	mmbi->mem_virt = devm_ioremap_resource(dev, &res);
	if (!mmbi->mem_virt) {
		dev_err(dev, "cannot map mmbi memory region\n");
		ret = -ENOMEM;
		goto out_region;
	}

	/* Get IRQ */
	mmbi->irq = platform_get_irq(pdev, 0);
	if (mmbi->irq < 0) {
		dev_err(&pdev->dev, "platform get of irq[=%d] failed!\n", mmbi->irq);
		goto out_unmap;
	}
	ret = devm_request_irq(&pdev->dev, mmbi->irq, aspeed_pcie_mmbi_isr, 0,
			       dev_name(&pdev->dev), mmbi);
	if (ret) {
		dev_err(dev, "pcie mmbi unable to get IRQ");
		goto out_unmap;
	}

	init_waitqueue_head(&mmbi->bmc_int_wq);

	mmbi->bmc_int_en = true;
	/* H2B Interrupt */
	ret = of_property_read_u8(dev->of_node, "mmbi-bmc-int-value", &mmbi->bmc_int_byte);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI H2B interrupt byte\n");
		mmbi->bmc_int_en = false;
	}
	ret = of_property_read_u32(dev->of_node, "mmbi-bmc-int-offset", &mmbi->bmc_int_offset);
	if (ret) {
		dev_err(dev, "cannot get valid MMBI H2B interrupt offset\n");
		mmbi->bmc_int_en = false;
	}

	ret = mmbi->platform->mmbi_init(pdev);
	if (ret) {
		dev_err(dev, "Initialize pcie mmbi failed\n");
		goto out_irq;
	}

	dev_info(dev, "ASPEED PCIe MMBI Dev %d: driver successfully loaded.\n", mmbi->id);

	return 0;
out_irq:
	devm_free_irq(dev, mmbi->irq, mmbi);
out_unmap:
	devm_iounmap(dev, mmbi->mem_virt);
out_region:
	devm_kfree(dev, mmbi);
	dev_warn(dev, "aspeed bmc device: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int  aspeed_pcie_mmbi_remove(struct platform_device *pdev)
{
	struct aspeed_pcie_mmbi *mmbi = platform_get_drvdata(pdev);

	misc_deregister(&mmbi->mdev);
	devm_free_irq(&pdev->dev, mmbi->irq, mmbi);
	devm_iounmap(&pdev->dev, mmbi->mem_virt);
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
