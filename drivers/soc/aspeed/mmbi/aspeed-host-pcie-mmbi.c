// SPDX-License-Identifier: GPL-2.0+
/* Implements PCIe Transport in host side for MMBI protocol
 * Copyright 2026 Aspeed Technology Inc.
 */

#include <linux/dev_printk.h>
#include <linux/irqreturn.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include "aspeed-mmbi.h"

#define DRIVER_NAME "aspeed-host-pcie-mmbi"
#define MAX_MSI_NUM 32
/* Value for triggering interrupt */
#define ASPEED_E2M_INT_VAL 0xC6

struct aspeed_host_pcie_mmbi_ins {
	struct mmbi_ins_desc mmbi_desc;
	int irq;
	int msi_index;
	int instance_index;
	int bar_index;
	const char *dev_name;
	bool valid;
};

struct aspeed_host_pcie_mmbi {
	struct device *dev;
	struct aspeed_host_pcie_mmbi_ins instances[PCI_STD_NUM_BARS];
	int msi_nums;
};

static int aspeed_pci_host_mmbi_setup(struct pci_dev *pdev)
{
	struct aspeed_host_pcie_mmbi *mmbi = pci_get_drvdata(pdev);
	struct aspeed_host_pcie_mmbi_ins *ins;
	struct device *dev = &pdev->dev;
	struct mmbi_ins_desc *mmbi_desc;
	int i, ret, instance_id;
	resource_size_t start, size;

	instance_id = 0;
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		ins = &mmbi->instances[i];
		mmbi_desc = &ins->mmbi_desc;
		start = pci_resource_start(pdev, i);
		size = pci_resource_len(pdev, i);

		if (!start || !size) {
			dev_err(dev,
				"Invalid BAR %d with start 0x%llx and size 0x%llx\n",
				i, (unsigned long long)start,
				(unsigned long long)size);
			continue;
		}

		mmbi_desc->desc_virt = pci_ioremap_bar(pdev, i);
		if (!mmbi_desc->desc_virt) {
			dev_err(dev, "Failed to ioremap BAR %d\n", i);
			continue;
		}
		mmbi_desc->dev = dev;
		mmbi_desc->role = MMBI_ROLE_HOST;

		ret = mmbi_instance_init(mmbi_desc);
		if (ret) {
			dev_info(dev, "Failed to initialize MMBI at bar %d, error %d\n", i, ret);
			 /* Unmap the previously mapped BAR */
			iounmap(mmbi_desc->desc_virt);
		} else {
			ins->instance_index = instance_id++;
			ins->bar_index = i;
			ins->valid = true;
			ins->dev_name = devm_kasprintf(dev, GFP_KERNEL, "mmbi_host_ins%d", ins->instance_index);

			if (mmbi_desc->host_int_type == MMBI_HOST_INT_PCIE) {
				if (mmbi_desc->host_int_location > mmbi->msi_nums) {
					dev_err(dev,
						"Invalid host interrupt location %d for MMBI instance at BAR %d, use MSI 0\n",
						mmbi_desc->host_int_location,
						i);
					ins->msi_index = 0;
				} else {
					ins->msi_index = mmbi_desc->host_int_location;
				}
			}
		}
	}

	return 0;
}

static int aspeed_pci_host_pcie_mmbi_probe(struct pci_dev *pdev,
					   const struct pci_device_id *id)
{
	struct aspeed_host_pcie_mmbi *mmbi;
	int rc = 0;

	mmbi = devm_kzalloc(&pdev->dev, sizeof(*mmbi), GFP_KERNEL);
	if (!mmbi)
		return -ENOMEM;

	mmbi->dev = &pdev->dev;

	rc = pci_enable_device(pdev);
	if (rc)
		return rc;

	pci_set_master(pdev);
	pci_set_drvdata(pdev, mmbi);
	rc = aspeed_pci_host_mmbi_setup(pdev);
	if (rc) {
		dev_err(&pdev->dev, "Failed to setup MMBI instances\n");
		goto err_pci;
	}

	dev_info(&pdev->dev, "ASPEED Host PCIe MMBI device probed successfully\n");
	return 0;

err_pci:
	pci_disable_device(pdev);
	return dev_err_probe(&pdev->dev, rc,
			     "Failed to probe ASPEED Host PCIe MMBI device\n");
}

static void aspeed_pci_host_pcie_mmbi_remove(struct pci_dev *pdev)
{
	struct aspeed_host_pcie_mmbi *mmbi;
	struct aspeed_host_pcie_mmbi_ins *ins;
	int i;

	mmbi = pci_get_drvdata(pdev);
	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		ins = &mmbi->instances[i];
		if (ins->valid) {
			mmbi_instance_remove(&ins->mmbi_desc);
			pci_iounmap(pdev, ins->mmbi_desc.desc_virt);
		}
	}

	pci_disable_device(pdev);
}

static struct pci_device_id aspeed_host_pcie_mmbi_pci_ids[] = {
	{ PCI_DEVICE(0x1A03, 0x2402), .class = 0x0C0C00, .class_mask = (0xFFFF00)},
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, aspeed_host_pcie_mmbi_pci_ids);

static struct pci_driver aspeed_host_pcie_mmbi_driver = {
	.name		= DRIVER_NAME,
	.id_table	= aspeed_host_pcie_mmbi_pci_ids,
	.probe		= aspeed_pci_host_pcie_mmbi_probe,
	.remove		= aspeed_pci_host_pcie_mmbi_remove,
};

static int __init aspeed_host_pcie_mmbi_init(void)
{
	return pci_register_driver(&aspeed_host_pcie_mmbi_driver);
}

static void aspeed_host_pcie_mmbi_exit(void)
{
	/* unregister pci driver */
	pci_unregister_driver(&aspeed_host_pcie_mmbi_driver);
}

late_initcall(aspeed_host_pcie_mmbi_init);
module_exit(aspeed_host_pcie_mmbi_exit);

MODULE_AUTHOR("YH Chung <yh_chung@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Host PCIE MMBI Transport Driver");
MODULE_LICENSE("GPL");
