// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.

#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <linux/pci.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include "../../char/ipmi/ipmi_si.h"

#define ASPEED_PCI_BMC_HOST2BMC_Q1		0x30000
#define ASPEED_PCI_BMC_HOST2BMC_Q2		0x30010
#define ASPEED_PCI_BMC_BMC2HOST_Q1		0x30020
#define ASPEED_PCI_BMC_BMC2HOST_Q2		0x30030
#define ASPEED_PCI_BMC_BMC2HOST_STS		0x30040
#define	 BMC2HOST_INT_STS_DOORBELL		BIT(31)
#define	 BMC2HOST_ENABLE_INTB			BIT(30)
/* */
#define	 BMC2HOST_Q1_FULL				BIT(27)
#define	 BMC2HOST_Q1_EMPTY				BIT(26)
#define	 BMC2HOST_Q2_FULL				BIT(25)
#define	 BMC2HOST_Q2_EMPTY				BIT(24)
#define	 BMC2HOST_Q1_FULL_UNMASK		BIT(23)
#define	 BMC2HOST_Q1_EMPTY_UNMASK		BIT(22)
#define	 BMC2HOST_Q2_FULL_UNMASK		BIT(21)
#define	 BMC2HOST_Q2_EMPTY_UNMASK		BIT(20)

#define ASPEED_PCI_BMC_HOST2BMC_STS		0x30044
#define	 HOST2BMC_INT_STS_DOORBELL		BIT(31)
#define	 HOST2BMC_ENABLE_INTB			BIT(30)
/* */
#define	 HOST2BMC_Q1_FULL				BIT(27)
#define	 HOST2BMC_Q1_EMPTY				BIT(26)
#define	 HOST2BMC_Q2_FULL				BIT(25)
#define	 HOST2BMC_Q2_EMPTY				BIT(24)
#define	 HOST2BMC_Q1_FULL_UNMASK		BIT(23)
#define	 HOST2BMC_Q1_EMPTY_UNMASK		BIT(22)
#define	 HOST2BMC_Q2_FULL_UNMASK		BIT(21)
#define	 HOST2BMC_Q2_EMPTY_UNMASK		BIT(20)

struct aspeed_pci_bmc_dev {
	struct device *dev;
	struct miscdevice miscdev;

	unsigned long mem_bar_base;
	unsigned long mem_bar_size;
	void __iomem *mem_bar_reg;

	unsigned long message_bar_base;
	unsigned long message_bar_size;
	void __iomem *msg_bar_reg;

	struct bin_attribute	bin0;
	struct bin_attribute	bin1;

	struct kernfs_node	*kn0;
	struct kernfs_node	*kn1;

	void *serial_priv;

	u8 IntLine;
};

#define HOST_BMC_QUEUE_SIZE			(16 * 4)

#define BMC_MSI_INT
#define BMC_MULTI_MSI	32

#define DRIVER_NAME "ASPEED BMC DEVICE"

#ifdef CONFIG_IPMI_SI
#define KCS_MAX_PARMS		4
static uint16_t kcs_ioport[KCS_MAX_PARMS];
static uint16_t kcs_sirq[KCS_MAX_PARMS];
#endif

#define VUART_MAX_PARMS		2
static uint16_t vuart_ioport[VUART_MAX_PARMS];
static uint16_t vuart_sirq[VUART_MAX_PARMS];

static struct aspeed_pci_bmc_dev *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_pci_bmc_dev,
			miscdev);
}

static int aspeed_pci_bmc_dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = file_aspeed_bmc_device(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (vma->vm_pgoff + vsize > pci_bmc_dev->mem_bar_base + 0x100000)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
		(pci_bmc_dev->mem_bar_base >> PAGE_SHIFT) + vma->vm_pgoff,
		vsize, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_pci_bmc_dev_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_pci_bmc_dev_mmap,
};

static ssize_t aspeed_pci_bmc_dev_queue1_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 *data = (u32 *) buf;
	int rc;

	if (readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q1_EMPTY)
		rc = 0;
	else {
		data[0] = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_Q1);
		rc = 4;
	}

	return rc;
}

static ssize_t aspeed_pci_bmc_dev_queue2_rx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 *data = (u32 *) buf;
	int rc;

	if (!(readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q2_EMPTY)) {
		data[0] = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_Q2);
		rc = 4;
	} else
		rc = 0;

	return rc;
}

static ssize_t aspeed_pci_bmc_dev_queue1_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 tx_buff;
	int rc;

	if (count != 4)
		return -EINVAL;

	if (readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q1_FULL)
		rc = -ENOSPC;
	else {
		memcpy(&tx_buff, buf, 4);
		writel(tx_buff, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_Q1);
		//trigger to host
		writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS);
		rc = 4;
	}
	return rc;
}

static ssize_t aspeed_pci_bmc_dev_queue2_tx(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_get_drvdata(container_of(kobj, struct device, kobj));
	u32 tx_buff = 0;
	int rc;

	if (count != 4)
		return -EINVAL;

	if (readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q2_FULL)
		rc = -ENOSPC;
	else {
		memcpy(&tx_buff, buf, 4);
		writel(tx_buff, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_Q2);
		//trigger to host
		writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_HOST2BMC_STS);
		rc = 4;
	}
	return rc;
}

irqreturn_t aspeed_pci_host_bmc_device_interrupt(int irq, void *dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;
	u32 bmc2host_q_sts = readl(pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_INT_STS_DOORBELL)
		writel(BMC2HOST_INT_STS_DOORBELL, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_ENABLE_INTB)
		writel(BMC2HOST_ENABLE_INTB, pci_bmc_device->msg_bar_reg + ASPEED_PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_Q1_FULL)
		dev_info(pci_bmc_device->dev, "Q2 Full\n");

	if (bmc2host_q_sts & BMC2HOST_Q2_FULL)
		dev_info(pci_bmc_device->dev, "Q2 Full\n");

	return IRQ_HANDLED;

}

#define BMC_MSI_IDX_BASE	4
static int aspeed_pci_host_bmc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
#ifdef CONFIG_IPMI_SI
	struct si_sm_io *kcs_io = kmalloc(sizeof(struct si_sm_io) * KCS_MAX_PARMS, GFP_KERNEL);
#endif
	struct uart_8250_port uart[VUART_MAX_PARMS];
	struct aspeed_pci_bmc_dev *pci_bmc_dev;
	struct device *dev = &pdev->dev;
	u16 config_cmd_val;
	int rc = 0;
	int i = 0;

	pr_info("ASPEED BMC PCI ID %04x:%04x, IRQ=%u\n", pdev->vendor, pdev->device, pdev->irq);

	rc = pci_enable_device(pdev);
	if (rc != 0) {
		dev_err(&pdev->dev, "pci_enable_device() returned error %d\n", rc);
		goto out_err;
	}

	/* set PCI host mastering  */
	pci_set_master(pdev);

#ifdef BMC_MSI_INT
#ifdef BMC_MULTI_MSI
	rc = pci_alloc_irq_vectors(pdev, 1, BMC_MULTI_MSI, PCI_IRQ_MSI);
#else
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
#endif
	if (rc < 0) {
		dev_err(&pdev->dev, "cannot allocate PCI MSI, rc=%d\n", rc);
		return rc;
	}
	pr_info("xx ASPEED BMC PCI ID %04x:%04x, IRQ=%u\n", pdev->vendor, pdev->device, pdev->irq);

	pci_read_config_word(pdev, PCI_COMMAND, &config_cmd_val);
	config_cmd_val |= PCI_COMMAND_INTX_DISABLE;
	pci_write_config_word((struct pci_dev *)pdev, PCI_COMMAND, config_cmd_val);
	pdev->irq = pci_irq_vector(pdev, BMC_MSI_IDX_BASE);
#else
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_LEGACY);
	if (rc < 0)
		dev_err(&pdev->dev, "cannot allocate PCI INTx, rc=%d\n", rc);

	pci_read_config_word(pdev, PCI_COMMAND, &config_cmd_val);
	config_cmd_val &= ~PCI_COMMAND_INTX_DISABLE;
	pci_write_config_word((struct pci_dev *)pdev, PCI_COMMAND, config_cmd_val);
#endif

	pci_bmc_dev = kzalloc(sizeof(*pci_bmc_dev), GFP_KERNEL);
	if (!pci_bmc_dev) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "kmalloc() returned NULL memory.\n");
		goto out_err;
	}

	//Get MEM bar
	pci_bmc_dev->mem_bar_base = pci_resource_start(pdev, 0);
	pci_bmc_dev->mem_bar_size = pci_resource_len(pdev, 0);

	pr_info("BAR0 I/O Mapped Base Address is: %08lx End %08lx\n", pci_bmc_dev->mem_bar_base, pci_bmc_dev->mem_bar_size);

	rc = pci_request_region(pdev, 0, dev_name(&pdev->dev));
	if (rc != 0)
		dev_err(&pdev->dev, "pci_request_region returned error %d\n", rc);

	pci_bmc_dev->mem_bar_reg = ioremap_nocache(pci_bmc_dev->mem_bar_base, pci_bmc_dev->mem_bar_size);
	if (!pci_bmc_dev->mem_bar_reg) {
		rc = -ENOMEM;
		goto out_free0;
	}

    //Get MSG BAR info
	pci_bmc_dev->message_bar_base = pci_resource_start(pdev, 1);
	pci_bmc_dev->message_bar_size = pci_resource_len(pdev, 1);

	pr_info("MSG BAR1 Memory Mapped Base Address is: %08lx End %08lx\n", pci_bmc_dev->message_bar_base, pci_bmc_dev->message_bar_size);

	pci_bmc_dev->msg_bar_reg = ioremap_nocache(pci_bmc_dev->message_bar_base, pci_bmc_dev->message_bar_size);
	if (!pci_bmc_dev->msg_bar_reg) {
		rc = -ENOMEM;
		goto out_free1;
	}

	/* ERRTA40: dummy read */
	(void)__raw_readl((void __iomem *)pci_bmc_dev->msg_bar_reg);

	sysfs_bin_attr_init(&pci_bmc_dev->bin0);
	sysfs_bin_attr_init(&pci_bmc_dev->bin1);

	pci_bmc_dev->bin0.attr.name = "pci-bmc-dev-queue1";
	pci_bmc_dev->bin0.attr.mode = 0600;
	pci_bmc_dev->bin0.read = aspeed_pci_bmc_dev_queue1_rx;
	pci_bmc_dev->bin0.write = aspeed_pci_bmc_dev_queue1_tx;
	pci_bmc_dev->bin0.size = 4;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &pci_bmc_dev->bin0);
	if (rc) {
		pr_err("error for bin file ");
		goto out_free1;
	}

	pci_bmc_dev->kn0 = kernfs_find_and_get(dev->kobj.sd, pci_bmc_dev->bin0.attr.name);
	if (!pci_bmc_dev->kn0) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin0);
		goto out_free1;
	}

	pci_bmc_dev->bin1.attr.name = "pci-bmc-dev-queue2";
	pci_bmc_dev->bin1.attr.mode = 0600;
	pci_bmc_dev->bin1.read = aspeed_pci_bmc_dev_queue2_rx;
	pci_bmc_dev->bin1.write = aspeed_pci_bmc_dev_queue2_tx;
	pci_bmc_dev->bin1.size = 4;

	rc = sysfs_create_bin_file(&pdev->dev.kobj, &pci_bmc_dev->bin1);
	if (rc) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin1);
		goto out_free1;
	}

	pci_bmc_dev->kn1 = kernfs_find_and_get(dev->kobj.sd, pci_bmc_dev->bin1.attr.name);
	if (!pci_bmc_dev->kn1) {
		sysfs_remove_bin_file(&dev->kobj, &pci_bmc_dev->bin1);
		goto out_free1;
	}

	pci_bmc_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pci_bmc_dev->miscdev.name = DRIVER_NAME;
	pci_bmc_dev->miscdev.fops = &aspeed_pci_bmc_dev_fops;
	pci_bmc_dev->miscdev.parent = dev;

	rc = misc_register(&pci_bmc_dev->miscdev);
	if (rc) {
		pr_err("host bmc register fail %d\n", rc);
		goto out_free;
	}

	pci_set_drvdata(pdev, pci_bmc_dev);

	rc = request_irq(pdev->irq, aspeed_pci_host_bmc_device_interrupt, IRQF_SHARED, "ASPEED BMC DEVICE", pci_bmc_dev);
	if (rc) {
		pr_err("host bmc device Unable to get IRQ %d\n", rc);
		goto out_unreg;
	}

#ifdef CONFIG_IPMI_SI
	/* hardcode for fix kcs_addr/kcs_sirq */
	kcs_ioport[0] = 0x3a0;
	kcs_ioport[1] = 0x3a8;
	kcs_ioport[2] = 0x3a2;
	kcs_ioport[3] = 0x3a4;
	kcs_sirq[0] = 0x10 + 0x2 - BMC_MSI_IDX_BASE;
	kcs_sirq[1] = 0x10 + 0x1 - BMC_MSI_IDX_BASE;
	kcs_sirq[2] = 0x10 + 0xc - BMC_MSI_IDX_BASE;
	kcs_sirq[3] = 0x10 + 0x7 - BMC_MSI_IDX_BASE;

	/* setup IPMI-KCS over PCIe */
//	memset(kcs_io, 0, sizeof(kcs_io));
	for (i = 0; i < KCS_MAX_PARMS; i++) {
		kcs_io[i].addr_source = SI_PCI;
		kcs_io[i].addr_source_data = pdev;
		kcs_io[i].si_type = SI_KCS;
		kcs_io[i].addr_space = IPMI_MEM_ADDR_SPACE;
		kcs_io[i].io_setup = ipmi_si_mem_setup;
		kcs_io[i].addr_data = pci_bmc_dev->message_bar_base + (kcs_ioport[i] << 2);
		kcs_io[i].dev = &pdev->dev;
		kcs_io[i].regspacing = 4;
		kcs_io[i].regsize = 1;
		kcs_io[i].regshift = 0;
#ifdef BMC_MULTI_MSI
		kcs_io[i].irq = pci_irq_vector(pdev, kcs_sirq[i]);
#else
		kcs_io[i].irq = pci_irq_vector(pdev, 0);
#endif
		if (kcs_io[i].irq)
			kcs_io[i].irq_setup = ipmi_std_irq_setup;

		rc = ipmi_si_add_smi(&kcs_io[i]);
		if (rc)
			dev_err(dev, "cannot setup IPMI-KCS@%xh over PCIe, rc=%d\n", kcs_ioport[i], rc);
	}
#endif
	/* setup VUART */
	memset(uart, 0, sizeof(uart));

	for (i = 0; i < VUART_MAX_PARMS; i++) {
		vuart_ioport[i] = 0x3F8 - (i * 0x100);
		vuart_sirq[i] = 0x10 + 4 - i - BMC_MSI_IDX_BASE;
		uart[i].port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
		uart[i].port.uartclk = 115200 * 16;

#ifdef BMC_MULTI_MSI
		uart[i].port.irq = pci_irq_vector(pdev, vuart_sirq[i]);
#else
		uart[i].port.irq = pci_irq_vector(pdev, 0);
#endif
		uart[i].port.dev = &pdev->dev;

		uart[i].port.iotype = UPIO_MEM32;
		uart[i].port.iobase = 0;
		uart[i].port.mapbase = pci_bmc_dev->message_bar_base + (vuart_ioport[i] << 2);
		uart[i].port.membase = 0;
		uart[i].port.type = PORT_16550A;
		uart[i].port.flags |= (UPF_IOREMAP | UPF_FIXED_PORT | UPF_FIXED_TYPE);
		uart[i].port.regshift = 2;

		rc = serial8250_register_8250_port(&uart[i]);
		if (rc < 0) {
			dev_err(dev, "cannot setup VUART@%xh over PCIe, rc=%d\n", vuart_ioport[i], rc);
			goto out_unreg;
		}
	}

	return 0;

out_unreg:
	misc_deregister(&pci_bmc_dev->miscdev);
out_free1:
	pci_release_region(pdev, 1);
out_free0:
	pci_release_region(pdev, 0);
out_free:
	kfree(pci_bmc_dev);
out_err:
	pci_disable_device(pdev);

	return rc;

}

static void aspeed_pci_host_bmc_device_remove(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	free_irq(pdev->irq, pdev);
	misc_deregister(&pci_bmc_dev->miscdev);
	pci_release_regions(pdev);
	kfree(pci_bmc_dev);
	pci_disable_device(pdev);
}

/**
 * This table holds the list of (VendorID,DeviceID) supported by this driver
 *
 */
static struct pci_device_id aspeed_host_bmc_dev_pci_ids[] = {
	{ PCI_DEVICE(0x1A03, 0x2402), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, aspeed_host_bmc_dev_pci_ids);

static struct pci_driver aspeed_host_bmc_dev_driver = {
	.name		= DRIVER_NAME,
	.id_table	= aspeed_host_bmc_dev_pci_ids,
	.probe		= aspeed_pci_host_bmc_device_probe,
	.remove		= aspeed_pci_host_bmc_device_remove,
};

static int __init aspeed_host_bmc_device_init(void)
{
	int ret;

	/* register pci driver */
	ret = pci_register_driver(&aspeed_host_bmc_dev_driver);
	if (ret < 0) {
		pr_err("pci-driver: can't register pci driver\n");
		return ret;
	}

	return 0;

}

static void aspeed_host_bmc_device_exit(void)
{
	/* unregister pci driver */
	pci_unregister_driver(&aspeed_host_bmc_dev_driver);
}

late_initcall(aspeed_host_bmc_device_init);
module_exit(aspeed_host_bmc_device_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Host BMC DEVICE Driver");
MODULE_LICENSE("GPL");
