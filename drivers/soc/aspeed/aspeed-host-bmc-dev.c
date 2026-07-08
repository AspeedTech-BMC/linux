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
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/poll.h>
#include <linux/bitfield.h>

#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/mctp.h>
#include <net/mctp.h>
#include <net/pkt_sched.h>
#if IS_ENABLED(CONFIG_ASPEED_MMBI)
#include "mmbi/aspeed-mmbi.h"
#endif

#define PCI_BMC_HOST2BMC_Q1		0x30000
#define PCI_BMC_HOST2BMC_Q2		0x30010
#define PCI_BMC_BMC2HOST_Q1		0x30020
#define PCI_BMC_BMC2HOST_Q2		0x30030
#define PCI_BMC_BMC2HOST_STS		0x30040
#define	 BMC2HOST_INT_STS_DOORBELL	BIT(31)
#define	 BMC2HOST_ENABLE_INTB		BIT(30)

#define	 BMC2HOST_Q1_FULL		BIT(27)
#define	 BMC2HOST_Q1_EMPTY		BIT(26)
#define	 BMC2HOST_Q2_FULL		BIT(25)
#define	 BMC2HOST_Q2_EMPTY		BIT(24)
#define	 BMC2HOST_Q1_FULL_UNMASK	BIT(23)
#define	 BMC2HOST_Q1_EMPTY_UNMASK	BIT(22)
#define	 BMC2HOST_Q2_FULL_UNMASK	BIT(21)
#define	 BMC2HOST_Q2_EMPTY_UNMASK	BIT(20)

#define PCI_BMC_HOST2BMC_STS		0x30044
#define	 HOST2BMC_INT_STS_DOORBELL	BIT(31)
#define	 HOST2BMC_ENABLE_INTB		BIT(30)

#define	 HOST2BMC_Q1_FULL		BIT(27)
#define	 HOST2BMC_Q1_EMPTY		BIT(26)
#define	 HOST2BMC_Q2_FULL		BIT(25)
#define	 HOST2BMC_Q2_EMPTY		BIT(24)
#define	 HOST2BMC_Q1_FULL_UNMASK	BIT(23)
#define	 HOST2BMC_Q1_EMPTY_UNMASK	BIT(22)
#define	 HOST2BMC_Q2_FULL_UNMASK	BIT(21)
#define	 HOST2BMC_Q2_EMPTY_UNMASK	BIT(20)

static DEFINE_IDA(bmc_device_ida);

#define VUART_MAX_PARMS		2
#define ASPEED_QUEUE_NUM	2
#define MAX_MSI_NUM		8

enum queue_index {
	QUEUE1 = 0,
	QUEUE2,
};

enum msi_index {
	BMC_MSI,
	MBX_MSI,
	VUART0_MSI,
	VUART1_MSI,
};

/* Match msi_index */
static int ast2600_msi_idx_table[MAX_MSI_NUM] = { 4, 21, 16, 15 };
static int ast2700_soc0_msi_idx_table[MAX_MSI_NUM] = { 0, 11, 6, 5};

struct aspeed_platform {
	int (*setup)(struct pci_dev *pdev);
};

struct aspeed_queue_message {
	/* Queue waiters for idle engine */
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	struct kernfs_node *kn;
	struct bin_attribute bin;
	int index;
	struct aspeed_pci_bmc_dev *pci_bmc_device;
};

#if IS_ENABLED(CONFIG_ASPEED_MMBI)
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

/*
 * app_interface: per-channel application interface selection.
 *   0 = ioctl  (MMBI_APP_INTF_IOCTL)
 *   1 = mctp-netdev  (MMBI_APP_INTF_MCTP_NETDEV)
 *
 * One value applies to every channel. Multiple values map by channel index,
 * and omitted channels keep their defaults.
 * Examples:
 *   app_interface=1          -> all channels use mctp-netdev
 *   app_interface=1,0,0,0,1  -> ch0/ch4 use mctp-netdev
 * Invalid values fall back to the default map.
 */
static const uint app_interface_defaults[MMBI_MAX_CHANNELS] = {
	[0] = MMBI_APP_INTF_MCTP_NETDEV,
};

static uint app_interface[MMBI_MAX_CHANNELS] = { [0] = MMBI_APP_INTF_MCTP_NETDEV };
static int app_interface_count;
module_param_array(app_interface, uint, &app_interface_count, 0444);
MODULE_PARM_DESC(app_interface,
		 "App interface per channel: 0=ioctl, 1=mctp-netdev; one value applies to all channels");

#endif

struct aspeed_pci_bmc_dev {
	struct device *dev;
	struct miscdevice miscdev;
	struct aspeed_platform *platform;
	kernel_ulong_t driver_data;
	int id;

	unsigned long mem_bar_base;
	unsigned long mem_bar_size;
	void __iomem *mem_bar_reg;

	unsigned long message_bar_base;
	unsigned long message_bar_size;
	void __iomem *msg_bar_reg;

	void __iomem *pcie_sio_decode_addr;

	struct aspeed_queue_message queue[ASPEED_QUEUE_NUM];

	void __iomem *sio_mbox_reg;
	struct uart_8250_port uart[VUART_MAX_PARMS];
	int uart_line[VUART_MAX_PARMS];

	/* Interrupt
	 * The index of array is using to enum msi_index
	 */
	int *msi_idx_table;
#if IS_ENABLED(CONFIG_ASPEED_MMBI)
	struct aspeed_host_pcie_mmbi mmbi;
#endif
};

#define PCIE_DEVICE_SIO_ADDR	(0x2E * 4)
#define BMC_MULTI_MSI		32

#define DRIVER_NAME "aspeed-host-bmc-dev"

static struct aspeed_pci_bmc_dev *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_pci_bmc_dev, miscdev);
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

static ssize_t aspeed_queue_rx(struct file *filp, struct kobject *kobj,
			       const struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_pci_bmc_dev *pci_bmc_device = queue->pci_bmc_device;
	int index = queue->index;
	u32 *data = (u32 *)buf;
	int ret;

	ret = wait_event_interruptible(queue->rx_wait,
				       !(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) &
				       ((index == QUEUE1) ? BMC2HOST_Q1_EMPTY : BMC2HOST_Q2_EMPTY)));
	if (ret)
		return -EINTR;

	data[0] = readl(pci_bmc_device->msg_bar_reg +
			((index == QUEUE1) ? PCI_BMC_BMC2HOST_Q1 : PCI_BMC_BMC2HOST_Q2));

	writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB,
	       pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS);

	return sizeof(u32);
}

static ssize_t aspeed_queue_tx(struct file *filp, struct kobject *kobj,
			       const struct bin_attribute *attr,
			       char *buf, loff_t off, size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_pci_bmc_dev *pci_bmc_device = queue->pci_bmc_device;
	int index = queue->index;
	u32 tx_buff;
	int ret;

	if (count != sizeof(u32))
		return -EINVAL;

	ret = wait_event_interruptible(queue->tx_wait,
				       !(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) &
				       ((index == QUEUE1) ? HOST2BMC_Q1_FULL : HOST2BMC_Q2_FULL)));
	if (ret)
		return -EINTR;

	memcpy(&tx_buff, buf, 4);
	writel(tx_buff, pci_bmc_device->msg_bar_reg +
				((index == QUEUE1) ? PCI_BMC_HOST2BMC_Q1 : PCI_BMC_HOST2BMC_Q2));
	//trigger to host
	writel(HOST2BMC_INT_STS_DOORBELL | HOST2BMC_ENABLE_INTB,
	       pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS);

	return sizeof(u32);
}

static irqreturn_t aspeed_pci_host_bmc_device_interrupt(int irq, void *dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;
	u32 bmc2host_q_sts = readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_INT_STS_DOORBELL)
		writel(BMC2HOST_INT_STS_DOORBELL,
		       pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_ENABLE_INTB)
		writel(BMC2HOST_ENABLE_INTB, pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS);

	if (bmc2host_q_sts & BMC2HOST_Q1_FULL)
		dev_info(pci_bmc_device->dev, "Q1 Full\n");

	if (bmc2host_q_sts & BMC2HOST_Q2_FULL)
		dev_info(pci_bmc_device->dev, "Q2 Full\n");

	//check q1
	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q1_FULL))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE1].tx_wait);

	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q1_EMPTY))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE1].rx_wait);
	//chech q2
	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_HOST2BMC_STS) & HOST2BMC_Q2_FULL))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE2].tx_wait);

	if (!(readl(pci_bmc_device->msg_bar_reg + PCI_BMC_BMC2HOST_STS) & BMC2HOST_Q2_EMPTY))
		wake_up_interruptible(&pci_bmc_device->queue[QUEUE2].rx_wait);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_pci_host_mbox_interrupt(int irq, void *dev_id)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = dev_id;
	u32 isr = readl(pci_bmc_device->sio_mbox_reg + 0x94);

	if (isr & BIT(7))
		writel(BIT(7), pci_bmc_device->sio_mbox_reg + 0x94);

	return IRQ_HANDLED;
}

static void aspeed_pci_setup_irq_resource(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	/* Assign static msi index table by platform */
	if (pdev->revision == 0x27) {
		pci_bmc_dev->msi_idx_table = ast2700_soc0_msi_idx_table;
	} else {
		pci_bmc_dev->msi_idx_table = ast2600_msi_idx_table;
	}

	if (pci_alloc_irq_vectors(pdev, 1, BMC_MULTI_MSI, PCI_IRQ_INTX | PCI_IRQ_MSI) <= 1)
		/* Set all msi index to the first vector */
		memset(pci_bmc_dev->msi_idx_table, 0, sizeof(int) * MAX_MSI_NUM);
}

static int aspeed_pci_bmc_device_setup_queue(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_device = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret, i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++) {
		struct aspeed_queue_message *queue = &pci_bmc_device->queue[i];

		init_waitqueue_head(&queue->tx_wait);
		init_waitqueue_head(&queue->rx_wait);

		sysfs_bin_attr_init(&queue->bin);

		/* Queue name index starts from 1 */
		queue->bin.attr.name =
			devm_kasprintf(dev, GFP_KERNEL, "pci-bmc-dev-queue%d", (i + 1));
		queue->bin.attr.mode = 0600;
		queue->bin.read = aspeed_queue_rx;
		queue->bin.write = aspeed_queue_tx;
		queue->bin.size = 4;
		queue->bin.private = queue;

		ret = sysfs_create_bin_file(&pdev->dev.kobj, &queue->bin);
		if (ret) {
			dev_err(dev, "error for bin%d file\n", i);
			return ret;
		}

		queue->kn = kernfs_find_and_get(dev->kobj.sd, queue->bin.attr.name);
		if (!queue->kn) {
			sysfs_remove_bin_file(&dev->kobj, &queue->bin);
			return ret;
		}

		queue->index = i;
		queue->pci_bmc_device = pci_bmc_device;
	}

	return 0;
}

static int aspeed_pci_bmc_device_setup_vuart(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u16 vuart_ioport;
	int ret, i;

	for (i = 0; i < VUART_MAX_PARMS; i++) {
		/* Assign the line to non-exist device */
		pci_bmc_dev->uart_line[i] = -ENOENT;
		vuart_ioport = 0x3F8 - (i * 0x100);
		pci_bmc_dev->uart[i].port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
		pci_bmc_dev->uart[i].port.uartclk = 115200 * 16;
		pci_bmc_dev->uart[i].port.irq =
			pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[VUART0_MSI + i]);
		pci_bmc_dev->uart[i].port.dev = dev;
		pci_bmc_dev->uart[i].port.iotype = UPIO_MEM32;
		pci_bmc_dev->uart[i].port.iobase = 0;
		pci_bmc_dev->uart[i].port.mapbase =
			pci_bmc_dev->message_bar_base + (vuart_ioport << 2);
		pci_bmc_dev->uart[i].port.membase = 0;
		pci_bmc_dev->uart[i].port.type = PORT_16550A;
		pci_bmc_dev->uart[i].port.flags |= (UPF_IOREMAP | UPF_FIXED_PORT | UPF_FIXED_TYPE);
		pci_bmc_dev->uart[i].port.regshift = 2;
		ret = serial8250_register_8250_port(&pci_bmc_dev->uart[i]);
		if (ret < 0) {
			dev_err_probe(dev, ret, "Can't setup PCIe VUART\n");
			return ret;
		}
		pci_bmc_dev->uart_line[i] = ret;
	}
	return 0;
}

static int aspeed_pci_bmc_device_setup_memory_mapping(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	pci_bmc_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pci_bmc_dev->miscdev.name =
		devm_kasprintf(dev, GFP_KERNEL, "%s%d", DRIVER_NAME, pci_bmc_dev->id);
	pci_bmc_dev->miscdev.fops = &aspeed_pci_bmc_dev_fops;
	pci_bmc_dev->miscdev.parent = dev;

	ret = misc_register(&pci_bmc_dev->miscdev);
	if (ret) {
		pr_err("host bmc register fail %d\n", ret);
		return ret;
	}

	return 0;
}

static int aspeed_pci_bmc_device_setup_mbox(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	/* setup mbox */
	pci_bmc_dev->pcie_sio_decode_addr = pci_bmc_dev->msg_bar_reg + PCIE_DEVICE_SIO_ADDR;
	writel(0xaa, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0xa5, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0xa5, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x07, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x0e, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* disable */
	writel(0x30, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x00, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* set decode address 0x100 */
	writel(0x60, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x01, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	writel(0x61, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x00, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	/* enable */
	writel(0x30, pci_bmc_dev->pcie_sio_decode_addr);
	writel(0x01, pci_bmc_dev->pcie_sio_decode_addr + 0x04);
	pci_bmc_dev->sio_mbox_reg = pci_bmc_dev->msg_bar_reg + 0x400;

	ret = devm_request_irq(dev,
			       pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
			       aspeed_pci_host_mbox_interrupt, IRQF_SHARED,
			       devm_kasprintf(dev, GFP_KERNEL, "aspeed-sio-mbox%d", pci_bmc_dev->id),
			       pci_bmc_dev);
	if (ret) {
		pr_err("host bmc device Unable to get IRQ %d\n", ret);
		return ret;
	}

	return 0;
}

static void aspeed_pci_host_bmc_device_release_queue(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++)
		sysfs_remove_bin_file(&pdev->dev.kobj, &pci_bmc_dev->queue[i].bin);
}

static void aspeed_pci_host_bmc_device_release_vuart(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int i;

	for (i = 0; i < VUART_MAX_PARMS; i++) {
		if (pci_bmc_dev->uart_line[i] >= 0)
			serial8250_unregister_port(pci_bmc_dev->uart_line[i]);
	}
}

static void aspeed_pci_host_bmc_device_release_memory_mapping(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);

	if (!list_empty(&pci_bmc_dev->miscdev.list))
		misc_deregister(&pci_bmc_dev->miscdev);
}

static int aspeed_pci_host_setup(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	int rc = 0;

	/* Get share memory BAR */
	pci_bmc_dev->mem_bar_base = pci_resource_start(pdev, 0);
	pci_bmc_dev->mem_bar_size = pci_resource_len(pdev, 0);
	pci_bmc_dev->mem_bar_reg = pci_ioremap_bar(pdev, 0);
	if (!pci_bmc_dev->mem_bar_reg)
		return -ENOMEM;

	/* Get Message BAR */
	pci_bmc_dev->message_bar_base = pci_resource_start(pdev, 1);
	pci_bmc_dev->message_bar_size = pci_resource_len(pdev, 1);
	pci_bmc_dev->msg_bar_reg = pci_ioremap_bar(pdev, 1);
	if (!pci_bmc_dev->msg_bar_reg) {
		rc = -ENOMEM;
		goto out_free0;
	}

	/* AST2600 ERRTA40: dummy read */
	if (pdev->revision < 0x27)
		(void)__raw_readl((void __iomem *)pci_bmc_dev->msg_bar_reg);

	rc = aspeed_pci_bmc_device_setup_queue(pdev);
	if (rc) {
		pr_err("Cannot setup Queue Message");
		goto out_free1;
	}

	rc = aspeed_pci_bmc_device_setup_memory_mapping(pdev);
	if (rc) {
		pr_err("Cannot setup Memory Mapping");
		goto out_free_queue;
	}

	rc = aspeed_pci_bmc_device_setup_mbox(pdev);
	if (rc) {
		pr_err("Cannot setup Mailnbox");
		goto out_free_mmapping;
	}

	rc = aspeed_pci_bmc_device_setup_vuart(pdev);
	if (rc) {
		pr_err("Cannot setup Virtual UART");
		goto out_free_uart;
	}

	rc = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[BMC_MSI]),
			      aspeed_pci_host_bmc_device_interrupt, IRQF_SHARED,
			      pci_bmc_dev->miscdev.name, pci_bmc_dev);
	if (rc) {
		pr_err("Get BMC DEVICE IRQ failed. (err=%d)\n", rc);
		goto out_free_uart;
	}

	return 0;

out_free_uart:
	aspeed_pci_host_bmc_device_release_vuart(pdev);
	devm_free_irq(&pdev->dev, pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
		      pci_bmc_dev);
out_free_mmapping:
	aspeed_pci_host_bmc_device_release_memory_mapping(pdev);
out_free_queue:
	aspeed_pci_host_bmc_device_release_queue(pdev);
out_free1:
	pci_iounmap(pdev, pci_bmc_dev->msg_bar_reg);
out_free0:
	pci_iounmap(pdev, pci_bmc_dev->mem_bar_reg);

	pci_release_regions(pdev);
	return rc;
}

static struct aspeed_platform aspeed_pcie_host[] = {
	{ .setup = aspeed_pci_host_setup },
	{ 0 }
};

#if IS_ENABLED(CONFIG_ASPEED_MMBI)
static const char *
aspeed_host_pcie_mmbi_app_interface_name(enum mmbi_app_interface intf)
{
	switch (intf) {
	case MMBI_APP_INTF_MCTP_NETDEV:
		return "mctp-netdev";
	case MMBI_APP_INTF_IOCTL:
	default:
		return "ioctl";
	}
}

static int aspeed_host_pcie_mmbi_validate_app_interface(struct device *dev)
{
	int i;

	for (i = 0; i < app_interface_count; i++) {
		if (app_interface[i] > MMBI_APP_INTF_MCTP_NETDEV) {
			dev_warn(dev,
				 "invalid app_interface[%d]=%u, using defaults\n",
				 i, app_interface[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static enum mmbi_app_interface
aspeed_host_pcie_mmbi_get_app_interface(int channel, bool use_defaults)
{
	if (use_defaults)
		return app_interface_defaults[channel];

	if (app_interface_count == 1)
		return app_interface[0];

	if (channel < app_interface_count)
		return app_interface[channel];

	return app_interface_defaults[channel];
}

static void
aspeed_host_pcie_mmbi_init_app_interfaces(struct mmbi_ins_desc *mmbi_desc,
					  bool use_defaults)
{
	int i;

	for (i = 0; i < MMBI_MAX_CHANNELS; i++)
		mmbi_desc->chan_desc[i].app_interface =
			aspeed_host_pcie_mmbi_get_app_interface(i,
								use_defaults);
}

static void aspeed_host_pcie_mmbi_log_app_interfaces(struct device *dev,
						     struct mmbi_ins_desc *mmbi_desc,
						     int bar)
{
	struct mmbi_chan_desc *chan_desc;
	const char *interface;
	int i;

	for (i = 0; i < mmbi_desc->num_of_channels; i++) {
		chan_desc = &mmbi_desc->chan_desc[i];
		interface = aspeed_host_pcie_mmbi_app_interface_name(chan_desc->app_interface);
		dev_info(dev, "BAR %d channel %d application interface: %s\n",
			 bar, i, interface);
	}
}

static int aspeed_pci_host_mmbi_setup(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
	struct aspeed_host_pcie_mmbi *mmbi = &pci_bmc_dev->mmbi;
	struct aspeed_host_pcie_mmbi_ins *ins;
	struct device *dev = &pdev->dev;
	struct mmbi_ins_desc *mmbi_desc;
	int i, ret, instance_id;
	int last_ret = -ENODEV;
	bool use_default_app_interfaces;
	resource_size_t start, size;

	ret = aspeed_host_pcie_mmbi_validate_app_interface(dev);
	use_default_app_interfaces = ret != 0;

	instance_id = 0;
	for (i = 2; i < PCI_STD_NUM_BARS; i++) {
		ins = &mmbi->instances[i];
		mmbi_desc = &ins->mmbi_desc;
		start = pci_resource_start(pdev, i);
		size = pci_resource_len(pdev, i);

		if (!start || !size) {
			dev_dbg(dev,
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

		aspeed_host_pcie_mmbi_init_app_interfaces(mmbi_desc,
							  use_default_app_interfaces);
		ret = mmbi_instance_init(mmbi_desc);
		if (ret) {
			dev_info(dev,
				 "Failed to initialize MMBI at bar %d, error %d\n",
				 i, ret);
			pci_iounmap(pdev, mmbi_desc->desc_virt);
			last_ret = ret;
		} else {
			ins->instance_index = instance_id++;
			ins->bar_index = i;
			ins->valid = true;
			ins->dev_name = devm_kasprintf(dev, GFP_KERNEL,
						       "mmbi_host_ins%d",
						       ins->instance_index);
			aspeed_host_pcie_mmbi_log_app_interfaces(dev, mmbi_desc, i);

			ins->msi_index = mmbi_desc->host_int_location;
		}
	}

	if (!instance_id)
		return last_ret;

	return 0;
}
#endif

static int aspeed_pci_host_bmc_device_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev;
	int rc = 0;

	pr_info("ASPEED BMC PCI ID %04x:%04x, IRQ=%u\n", pdev->vendor, pdev->device, pdev->irq);

	pci_bmc_dev = devm_kzalloc(&pdev->dev, sizeof(*pci_bmc_dev), GFP_KERNEL);
	if (!pci_bmc_dev)
		return -ENOMEM;

	/* Get platform id */
	pci_bmc_dev->driver_data = ent->driver_data;
	pci_bmc_dev->platform = &aspeed_pcie_host[ent->driver_data];

	pci_bmc_dev->id = ida_alloc(&bmc_device_ida, GFP_KERNEL);
	if (pci_bmc_dev->id < 0)
		return pci_bmc_dev->id;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "pci_enable_device() returned error %d\n", rc);
		goto out_free_ida;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, pci_bmc_dev);

	/* Prepare IRQ resource */
	aspeed_pci_setup_irq_resource(pdev);

	/* Setup BMC PCI device */
	rc = pci_bmc_dev->platform->setup(pdev);
	if (rc) {
		dev_err(&pdev->dev, "ASPEED PCIe Host device returned error %d\n", rc);
		goto out_free_irq_vectors;
	}

#if IS_ENABLED(CONFIG_ASPEED_MMBI)
	pci_bmc_dev->mmbi.dev = &pdev->dev;
	if (aspeed_pci_host_mmbi_setup(pdev))
		dev_warn(&pdev->dev, "MMBI not available, continuing without it\n");
#endif

	return 0;

out_free_irq_vectors:
	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
out_free_ida:
	ida_free(&bmc_device_ida, pci_bmc_dev->id);
	return rc;
}

static void aspeed_pci_host_bmc_device_remove(struct pci_dev *pdev)
{
	struct aspeed_pci_bmc_dev *pci_bmc_dev = pci_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_ASPEED_MMBI)
	struct aspeed_host_pcie_mmbi *mmbi = &pci_bmc_dev->mmbi;
	struct aspeed_host_pcie_mmbi_ins *ins;
	int i;

	for (i = 2; i < PCI_STD_NUM_BARS; i++) {
		ins = &mmbi->instances[i];
		if (ins->valid) {
			mmbi_instance_remove(&ins->mmbi_desc);
			pci_iounmap(pdev, ins->mmbi_desc.desc_virt);
		}
	}
#endif
	aspeed_pci_host_bmc_device_release_queue(pdev);
	aspeed_pci_host_bmc_device_release_memory_mapping(pdev);
	aspeed_pci_host_bmc_device_release_vuart(pdev);

	devm_free_irq(&pdev->dev,
		      pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[BMC_MSI]),
		      pci_bmc_dev);
	devm_free_irq(&pdev->dev,
		      pci_irq_vector(pdev, pci_bmc_dev->msi_idx_table[MBX_MSI]),
		      pci_bmc_dev);

	ida_free(&bmc_device_ida, pci_bmc_dev->id);

	pci_iounmap(pdev, pci_bmc_dev->msg_bar_reg);
	pci_iounmap(pdev, pci_bmc_dev->mem_bar_reg);

	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/**
 * This table holds the list of (VendorID,DeviceID) supported by this driver
 *
 */
static struct pci_device_id aspeed_host_bmc_dev_pci_ids[] = {
	{ PCI_DEVICE(0x1A03, 0x2402), .class = 0xFF0000, .class_mask = 0xFFFF00 },
	{ PCI_DEVICE(0x1A03, 0x2402), .class = 0x0C0C00, .class_mask = 0xFFFF00 },
	{
		0,
	}
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
	return pci_register_driver(&aspeed_host_bmc_dev_driver);
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
