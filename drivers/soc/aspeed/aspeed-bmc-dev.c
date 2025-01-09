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

static DEFINE_IDA(bmc_device_ida);

#define SCU_TRIGGER_MSI

/* AST2600 SCU */
#define ASPEED_SCU04			0x04
#define AST2600A3_SCU04				0x05030303
#define ASPEED_SCUC20			0xC20
#define ASPEED_SCUC24			0xC24
#define MSI_ROUTING_MASK			GENMASK(11, 10)
#define PCIDEV1_INTX_MSI_HOST2BMC_EN		BIT(18)
#define MSI_ROUTING_PCIe2LPC_PCIDEV0		(0x1 << 10)
#define MSI_ROUTING_PCIe2LPC_PCIDEV1		(0x2 << 10)
/* AST2700 SCU */
#define SCU0_REVISION_ID		0x0
#define REVISION_ID				GENMASK(23, 16)
/* Host2BMC */
#define ASPEED_BMC_MEM_BAR			0xF10
#define  PCIE2PCI_MEM_BAR_ENABLE		BIT(1)
#define  HOST2BMC_MEM_BAR_ENABLE		BIT(0)
#define ASPEED_BMC_MEM_BAR_REMAP	0xF18

#define ASPEED_BMC_SHADOW_CTRL		0xF50
#define  READ_ONLY_MASK				BIT(31)
#define  MASK_BAR1					BIT(2)
#define  MASK_BAR0					BIT(1)
#define  SHADOW_CFG					BIT(0)

#define ASPEED_BMC_HOST2BMC_Q1		0xA000
#define ASPEED_BMC_HOST2BMC_Q2		0xA010
#define ASPEED_BMC_BMC2HOST_Q1		0xA020
#define ASPEED_BMC_BMC2HOST_Q2		0xA030
#define ASPEED_BMC_BMC2HOST_STS		0xA040
#define	 BMC2HOST_INT_STS_DOORBELL		BIT(31)
#define	 BMC2HOST_ENABLE_INTB			BIT(30)
#define	 BMC2HOST_Q1_FULL				BIT(27)
#define	 BMC2HOST_Q1_EMPTY				BIT(26)
#define	 BMC2HOST_Q2_FULL				BIT(25)
#define	 BMC2HOST_Q2_EMPTY				BIT(24)
#define	 BMC2HOST_Q1_FULL_UNMASK		BIT(23)
#define	 BMC2HOST_Q1_EMPTY_UNMASK		BIT(22)
#define	 BMC2HOST_Q2_FULL_UNMASK		BIT(21)
#define	 BMC2HOST_Q2_EMPTY_UNMASK		BIT(20)

#define ASPEED_BMC_HOST2BMC_STS		0xA044
#define	 HOST2BMC_INT_STS_DOORBELL		BIT(31)
#define	 HOST2BMC_ENABLE_INTB			BIT(30)
#define	 HOST2BMC_Q1_FULL				BIT(27)
#define	 HOST2BMC_Q1_EMPTY				BIT(26)
#define	 HOST2BMC_Q2_FULL				BIT(25)
#define	 HOST2BMC_Q2_EMPTY				BIT(24)
#define	 HOST2BMC_Q1_FULL_UNMASK		BIT(23)
#define	 HOST2BMC_Q1_EMPTY_UNMASK		BIT(22)
#define	 HOST2BMC_Q2_FULL_UNMASK		BIT(21)
#define	 HOST2BMC_Q2_EMPTY_UNMASK		BIT(20)

#define ASPEED_SCU_PCIE_CONF_CTRL	0xC20
#define  SCU_PCIE_CONF_BMC_DEV_EN			 BIT(8)
#define  SCU_PCIE_CONF_BMC_DEV_EN_MMIO		 BIT(9)
#define  SCU_PCIE_CONF_BMC_DEV_EN_MSI		 BIT(11)
#define  SCU_PCIE_CONF_BMC_DEV_EN_IRQ		 BIT(13)
#define  SCU_PCIE_CONF_BMC_DEV_EN_DMA		 BIT(14)
#define  SCU_PCIE_CONF_BMC_DEV_EN_E2L		 BIT(15)
#define  SCU_PCIE_CONF_BMC_DEV_EN_LPC_DECODE BIT(21)

#define ASPEED_SCU_BMC_DEV_CLASS	0xC68

#define ASPEED_QUEUE_NUM 2
enum queue_index {
	QUEUE1 = 0,
	QUEUE2,
};

struct aspeed_platform {
	int (*init)(struct platform_device *pdev);
	ssize_t (*queue_rx)(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
			    char *buf, loff_t off, size_t count);
	ssize_t (*queue_tx)(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
			    char *buf, loff_t off, size_t count);
};

struct aspeed_queue_message {
	/* Queue waiters for idle engine */
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	struct kernfs_node *kn;
	struct bin_attribute bin;
	int index;
	struct aspeed_bmc_device *bmc_device;
};

struct aspeed_bmc_device {
	unsigned char *host2bmc_base_virt;
	struct device *dev;
	struct miscdevice miscdev;
	int id;
	void __iomem *reg_base;
	void __iomem *bmc_mem_virt;
	dma_addr_t bmc_mem_phy;
	phys_addr_t bmc_mem_size;

	int pcie2lpc;
	int irq;

	struct aspeed_queue_message queue[ASPEED_QUEUE_NUM];

	const struct aspeed_platform *platform;

	/* AST2700 */
	struct regmap *config;
	struct regmap *device;
	struct regmap *e2m;
	/*AST2600*/
	struct regmap *scu;
	int pcie_irq;
};

static struct aspeed_bmc_device *file_aspeed_bmc_device(struct file *file)
{
	return container_of(file->private_data, struct aspeed_bmc_device,
			miscdev);
}

static int aspeed_bmc_device_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct aspeed_bmc_device *bmc_device = file_aspeed_bmc_device(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	pgprot_t prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vsize) > bmc_device->bmc_mem_size)
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (bmc_device->bmc_mem_phy >> PAGE_SHIFT) + vma->vm_pgoff, vsize, prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations aspeed_bmc_device_fops = {
	.owner		= THIS_MODULE,
	.mmap		= aspeed_bmc_device_mmap,
};

static ssize_t aspeed_ast2600_queue_rx(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *attr, char *buf, loff_t off,
				       size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_bmc_device *bmc_device = queue->bmc_device;
	int index = queue->index;
	u32 *data = (u32 *)buf;
	u32 scu_id;
	int ret;

	ret = wait_event_interruptible(queue->rx_wait,
				       !(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) &
				       ((index == QUEUE1) ? HOST2BMC_Q1_EMPTY : HOST2BMC_Q2_EMPTY)));
	if (ret)
		return -EINTR;

	data[0] = readl(bmc_device->reg_base +
			((index == QUEUE1) ? ASPEED_BMC_HOST2BMC_Q1 : ASPEED_BMC_HOST2BMC_Q2));

	regmap_read(bmc_device->scu, ASPEED_SCU04, &scu_id);
	if (scu_id == AST2600A3_SCU04) {
		writel(BMC2HOST_INT_STS_DOORBELL | BMC2HOST_ENABLE_INTB,
		       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	} else {
		//A0 : BIT(12) A1 : BIT(15)
		regmap_update_bits(bmc_device->scu, 0x560, BIT(15), BIT(15));
		regmap_update_bits(bmc_device->scu, 0x560, BIT(15), 0);
	}

	return sizeof(u32);
}

static ssize_t aspeed_ast2600_queue_tx(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *attr, char *buf, loff_t off,
				       size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_bmc_device *bmc_device = queue->bmc_device;
	int index = queue->index;
	u32 tx_buff;
	u32 scu_id;
	int ret;

	if (count != sizeof(u32))
		return -EINVAL;

	ret = wait_event_interruptible(queue->tx_wait,
				       !(readl(bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS) &
				       ((index == QUEUE1) ? BMC2HOST_Q1_FULL : BMC2HOST_Q2_FULL)));
	if (ret)
		return -EINTR;

	memcpy(&tx_buff, buf, 4);
	writel(tx_buff, bmc_device->reg_base + ((index == QUEUE1) ? ASPEED_BMC_BMC2HOST_Q1 :
								    ASPEED_BMC_BMC2HOST_Q2));

	/* trigger to host
	 * Only After AST2600A3 support DoorBell MSI
	 */
	regmap_read(bmc_device->scu, ASPEED_SCU04, &scu_id);
	if (scu_id == AST2600A3_SCU04) {
		writel(BMC2HOST_INT_STS_DOORBELL | BMC2HOST_ENABLE_INTB,
		       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	} else {
		//A0 : BIT(12) A1 : BIT(15)
		regmap_update_bits(bmc_device->scu, 0x560, BIT(15), BIT(15));
		regmap_update_bits(bmc_device->scu, 0x560, BIT(15), 0);
	}

	return sizeof(u32);
}

static ssize_t aspeed_ast2700_queue_rx(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *attr, char *buf, loff_t off,
				       size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_bmc_device *bmc_device = queue->bmc_device;
	int index = queue->index;
	u32 *data = (u32 *)buf;
	int ret;

	ret = wait_event_interruptible(queue->rx_wait,
				       !(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) &
				       ((index == QUEUE1) ? HOST2BMC_Q1_EMPTY : HOST2BMC_Q2_EMPTY)));
	if (ret)
		return -EINTR;

	data[0] = readl(bmc_device->reg_base +
			((index == QUEUE1) ? ASPEED_BMC_HOST2BMC_Q1 : ASPEED_BMC_HOST2BMC_Q2));

	writel(BMC2HOST_INT_STS_DOORBELL | BMC2HOST_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);

	return sizeof(u32);
}

static ssize_t aspeed_ast2700_queue_tx(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *attr, char *buf, loff_t off,
				       size_t count)
{
	struct aspeed_queue_message *queue = attr->private;
	struct aspeed_bmc_device *bmc_device = queue->bmc_device;
	int index = queue->index;
	u32 tx_buff;
	int ret;

	if (count != sizeof(u32))
		return -EINVAL;

	ret = wait_event_interruptible(queue->tx_wait,
				       !(readl(bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS) &
				       ((index == QUEUE1) ? BMC2HOST_Q1_FULL : BMC2HOST_Q2_FULL)));
	if (ret)
		return -EINTR;

	memcpy(&tx_buff, buf, 4);
	writel(tx_buff, bmc_device->reg_base + ((index == QUEUE1) ? ASPEED_BMC_BMC2HOST_Q1 :
								    ASPEED_BMC_BMC2HOST_Q2));

	writel(BMC2HOST_INT_STS_DOORBELL | BMC2HOST_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);

	return sizeof(u32);
}

/* AST2600 */
static irqreturn_t aspeed_bmc_dev_pcie_isr(int irq, void *dev_id)
{
	struct aspeed_bmc_device *bmc_device = dev_id;

	while (!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q1_EMPTY))
		readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_Q1);

	while (!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q2_EMPTY))
		readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_Q2);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_bmc_dev_isr(int irq, void *dev_id)
{
	struct aspeed_bmc_device *bmc_device = dev_id;
	u32 host2bmc_q_sts = readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	if (host2bmc_q_sts & HOST2BMC_INT_STS_DOORBELL)
		writel(HOST2BMC_INT_STS_DOORBELL, bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	if (host2bmc_q_sts & HOST2BMC_ENABLE_INTB)
		writel(HOST2BMC_ENABLE_INTB, bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	if (host2bmc_q_sts & HOST2BMC_Q1_FULL)
		dev_info(bmc_device->dev, "Q1 Full\n");

	if (host2bmc_q_sts & HOST2BMC_Q2_FULL)
		dev_info(bmc_device->dev, "Q2 Full\n");

	if (!(readl(bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS) & BMC2HOST_Q1_FULL))
		wake_up_interruptible(&bmc_device->queue[QUEUE1].tx_wait);

	if (!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q1_EMPTY))
		wake_up_interruptible(&bmc_device->queue[QUEUE1].rx_wait);

	if (!(readl(bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS) & BMC2HOST_Q2_FULL))
		wake_up_interruptible(&bmc_device->queue[QUEUE2].tx_wait);

	if (!(readl(bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS) & HOST2BMC_Q2_EMPTY))
		wake_up_interruptible(&bmc_device->queue[QUEUE2].rx_wait);

	return IRQ_HANDLED;
}

static int aspeed_ast2600_init(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 pcie_config_ctl = SCU_PCIE_CONF_BMC_DEV_EN_IRQ |
			      SCU_PCIE_CONF_BMC_DEV_EN_MMIO | SCU_PCIE_CONF_BMC_DEV_EN;
	u32 scu_id;

	bmc_device->scu = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu");
	if (IS_ERR(bmc_device->scu)) {
		dev_err(&pdev->dev, "failed to find SCU regmap\n");
		return PTR_ERR(bmc_device->scu);
	}

	if (bmc_device->pcie2lpc)
		pcie_config_ctl |= SCU_PCIE_CONF_BMC_DEV_EN_E2L |
				   SCU_PCIE_CONF_BMC_DEV_EN_LPC_DECODE;

	regmap_update_bits(bmc_device->scu, ASPEED_SCU_PCIE_CONF_CTRL,
			   pcie_config_ctl, pcie_config_ctl);

	/* update class code to others as it is a MFD device */
	regmap_write(bmc_device->scu, ASPEED_SCU_BMC_DEV_CLASS, 0xff000000);

#ifdef SCU_TRIGGER_MSI
	//SCUC24[17]: Enable PCI device 1 INTx/MSI from SCU560[15]. Will be added in next version
	regmap_update_bits(bmc_device->scu, ASPEED_SCUC20, BIT(11) | BIT(14), BIT(11) | BIT(14));

	regmap_read(bmc_device->scu, ASPEED_SCU04, &scu_id);
	if (scu_id == AST2600A3_SCU04)
		regmap_update_bits(bmc_device->scu, ASPEED_SCUC24,
				   PCIDEV1_INTX_MSI_HOST2BMC_EN | MSI_ROUTING_MASK,
				   PCIDEV1_INTX_MSI_HOST2BMC_EN | MSI_ROUTING_PCIe2LPC_PCIDEV1);
	else
		regmap_update_bits(bmc_device->scu, ASPEED_SCUC24,
				   BIT(17) | BIT(14) | BIT(11), BIT(17) | BIT(14) | BIT(11));
#else
	//SCUC24[18]: Enable PCI device 1 INTx/MSI from Host-to-BMC controller.
	regmap_update_bits(bmc_device->scu, 0xc24, BIT(18) | BIT(14), BIT(18) | BIT(14));
#endif

	writel((~(bmc_device->bmc_mem_size - 1) & 0xFFFFFFFF) | HOST2BMC_MEM_BAR_ENABLE,
	       bmc_device->reg_base + ASPEED_BMC_MEM_BAR);
	writel(bmc_device->bmc_mem_phy, bmc_device->reg_base + ASPEED_BMC_MEM_BAR_REMAP);

	//Setting BMC to Host Q register
	writel(BMC2HOST_Q2_FULL_UNMASK | BMC2HOST_Q1_FULL_UNMASK | BMC2HOST_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	writel(HOST2BMC_Q2_FULL_UNMASK |  HOST2BMC_Q1_FULL_UNMASK | HOST2BMC_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	return 0;
}

static int aspeed_ast2700_init(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 pcie_config_ctl = SCU_PCIE_CONF_BMC_DEV_EN_IRQ |
			      SCU_PCIE_CONF_BMC_DEV_EN_MMIO | SCU_PCIE_CONF_BMC_DEV_EN;
	u32 scu_id;
	int i;

	bmc_device->config = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,config");
	if (IS_ERR(bmc_device->config)) {
		dev_err(&pdev->dev, "failed to find config regmap\n");
		return PTR_ERR(bmc_device->config);
	}

	bmc_device->device = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,device");
	if (IS_ERR(bmc_device->device)) {
		dev_err(&pdev->dev, "failed to find device regmap\n");
		return PTR_ERR(bmc_device->device);
	}

	bmc_device->e2m = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,e2m");
	if (IS_ERR(bmc_device->e2m)) {
		dev_err(&pdev->dev, "failed to find e2m regmap\n");
		return PTR_ERR(bmc_device->e2m);
	}

	bmc_device->scu = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu");
	if (IS_ERR(bmc_device->scu)) {
		dev_err(&pdev->dev, "failed to find SCU regmap\n");
		return PTR_ERR(bmc_device->scu);
	}

	if (bmc_device->pcie2lpc)
		pcie_config_ctl |= SCU_PCIE_CONF_BMC_DEV_EN_E2L |
				   SCU_PCIE_CONF_BMC_DEV_EN_LPC_DECODE;

	regmap_update_bits(bmc_device->config, 0x10, pcie_config_ctl, pcie_config_ctl);

	/* update class code to others as it is a MFD device */
	regmap_write(bmc_device->device, 0x18, 0xff000027);

	//MSI
	regmap_update_bits(bmc_device->device, 0x74, GENMASK(7, 4), BIT(7) | (5 << 4));

	//EnPCIaMSI_EnPCIaIntA_EnPCIaMst_EnPCIaDev
	//Disable MSI[bit25] in ast2700A0 int only
	regmap_read(bmc_device->scu, SCU0_REVISION_ID, &scu_id);
	if (scu_id & REVISION_ID)
		regmap_update_bits(bmc_device->device, 0x70,
				   BIT(25) | BIT(17) | BIT(9) | BIT(1),
				   BIT(25) | BIT(17) | BIT(9) | BIT(1));
	else
		regmap_update_bits(bmc_device->device, 0x70,
				   BIT(17) | BIT(9) | BIT(1),
				   BIT(25) | BIT(17) | BIT(9) | BIT(1));

	//bar size check for 4k align
	for (i = 1; i < 16; i++) {
		if ((bmc_device->bmc_mem_size / 4096) == (1 << (i - 1)))
			break;
	}
	if (i == 16) {
		i = 0;
		dev_warn(bmc_device->dev,
			 "Bar size not align for 4K : %dK\n", (u32)bmc_device->bmc_mem_size / 1024);
	}

	/*
	 * BAR assign in scu
	 * ((bar_mem / 4k) << 8) | per_size
	 */
	regmap_write(bmc_device->device, 0x1c, ((bmc_device->bmc_mem_phy) >> 4) | i);

	/*
	 * BAR assign in e2m
	 * e2m0:12c21000
	 * 108:host2bmc-0 for pcie0
	 * 128:host2bmc-1 for pcie0
	 * e2m1:12c22000
	 * 108:host2bmc-0 for pcie1
	 * 128:host2bmc-1 for pcie1
	 */
	if (bmc_device->id)
		regmap_write(bmc_device->e2m, 0x128, ((bmc_device->bmc_mem_phy) >> 4) | i);
	else
		regmap_write(bmc_device->e2m, 0x108, ((bmc_device->bmc_mem_phy) >> 4) | i);

	//Setting BMC to Host Q register
	writel(BMC2HOST_Q2_FULL_UNMASK | BMC2HOST_Q1_FULL_UNMASK | BMC2HOST_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_BMC2HOST_STS);
	writel(HOST2BMC_Q2_FULL_UNMASK | HOST2BMC_Q1_FULL_UNMASK | HOST2BMC_ENABLE_INTB,
	       bmc_device->reg_base + ASPEED_BMC_HOST2BMC_STS);

	return 0;
}

static int aspeed_bmc_device_setup_queue(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret, i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++) {
		struct aspeed_queue_message *queue = &bmc_device->queue[i];

		init_waitqueue_head(&queue->tx_wait);
		init_waitqueue_head(&queue->rx_wait);

		sysfs_bin_attr_init(&queue->bin);

		/* Queue name index starts from 1 */
		queue->bin.attr.name =
			devm_kasprintf(dev, GFP_KERNEL, "bmc-dev-queue%d", (i + 1));
		queue->bin.attr.mode = 0600;
		queue->bin.read = bmc_device->platform->queue_rx;
		queue->bin.write = bmc_device->platform->queue_tx;
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
		queue->bmc_device = bmc_device;
	}

	return 0;
}

static int aspeed_bmc_device_setup_memory_mapping(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	bmc_device->miscdev.minor = MISC_DYNAMIC_MINOR;
	bmc_device->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "bmc-device%d", bmc_device->id);
	bmc_device->miscdev.fops = &aspeed_bmc_device_fops;
	bmc_device->miscdev.parent = dev;
	ret = misc_register(&bmc_device->miscdev);
	if (ret) {
		dev_err(dev, "Unable to register device\n");
		return ret;
	}

	return 0;
}

static struct aspeed_platform ast2600_plaform = {
	.init = aspeed_ast2600_init,
	.queue_rx = aspeed_ast2600_queue_rx,
	.queue_tx = aspeed_ast2600_queue_tx
};

static struct aspeed_platform ast2700_plaform = {
	.init = aspeed_ast2700_init,
	.queue_rx = aspeed_ast2700_queue_rx,
	.queue_tx = aspeed_ast2700_queue_tx
};

static const struct of_device_id aspeed_bmc_device_of_matches[] = {
	{ .compatible = "aspeed,ast2600-bmc-device", .data = &ast2600_plaform },
	{ .compatible = "aspeed,ast2700-bmc-device", .data = &ast2700_plaform },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_bmc_device_of_matches);

static int aspeed_bmc_device_probe(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device;
	struct device *dev = &pdev->dev;
	struct reserved_mem *mem;
	const void *md = of_device_get_match_data(dev);
	struct device_node *np;
	int ret = 0, i;

	if (!md)
		return -ENODEV;

	bmc_device = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_bmc_device), GFP_KERNEL);
	if (!bmc_device)
		return -ENOMEM;
	dev_set_drvdata(dev, bmc_device);

	bmc_device->platform = md;

	bmc_device->id = ida_simple_get(&bmc_device_ida, 0, 0, GFP_KERNEL);
	if (bmc_device->id < 0)
		goto out_region;

	bmc_device->dev = dev;
	bmc_device->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bmc_device->reg_base))
		goto out_region;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "Failed to find memory-region.\n");
		ret = -ENOMEM;
		goto out_region;
	}

	mem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!mem) {
		dev_err(dev, "Failed to find reserved memory.\n");
		ret = -ENOMEM;
		goto out_region;
	}

	bmc_device->bmc_mem_size = mem->size;

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));

	if (of_reserved_mem_device_init(dev))
		dev_err(dev, "can't get reserved memory\n");
	bmc_device->bmc_mem_virt = dma_alloc_coherent(&pdev->dev, bmc_device->bmc_mem_size,
						      &bmc_device->bmc_mem_phy, GFP_KERNEL);

	bmc_device->irq = platform_get_irq(pdev, 0);
	if (bmc_device->irq < 0) {
		dev_err(&pdev->dev, "platform get of irq[=%d] failed!\n", bmc_device->irq);
		goto out_unmap;
	}
	ret = devm_request_irq(&pdev->dev, bmc_device->irq, aspeed_bmc_dev_isr, 0,
			       dev_name(&pdev->dev), bmc_device);
	if (ret) {
		dev_err(dev, "aspeed bmc device Unable to get IRQ");
		goto out_unmap;
	}

	ret = aspeed_bmc_device_setup_queue(pdev);
	if (ret) {
		dev_err(dev, "Cannot setup queue message");
		goto out_irq;
	}

	ret = aspeed_bmc_device_setup_memory_mapping(pdev);
	if (ret) {
		dev_err(dev, "Cannot setup memory mapping misc");
		goto out_free_queue;
	}

	if (of_property_read_bool(dev->of_node, "pcie2lpc"))
		bmc_device->pcie2lpc = 1;

	ret = bmc_device->platform->init(pdev);
	if (ret) {
		dev_err(dev, "Initialize bmc device failed\n");
		goto out_free_misc;
	}

	bmc_device->pcie_irq =  platform_get_irq(pdev, 1);
	if (bmc_device->pcie_irq < 0) {
		dev_warn(&pdev->dev,
			 "platform get of pcie irq[=%d] failed!\n", bmc_device->pcie_irq);
	} else {
		ret = devm_request_irq(&pdev->dev, bmc_device->pcie_irq,
				       aspeed_bmc_dev_pcie_isr, IRQF_SHARED,
				       dev_name(&pdev->dev), bmc_device);
		if (ret < 0) {
			dev_warn(dev, "Failed to request PCI-E IRQ %d.\n", ret);
			bmc_device->pcie_irq = -1;
		}
	}

	dev_info(dev, "aspeed bmc device: driver successfully loaded.\n");

	return 0;

out_free_misc:
	misc_deregister(&bmc_device->miscdev);
out_free_queue:
	for (i = 0; i < ASPEED_QUEUE_NUM; i++)
		sysfs_remove_bin_file(&pdev->dev.kobj, &bmc_device->queue[i].bin);
out_irq:
	devm_free_irq(&pdev->dev, bmc_device->irq, bmc_device);
out_unmap:
	iounmap(bmc_device->reg_base);
	dma_free_coherent(&pdev->dev, bmc_device->bmc_mem_size,
			  bmc_device->bmc_mem_virt, bmc_device->bmc_mem_phy);
out_region:
	devm_kfree(&pdev->dev, bmc_device);
	dev_warn(dev, "aspeed bmc device: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int  aspeed_bmc_device_remove(struct platform_device *pdev)
{
	struct aspeed_bmc_device *bmc_device = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ASPEED_QUEUE_NUM; i++)
		sysfs_remove_bin_file(&pdev->dev.kobj, &bmc_device->queue[i].bin);
	misc_deregister(&bmc_device->miscdev);
	devm_free_irq(&pdev->dev, bmc_device->irq, bmc_device);
	devm_free_irq(&pdev->dev, bmc_device->pcie_irq, bmc_device);

	iounmap(bmc_device->reg_base);

	dma_free_coherent(&pdev->dev, bmc_device->bmc_mem_size,
			  bmc_device->bmc_mem_virt, bmc_device->bmc_mem_phy);

	devm_kfree(&pdev->dev, bmc_device);

	return 0;
}

static struct platform_driver aspeed_bmc_device_driver = {
	.probe		= aspeed_bmc_device_probe,
	.remove		= aspeed_bmc_device_remove,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_bmc_device_of_matches,
	},
};

module_platform_driver(aspeed_bmc_device_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED BMC DEVICE Driver");
MODULE_LICENSE("GPL");
