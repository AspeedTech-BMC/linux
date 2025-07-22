// SPDX-License-Identifier: GPL-2.0-only
/*
 * AST2700 Interrupt Controller
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

/* INTC0 register layout */
#define INTC0_PROT_OFFS           0x40
#define INTC0_ROUTING_SEL0_BASE   0x200
#define INTC0_ROUTING_GAP         0x100
#define INTC0_GROUPS              4

/* INTC1 register layout */
#define INTC1_PROT_OFFS           0x00
#define INTC1_ROUTING_SEL0_BASE   0x80
#define INTC1_ROUTING_GAP         0x20
#define INTC1_GROUPS              6

struct aspeed_intc_data {
	const char                  *name;
	u32                          prot_offs;
	u32                          rout_sel0_base;
	u32                          rout_gap;
	unsigned int                 groups;
};

static const struct aspeed_intc_data aspeed_intc0_data = {
	.name            = "INTC0",
	.prot_offs       = INTC0_PROT_OFFS,
	.rout_sel0_base  = INTC0_ROUTING_SEL0_BASE,
	.rout_gap        = INTC0_ROUTING_GAP,
	.groups          = INTC0_GROUPS,
};

static const struct aspeed_intc_data aspeed_intc1_data = {
	.name            = "INTC1",
	.prot_offs       = INTC1_PROT_OFFS,
	.rout_sel0_base  = INTC1_ROUTING_SEL0_BASE,
	.rout_gap        = INTC1_ROUTING_GAP,
	.groups          = INTC1_GROUPS,
};

struct aspeed_intc {
	void __iomem                    *base;
	const struct aspeed_intc_data   *data;
#ifdef CONFIG_DEBUG_FS
	struct dentry                   *dbg_root;
#endif
};

#ifdef CONFIG_DEBUG_FS
static int aspeed_intc_regs_show(struct seq_file *s, void *unused)
{
	struct aspeed_intc *intc = s->private;
	const struct aspeed_intc_data *d = intc->data;
	void __iomem *base = intc->base;
	unsigned int i;

	for (i = 0; i < d->groups; i++) {
		void __iomem *b = base + d->rout_sel0_base + i * 4;
		u32 r0 = readl(b);
		u32 r1 = readl(b + d->rout_gap);
		u32 r2 = readl(b + 2 * d->rout_gap);

		seq_printf(s, "ROUTE[%u]: 0x%08x 0x%08x 0x%08x\n", i, r0, r1, r2);
	}
	return 0;
}

static int aspeed_intc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, aspeed_intc_regs_show, inode->i_private);
}

static const struct file_operations aspeed_intc_regs_fops = {
	.owner    = THIS_MODULE,
	.open     = aspeed_intc_regs_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release,
};

static int aspeed_intc_prot_show(struct seq_file *s, void *unused)
{
	struct aspeed_intc *intc = s->private;
	const struct aspeed_intc_data *d = intc->data;
	u32 prot = readl(intc->base + d->prot_offs);

	seq_printf(s, "%s_PROT: 0x%08x\n", d->name, prot);
	return 0;
}

static int aspeed_intc_prot_open(struct inode *inode, struct file *file)
{
	return single_open(file, aspeed_intc_prot_show, inode->i_private);
}

static const struct file_operations aspeed_intc_prot_fops = {
	.owner    = THIS_MODULE,
	.open     = aspeed_intc_prot_open,
	.read     = seq_read,
	.llseek   = seq_lseek,
	.release  = single_release,
};
#endif /* CONFIG_DEBUG_FS */

static int aspeed_intc_probe(struct platform_device *pdev)
{
	const struct aspeed_intc_data *data;
	struct aspeed_intc *intc;
	struct resource *res;

	data = of_device_get_match_data(&pdev->dev);
	if (!data)
		return -ENODEV;

	intc = devm_kzalloc(&pdev->dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	intc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(intc->base))
		return PTR_ERR(intc->base);

	intc->data = data;

	platform_set_drvdata(pdev, intc);

#ifdef CONFIG_DEBUG_FS
	intc->dbg_root = debugfs_create_dir(dev_name(&pdev->dev), NULL);
	if (intc->dbg_root) {
		debugfs_create_file("routing", 0400, intc->dbg_root, intc,
				    &aspeed_intc_regs_fops);
		debugfs_create_file("protection", 0400, intc->dbg_root, intc,
				    &aspeed_intc_prot_fops);
	}
#endif
	return 0;
}

static void aspeed_intc_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	struct aspeed_intc *intc = platform_get_drvdata(pdev);

	if (intc && intc->dbg_root)
		debugfs_remove_recursive(intc->dbg_root);
#endif
}

static const struct of_device_id aspeed_intc_of_match[] = {
	{ .compatible = "aspeed,ast2700-intc0", .data = &aspeed_intc0_data },
	{ .compatible = "aspeed,ast2700-intc1", .data = &aspeed_intc1_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aspeed_intc_of_match);

static struct platform_driver aspeed_intc_driver = {
	.probe   = aspeed_intc_probe,
	.remove  = aspeed_intc_remove,
	.driver  = {
		.name           = "aspeed-ast2700-intc",
		.of_match_table = aspeed_intc_of_match,
	},
};
module_platform_driver(aspeed_intc_driver);
