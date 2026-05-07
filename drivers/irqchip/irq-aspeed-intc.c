// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#define INTC_INT_ENABLE_REG	0x00
#define INTC_INT_STATUS_REG	0x04
#define INTC_IRQS_PER_WORD	32
#define INTC_IRQ_BASE		192

struct aspeed_intc_ic {
	void __iomem		*base;
	raw_spinlock_t		intc_lock;
	struct irq_domain	*irq_domain;
	struct device_node	*node;
	unsigned int		parent_irqs[INTC_IRQS_PER_WORD];
	unsigned int		parent_irq_count;
	struct kobject		kobj;
	struct list_head	list;
};

static LIST_HEAD(aspeed_intc_instances);

/*
 * INTC0 uses a 1:1 mapping between each parent GIC SPI line and one leaf
 * interrupt bit.  The leaf hwirq is derived directly from the parent GIC
 * hwirq number, so no status register scan is needed (contrast with intc1
 * which aggregates multiple sources behind a single parent line).
 */
static irqreturn_t aspeed_intc0_ic_irq_handler(int irq, void *dev_id)
{
	struct aspeed_intc_ic *intc_ic = dev_id;
	struct irq_data *irq_data = irq_get_irq_data(irq);
	unsigned long hwirq;

	if (!irq_data || !intc_ic) {
		pr_err("Invalid irq_data or intc_ic\n");
		return IRQ_NONE;
	}

	if (irq_data->hwirq < INTC_IRQ_BASE + 32) {
		pr_err("Invalid hwirq: %lu\n", irq_data->hwirq);
		return IRQ_NONE;
	}
	hwirq = irq_data->hwirq - INTC_IRQ_BASE - 32; /* 32 is SPI offset */

	generic_handle_domain_irq(intc_ic->irq_domain, hwirq);

	/*
	 * Serialize the status-clear to prevent potential race conditions
	 * when multiple interrupts are processed in a multi-core environment.
	 */
	raw_spin_lock(&intc_ic->intc_lock);
	writel(BIT(hwirq), intc_ic->base + INTC_INT_STATUS_REG);
	raw_spin_unlock(&intc_ic->intc_lock);

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_intc1_ic_irq_handler(int irq, void *dev_id)
{
	struct aspeed_intc_ic *intc_ic = dev_id;
	unsigned long bit, status;

	if (!intc_ic) {
		pr_err("Invalid intc_ic\n");
		return IRQ_NONE;
	}

	status = readl(intc_ic->base + INTC_INT_STATUS_REG);
	if (!status)
		return IRQ_NONE;

	for_each_set_bit(bit, &status, INTC_IRQS_PER_WORD) {
		generic_handle_domain_irq(intc_ic->irq_domain, bit);
		writel(BIT(bit), intc_ic->base + INTC_INT_STATUS_REG);
	}

	return IRQ_HANDLED;
}

static void aspeed_intc_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int mask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	mask = readl(intc_ic->base + INTC_INT_ENABLE_REG) & ~BIT(data->hwirq);
	writel(mask, intc_ic->base + INTC_INT_ENABLE_REG);
}

static void aspeed_intc_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int unmask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	unmask = readl(intc_ic->base + INTC_INT_ENABLE_REG) | BIT(data->hwirq);
	writel(unmask, intc_ic->base + INTC_INT_ENABLE_REG);
}

static void aspeed_intc_irq_print_chip(struct irq_data *data, struct seq_file *p)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);

	if (intc_ic && intc_ic->node)
		seq_printf(p, " %pOF", intc_ic->node);
	else
		seq_puts(p, " ASPEED INTC");
}

static struct irq_chip aspeed_intc_chip = {
	.name			= "ASPEED INTC",
	.irq_mask		= aspeed_intc_irq_mask,
	.irq_unmask		= aspeed_intc_irq_unmask,
	.irq_print_chip		= aspeed_intc_irq_print_chip,
};

static int aspeed_intc_ic_map_irq_domain(struct irq_domain *domain, unsigned int irq,
					 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_intc_ic_irq_domain_ops = {
	.map = aspeed_intc_ic_map_irq_domain,
};

static ssize_t irq_groups_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct aspeed_intc_ic *intc =
		container_of(kobj, struct aspeed_intc_ic, kobj);
	ssize_t len = 0;

	if (intc->parent_irq_count == 1) {
		/* Aggregated: one parent line serves all children. */
		len += sysfs_emit_at(buf, len, "group 0: parent_irq=%u members=",
				     intc->parent_irqs[0]);
		for (int hwirq = 0; hwirq < INTC_IRQS_PER_WORD; hwirq++) {
			unsigned int virq = irq_find_mapping(intc->irq_domain, hwirq);

			if (virq)
				len += sysfs_emit_at(buf, len, "%u ", virq);
		}
		len += sysfs_emit_at(buf, len, "\n");
	} else {
		/* 1:1: each parent serves one child (derived from GIC hwirq). */
		for (unsigned int i = 0; i < intc->parent_irq_count; i++) {
			struct irq_data *pdata = irq_get_irq_data(intc->parent_irqs[i]);
			unsigned int virq;
			unsigned long phw;

			if (!pdata)
				continue;
			phw = pdata->hwirq;
			if (phw < INTC_IRQ_BASE + 32)
				continue;
			virq = irq_find_mapping(intc->irq_domain,
						phw - INTC_IRQ_BASE - 32);
			if (!virq)
				continue;
			len += sysfs_emit_at(buf, len,
					     "group %u: parent_irq=%u members=%u\n",
					     i, intc->parent_irqs[i], virq);
		}
	}
	return len;
}

static struct kobj_attribute aspeed_intc_irq_groups_attr = __ATTR_RO(irq_groups);

static struct attribute *aspeed_intc_attrs[] = {
	&aspeed_intc_irq_groups_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(aspeed_intc);

static const struct kobj_type aspeed_intc_kobj_type = {
	.sysfs_ops	= &kobj_sysfs_ops,
	.default_groups	= aspeed_intc_groups,
};

static int __init aspeed_intc_ic_of_init(struct device_node *node,
					 struct device_node *parent)
{
	struct aspeed_intc_ic *intc_ic;
	irq_handler_t handler;
	int ret = 0;
	int irq, irq_count = 0, i;

	intc_ic = kzalloc(sizeof(*intc_ic), GFP_KERNEL);
	if (!intc_ic)
		return -ENOMEM;

	intc_ic->base = of_iomap(node, 0);
	if (!intc_ic->base) {
		pr_err("Failed to iomap intc_ic base\n");
		ret = -ENOMEM;
		goto err_free_ic;
	}
	intc_ic->node = node;
	writel(0xffffffff, intc_ic->base + INTC_INT_STATUS_REG);
	writel(0x0, intc_ic->base + INTC_INT_ENABLE_REG);

	irq_count = of_irq_count(node);
	if (irq_count == 0) {
		pr_err("Failed to get irq count\n");
		ret = -EINVAL;
		goto err_iounmap;
	}

	if (irq_count > INTC_IRQS_PER_WORD) {
		pr_err("Too many parent IRQs: %d\n", irq_count);
		ret = -EINVAL;
		goto err_iounmap;
	}

	intc_ic->irq_domain =
		irq_domain_create_linear(of_fwnode_handle(node),
					 INTC_IRQS_PER_WORD,
					 &aspeed_intc_ic_irq_domain_ops,
					 intc_ic);
	if (!intc_ic->irq_domain) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	raw_spin_lock_init(&intc_ic->intc_lock);

	if (irq_count > 1)
		handler = aspeed_intc0_ic_irq_handler;
	else
		handler = aspeed_intc1_ic_irq_handler;

	for (i = 0; i < irq_count; i++) {
		irq = irq_of_parse_and_map(node, i);
		if (!irq) {
			pr_err("Failed to get irq number\n");
			ret = -EINVAL;
			goto err_iounmap;
		} else {
			ret = request_irq(irq, handler, IRQF_NO_THREAD,
					  "aspeed-intc", intc_ic);
			if (ret) {
				pr_err("Failed to request IRQ %d\n", irq);
				irq_dispose_mapping(irq);
				goto err_iounmap;
			}

			intc_ic->parent_irqs[intc_ic->parent_irq_count++] = irq;
		}
	}

	list_add_tail(&intc_ic->list, &aspeed_intc_instances);

	return 0;

err_iounmap:
	for (i = 0; i < intc_ic->parent_irq_count; i++) {
		free_irq(intc_ic->parent_irqs[i], intc_ic);
		irq_dispose_mapping(intc_ic->parent_irqs[i]);
	}
	if (intc_ic->irq_domain)
		irq_domain_remove(intc_ic->irq_domain);
	iounmap(intc_ic->base);
err_free_ic:
	kfree(intc_ic);
	return ret;
}

/*
 * IRQCHIP_DECLARE callbacks fire from init_IRQ(), which runs before
 * kernel_kobj is created in core_initcall(ksysfs_init).  Defer the sysfs
 * setup until kernel_kobj is available so /sys/kernel/aspeed-intc-* can be
 * created.  The physical base address is appended to disambiguate multiple
 * controllers that share a basename (e.g. nested LTPI controllers).
 */
static int __init aspeed_intc_sysfs_init(void)
{
	struct aspeed_intc_ic *intc;
	struct resource res;
	int ret;

	list_for_each_entry(intc, &aspeed_intc_instances, list) {
		ret = of_address_to_resource(intc->node, 0, &res);
		if (ret) {
			pr_warn("Failed to get resource for %pOF: %d\n",
				intc->node, ret);
			continue;
		}

		ret = kobject_init_and_add(&intc->kobj, &aspeed_intc_kobj_type,
					   kernel_kobj, "aspeed-intc-%pOFn@%llx",
					   intc->node,
					   (unsigned long long)res.start);
		if (ret) {
			pr_warn("Failed to create sysfs entry for %pOF: %d\n",
				intc->node, ret);
			kobject_put(&intc->kobj);
		}
	}
	return 0;
}
late_initcall(aspeed_intc_sysfs_init);

IRQCHIP_DECLARE(ast2700_intc_ic, "aspeed,ast2700-intc-ic", aspeed_intc_ic_of_init);
