// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#define INTC_INT_ENABLE_REG	0x00
#define INTC_INT_STATUS_REG	0x04

struct aspeed_intc_ic {
	void __iomem		*base;
	raw_spinlock_t		gic_lock;
	raw_spinlock_t		intc_lock;
	struct irq_domain	*irq_domain;
};

static void aspeed_intc_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_intc_ic *intc_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status, flags;

	chained_irq_enter(chip, desc);

	raw_spin_lock_irqsave(&intc_ic->gic_lock, flags);
	status = readl(intc_ic->base + INTC_INT_STATUS_REG);
	for_each_set_bit(bit, &status, 32) {
		generic_handle_domain_irq(intc_ic->irq_domain, bit);
		writel(BIT(bit), intc_ic->base + INTC_INT_STATUS_REG);
	}
	raw_spin_unlock_irqrestore(&intc_ic->gic_lock, flags);

	chained_irq_exit(chip, desc);
}

static void aspeed_intc_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int mask = readl(intc_ic->base + INTC_INT_ENABLE_REG) & ~BIT(data->hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&intc_ic->intc_lock, flags);
	writel(mask, intc_ic->base + INTC_INT_ENABLE_REG);
	raw_spin_unlock_irqrestore(&intc_ic->intc_lock, flags);
}

static void aspeed_intc_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int unmask = readl(intc_ic->base + INTC_INT_ENABLE_REG) | BIT(data->hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&intc_ic->intc_lock, flags);
	writel(unmask, intc_ic->base + INTC_INT_ENABLE_REG);
	raw_spin_unlock_irqrestore(&intc_ic->intc_lock, flags);
}

static int aspeed_intc_irq_set_affinity(struct irq_data *data,
					  const struct cpumask *dest,
					  bool force)
{
	return -EINVAL;
}

static struct irq_chip aspeed_intc_chip = {
	.name			= "ASPEED INTC",
	.irq_mask		= aspeed_intc_irq_mask,
	.irq_unmask		= aspeed_intc_irq_unmask,
	.irq_set_affinity	= aspeed_intc_irq_set_affinity,
};

static int aspeed_intc_ic_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_intc_ic_irq_domain_ops = {
	.map = aspeed_intc_ic_map_irq_domain,
};

static int __init aspeed_intc_ic_of_init(struct device_node *node,
					struct device_node *parent)
{
	struct aspeed_intc_ic *intc_ic;
	int ret = 0;
	int irq;

	intc_ic = kzalloc(sizeof(*intc_ic), GFP_KERNEL);
	if (!intc_ic)
		return -ENOMEM;

	intc_ic->base = of_iomap(node, 0);
	if (!intc_ic->base) {
		pr_err("Failed to iomap intc_ic base\n");
		ret = -ENOMEM;
		goto err_free_ic;
	}
	writel(0xffffffff, intc_ic->base + INTC_INT_STATUS_REG);
	writel(0x0, intc_ic->base + INTC_INT_ENABLE_REG);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_err("Failed to get irq number\n");
		ret = -EINVAL;
		goto err_iounmap;
	}

	intc_ic->irq_domain = irq_domain_add_linear(node, 32,
						   &aspeed_intc_ic_irq_domain_ops,
						   intc_ic);
	if (!intc_ic->irq_domain) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	raw_spin_lock_init(&intc_ic->gic_lock);
	raw_spin_lock_init(&intc_ic->intc_lock);

	intc_ic->irq_domain->name = "aspeed-intc-domain";

	irq_set_chained_handler_and_data(irq,
					 aspeed_intc_ic_irq_handler, intc_ic);

	return 0;

err_iounmap:
	iounmap(intc_ic->base);
err_free_ic:
	kfree(intc_ic);
	return ret;
}

static int __init aspeed_intc_ic_of_init_v2(struct device_node *node,
					    struct device_node *parent)
{
	struct aspeed_intc_ic *intc_ic;
	int ret = 0;
	int irq, i;

	intc_ic = kzalloc(sizeof(*intc_ic), GFP_KERNEL);
	if (!intc_ic)
		return -ENOMEM;

	intc_ic->base = of_iomap(node, 0);
	if (!intc_ic->base) {
		pr_err("Failed to iomap intc_ic base\n");
		ret = -ENOMEM;
		goto err_free_ic;
	}
	writel(0xffffffff, intc_ic->base + INTC_INT_STATUS_REG);
	writel(0x0, intc_ic->base + INTC_INT_ENABLE_REG);

	intc_ic->irq_domain = irq_domain_add_linear(node, 32,
						    &aspeed_intc_ic_irq_domain_ops,
						    intc_ic);
	if (!intc_ic->irq_domain) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	raw_spin_lock_init(&intc_ic->gic_lock);
	raw_spin_lock_init(&intc_ic->intc_lock);

	intc_ic->irq_domain->name = "aspeed-intc-domain";

	for (i = 0; i < of_irq_count(node); i++) {
		irq = irq_of_parse_and_map(node, i);
		if (!irq) {
			pr_err("Failed to get irq number\n");
			ret = -EINVAL;
			goto err_iounmap;
		} else {
			irq_set_chained_handler_and_data(irq, aspeed_intc_ic_irq_handler, intc_ic);
		}
	}

	return 0;

err_iounmap:
	iounmap(intc_ic->base);
err_free_ic:
	kfree(intc_ic);
	return ret;
}

IRQCHIP_DECLARE(ast2700_intc_ic, "aspeed,ast2700-intc-ic", aspeed_intc_ic_of_init);
IRQCHIP_DECLARE(ast2700_intc_icv2, "aspeed,ast2700-intc-icv2", aspeed_intc_ic_of_init_v2);
