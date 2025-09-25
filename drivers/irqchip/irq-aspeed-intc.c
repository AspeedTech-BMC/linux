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
#define INTC_IRQS_PER_WORD	32
#define INTC_IRQ_BASE		192

struct aspeed_intc_ic {
	void __iomem		*base;
	raw_spinlock_t		intc_lock;
	struct irq_domain	*irq_domain;
};

static void aspeed_intc0_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_intc_ic *intc_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);
	unsigned long hwirq;

	if (!irq_data || !intc_ic) {
		pr_err("Invalid irq_data or intc_ic\n");
		return;
	}

	if (irq_data->hwirq < INTC_IRQ_BASE + 32) {
		pr_err("Invalid hwirq: %lu\n", irq_data->hwirq);
		return;
	}
	hwirq = irq_data->hwirq - INTC_IRQ_BASE - 32; /* 32 is SPI offset */

	chained_irq_enter(chip, desc);

	generic_handle_domain_irq(intc_ic->irq_domain, hwirq);
	writel(BIT(hwirq), intc_ic->base + INTC_INT_STATUS_REG);

	chained_irq_exit(chip, desc);
}

static void aspeed_intc1_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_intc_ic *intc_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;

	if (!intc_ic) {
		pr_err("Invalid intc_ic\n");
		return;
	}

	chained_irq_enter(chip, desc);

	status = readl(intc_ic->base + INTC_INT_STATUS_REG);

	for_each_set_bit(bit, &status, INTC_IRQS_PER_WORD) {
		generic_handle_domain_irq(intc_ic->irq_domain, bit);
		writel(BIT(bit), intc_ic->base + INTC_INT_STATUS_REG);
	}

	chained_irq_exit(chip, desc);
}

static void aspeed_intc_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int mask;

	guard(raw_spinlock)(&intc_ic->intc_lock);
	mask = readl(intc_ic->base + INTC_INT_ENABLE_REG) & ~BIT(data->hwirq);
	writel(mask, intc_ic->base + INTC_INT_ENABLE_REG);
}

static void aspeed_intc_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	unsigned int unmask;

	guard(raw_spinlock)(&intc_ic->intc_lock);
	unmask = readl(intc_ic->base + INTC_INT_ENABLE_REG) | BIT(data->hwirq);
	writel(unmask, intc_ic->base + INTC_INT_ENABLE_REG);
}

static struct irq_chip aspeed_intc_chip = {
	.name			= "ASPEED INTC",
	.irq_mask		= aspeed_intc_irq_mask,
	.irq_unmask		= aspeed_intc_irq_unmask,
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

static int __init aspeed_intc_ic_of_init(struct device_node *node,
					 struct device_node *parent)
{
	struct aspeed_intc_ic *intc_ic;
	int ret = 0;
	int irq, irq_count, i;

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

	intc_ic->irq_domain = irq_domain_add_linear(node, INTC_IRQS_PER_WORD,
						    &aspeed_intc_ic_irq_domain_ops, intc_ic);
	if (!intc_ic->irq_domain) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	raw_spin_lock_init(&intc_ic->intc_lock);

	irq_count = of_irq_count(node);
	if (irq_count == 0) {
		pr_err("Failed to get irq count\n");
		ret = -EINVAL;
		goto err_iounmap;
	}

	for (i = 0; i < irq_count; i++) {
		irq = irq_of_parse_and_map(node, i);
		if (!irq) {
			pr_err("Failed to get irq number\n");
			ret = -EINVAL;
			goto err_iounmap;
		} else {
			if (irq_count > 1)
				irq_set_chained_handler_and_data(irq, aspeed_intc0_ic_irq_handler, intc_ic);
			else
				irq_set_chained_handler_and_data(irq, aspeed_intc1_ic_irq_handler, intc_ic);
		}
	}

	return 0;

err_iounmap:
	for (i = 0; i < irq_count; i++) {
		irq = irq_of_parse_and_map(node, i);
		if (irq)
			irq_dispose_mapping(irq);
	}
	iounmap(intc_ic->base);
err_free_ic:
	kfree(intc_ic);
	return ret;
}

IRQCHIP_DECLARE(ast2700_intc_ic, "aspeed,ast2700-intc-ic", aspeed_intc_ic_of_init);
