// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

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
};

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

IRQCHIP_DECLARE(ast2700_intc_ic, "aspeed,ast2700-intc-ic", aspeed_intc_ic_of_init);
