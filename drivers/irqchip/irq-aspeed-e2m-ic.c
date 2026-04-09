// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Aspeed AST27XX E2M Interrupt Controller
 * Copyright (C) 2023 ASPEED Technology Inc.
 *
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#define ASPEED_AST2700_E2M_IC_SHIFT	0
#define ASPEED_AST2700_E2M_IC_ENABLE	\
	GENMASK(7, ASPEED_AST2700_E2M_IC_SHIFT)
#define ASPEED_AST2700_E2M_IC_NUM_IRQS	8
#define ASPEED_AST2700_E2M_IC_EN_REG	0x00
#define ASPEED_AST2700_E2M_IC_STS_REG	0x04

struct aspeed_e2m_ic {
	unsigned long irq_enable;
	unsigned long irq_shift;
	unsigned int num_irqs;
	unsigned int reg;
	unsigned int en_reg;
	unsigned int sts_reg;
	void __iomem *base;
	raw_spinlock_t lock;
	struct irq_domain *irq_domain;
};

static void aspeed_e2m_ic_irq_handler(struct irq_desc *desc)
{
	unsigned long bit;
	unsigned long enabled;
	unsigned long max;
	unsigned long status;
	struct aspeed_e2m_ic *e2m_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int mask;

	chained_irq_enter(chip, desc);

	mask = e2m_ic->irq_enable;
	enabled = readl(e2m_ic->base + e2m_ic->en_reg) & e2m_ic->irq_enable;
	status = readl(e2m_ic->base + e2m_ic->sts_reg) & enabled;

	bit = e2m_ic->irq_shift;
	max = e2m_ic->num_irqs + bit;

	for_each_set_bit_from(bit, &status, max) {
		generic_handle_domain_irq(e2m_ic->irq_domain, bit - e2m_ic->irq_shift);
		writel(BIT(bit), e2m_ic->base + e2m_ic->sts_reg);
	}

	chained_irq_exit(chip, desc);
}

static void aspeed_e2m_ic_irq_mask(struct irq_data *data)
{
	struct aspeed_e2m_ic *e2m_ic = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 mask, val;

	mask = BIT(data->hwirq + e2m_ic->irq_shift);
	raw_spin_lock_irqsave(&e2m_ic->lock, flags);
	val = readl(e2m_ic->base + e2m_ic->en_reg);
	writel(val & ~mask, e2m_ic->base + e2m_ic->en_reg);
	raw_spin_unlock_irqrestore(&e2m_ic->lock, flags);
}

static void aspeed_e2m_ic_irq_unmask(struct irq_data *data)
{
	struct aspeed_e2m_ic *e2m_ic = irq_data_get_irq_chip_data(data);
	unsigned long flags;
	u32 bit, val;

	bit = BIT(data->hwirq + e2m_ic->irq_shift);
	raw_spin_lock_irqsave(&e2m_ic->lock, flags);
	val = readl(e2m_ic->base + e2m_ic->en_reg);
	writel(val | bit, e2m_ic->base + e2m_ic->en_reg);
	raw_spin_unlock_irqrestore(&e2m_ic->lock, flags);
}

static int aspeed_e2m_ic_irq_set_affinity(struct irq_data *data,
					  const struct cpumask *dest,
					  bool force)
{
	return -EINVAL;
}

static struct irq_chip aspeed_scu_ic_chip = {
	.name			= "aspeed-e2m-ic",
	.irq_mask		= aspeed_e2m_ic_irq_mask,
	.irq_unmask		= aspeed_e2m_ic_irq_unmask,
	.irq_set_affinity	= aspeed_e2m_ic_irq_set_affinity,
};

static int aspeed_e2m_ic_map(struct irq_domain *domain, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &aspeed_scu_ic_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops aspeed_e2m_ic_domain_ops = {
	.map = aspeed_e2m_ic_map,
};

static int aspeed_e2m_ic_of_init_common(struct aspeed_e2m_ic *e2m_ic,
					struct device_node *node)
{
	int irq;
	int rc = 0;

	e2m_ic->base = of_iomap(node, 0);
	if (!e2m_ic->base) {
		rc = -ENOMEM;
		goto err;
	}
	raw_spin_lock_init(&e2m_ic->lock);

	/* Clear status and disable all interrupts */
	writel(e2m_ic->irq_enable, e2m_ic->base + e2m_ic->sts_reg);
	writel(0, e2m_ic->base + e2m_ic->en_reg);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		rc = -EINVAL;
		goto err_unmap;
	}

	e2m_ic->irq_domain = irq_domain_add_linear(node, e2m_ic->num_irqs,
						   &aspeed_e2m_ic_domain_ops,
						   e2m_ic);
	if (!e2m_ic->irq_domain) {
		rc = -ENOMEM;
		goto err_unmap;
	}

	irq_set_chained_handler_and_data(irq, aspeed_e2m_ic_irq_handler,
					 e2m_ic);

	return 0;

err_unmap:
	iounmap(e2m_ic->base);
err:
	kfree(e2m_ic);

	return rc;
}

static int aspeed_ast2700_e2m_ic_probe(struct platform_device *pdev, struct device_node *parent)
{
	struct aspeed_e2m_ic *e2m_ic;
	struct irq_domain *parent_domain;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("missing intc0 interrupt node\n");
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				   "Missing intc0 interrupt node\n");
	}

	e2m_ic = kzalloc(sizeof(*e2m_ic), GFP_KERNEL);
	if (!e2m_ic)
		return -ENOMEM;

	e2m_ic->irq_enable = ASPEED_AST2700_E2M_IC_ENABLE;
	e2m_ic->irq_shift = ASPEED_AST2700_E2M_IC_SHIFT;
	e2m_ic->num_irqs = ASPEED_AST2700_E2M_IC_NUM_IRQS;
	e2m_ic->en_reg = ASPEED_AST2700_E2M_IC_EN_REG;
	e2m_ic->sts_reg = ASPEED_AST2700_E2M_IC_STS_REG;

	return aspeed_e2m_ic_of_init_common(e2m_ic, pdev->dev.of_node);
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(ast2700_e2m)
IRQCHIP_MATCH("aspeed,ast2700-e2m-ic", aspeed_ast2700_e2m_ic_probe)
IRQCHIP_PLATFORM_DRIVER_END(ast2700_e2m)
