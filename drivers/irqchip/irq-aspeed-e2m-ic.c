// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Aspeed AST27XX E2M Interrupt Controller
 * Copyright (C) 2023 ASPEED Technology Inc.
 *
 */

#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

#define ASPEED_AST2700_E2M_IC_SHIFT	0
#define ASPEED_AST2700_E2M_IC_ENABLE	\
	GENMASK(7, ASPEED_AST2700_E2M_IC_SHIFT)
#define ASPEED_AST2700_E2M_IC_NUM_IRQS	8
#define ASPEED_AST2700_E2M_IC_EN_REG	0x14
#define ASPEED_AST2700_E2M_IC_STS_REG	0x18

struct aspeed_e2m_ic {
	unsigned long irq_enable;
	unsigned long irq_shift;
	unsigned int num_irqs;
	unsigned int reg;
	unsigned int en_reg;
	unsigned int sts_reg;
	struct regmap *e2m;
	struct irq_domain *irq_domain;
};

static void aspeed_e2m_ic_irq_handler(struct irq_desc *desc)
{
	unsigned int val;
	unsigned long bit;
	unsigned long enabled;
	unsigned long max;
	unsigned long status;
	struct aspeed_e2m_ic *e2m_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int mask;

	chained_irq_enter(chip, desc);

	mask = e2m_ic->irq_enable;
	regmap_read(e2m_ic->e2m, e2m_ic->en_reg, &val);
	enabled = val & e2m_ic->irq_enable;
	regmap_read(e2m_ic->e2m, e2m_ic->sts_reg, &val);
	status = val & enabled;

	bit = e2m_ic->irq_shift;
	max = e2m_ic->num_irqs + bit;

	for_each_set_bit_from(bit, &status, max) {
		generic_handle_domain_irq(e2m_ic->irq_domain, bit - e2m_ic->irq_shift);

		regmap_write_bits(e2m_ic->e2m, e2m_ic->sts_reg, mask, BIT(bit));
	}

	chained_irq_exit(chip, desc);
}

static void aspeed_e2m_ic_irq_mask(struct irq_data *data)
{
	struct aspeed_e2m_ic *e2m_ic = irq_data_get_irq_chip_data(data);
	unsigned int mask;

	mask = BIT(data->hwirq + e2m_ic->irq_shift);
	regmap_update_bits(e2m_ic->e2m, e2m_ic->en_reg, mask, 0);
}

static void aspeed_e2m_ic_irq_unmask(struct irq_data *data)
{
	struct aspeed_e2m_ic *e2m_ic = irq_data_get_irq_chip_data(data);
	unsigned int bit = BIT(data->hwirq + e2m_ic->irq_shift);
	unsigned int mask;

	mask = bit;
	regmap_update_bits(e2m_ic->e2m, e2m_ic->en_reg, mask, bit);
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

	if (!node->parent) {
		rc = -ENODEV;
		goto err;
	}

	e2m_ic->e2m = syscon_node_to_regmap(node->parent);
	if (IS_ERR(e2m_ic->e2m)) {
		rc = PTR_ERR(e2m_ic->e2m);
		goto err;
	}

	/* Clear status and disable all interrupt */
	regmap_write_bits(e2m_ic->e2m, e2m_ic->sts_reg,
			  e2m_ic->irq_enable, e2m_ic->irq_enable);
	regmap_write_bits(e2m_ic->e2m, e2m_ic->en_reg,
			  e2m_ic->irq_enable, 0);

	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		rc = -EINVAL;
		goto err;
	}

	e2m_ic->irq_domain = irq_domain_add_linear(node, e2m_ic->num_irqs,
						   &aspeed_e2m_ic_domain_ops,
						   e2m_ic);
	if (!e2m_ic->irq_domain) {
		rc = -ENOMEM;
		goto err;
	}

	irq_set_chained_handler_and_data(irq, aspeed_e2m_ic_irq_handler,
					 e2m_ic);

	return 0;

err:
	kfree(e2m_ic);

	return rc;
}

static int __init aspeed_ast2700_e2m_ic_of_init(struct device_node *node,
						struct device_node *parent)
{
	struct aspeed_e2m_ic *e2m_ic = kzalloc(sizeof(*e2m_ic), GFP_KERNEL);

	if (!e2m_ic)
		return -ENOMEM;

	e2m_ic->irq_enable = ASPEED_AST2700_E2M_IC_ENABLE;
	e2m_ic->irq_shift = ASPEED_AST2700_E2M_IC_SHIFT;
	e2m_ic->num_irqs = ASPEED_AST2700_E2M_IC_NUM_IRQS;
	e2m_ic->en_reg = ASPEED_AST2700_E2M_IC_EN_REG;
	e2m_ic->sts_reg = ASPEED_AST2700_E2M_IC_STS_REG;

	return aspeed_e2m_ic_of_init_common(e2m_ic, node);
}

IRQCHIP_DECLARE(ast2700_e2m_ic, "aspeed,ast2700-e2m-ic",
		aspeed_ast2700_e2m_ic_of_init);
