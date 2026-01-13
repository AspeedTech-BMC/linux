// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>

#define INTC1_IER	0x100
#define INTC1_ISR	0x104
#define INTC1_IRQS_PER_BANK	32
#define INTC1_BANK_NUM		6

struct aspeed_intc_ic {
	void __iomem		*base;
	raw_spinlock_t		intc_lock;
	struct irq_domain	*irq_domain;
};

static void aspeed_intc1_ic_irq_handler(struct irq_desc *desc)
{
	struct aspeed_intc_ic *intc_ic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;

	chained_irq_enter(chip, desc);

	for (int bank = 0; bank < INTC1_BANK_NUM; bank++) {
		status = readl(intc_ic->base + INTC1_ISR + (0x10 * bank));
		if (!status)
			continue;

		for_each_set_bit(bit, &status, INTC1_IRQS_PER_BANK) {
			generic_handle_domain_irq(intc_ic->irq_domain,
						  (bank * INTC1_IRQS_PER_BANK) + bit);
			writel(BIT(bit), intc_ic->base + INTC1_ISR + (0x10 * bank));
		}
	}

	chained_irq_exit(chip, desc);
}

static void aspeed_intc1_irq_mask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq % INTC1_IRQS_PER_BANK;
	int bank = data->hwirq / INTC1_IRQS_PER_BANK;
	unsigned int mask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	mask = readl(intc_ic->base + INTC1_IER + (0x10 * bank)) & ~BIT(bit);
	writel(mask, intc_ic->base + INTC1_IER + (0x10 * bank));
}

static void aspeed_intc1_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq % INTC1_IRQS_PER_BANK;
	int bank = data->hwirq / INTC1_IRQS_PER_BANK;
	unsigned int unmask;

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	unmask = readl(intc_ic->base + INTC1_IER + (0x10 * bank)) | BIT(bit);
	writel(unmask, intc_ic->base + INTC1_IER + (0x10 * bank));
}

static struct irq_chip aspeed_intc_chip = {
	.name			= "ASPEED INTC1",
	.irq_mask		= aspeed_intc1_irq_mask,
	.irq_unmask		= aspeed_intc1_irq_unmask,
};

static int aspeed_intc1_irq_domain_translate(struct irq_domain *domain, struct irq_fwspec *fwspec,
					     unsigned long *hwirq, unsigned int *type)
{
	if (fwspec->param_count != 1)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = IRQ_TYPE_LEVEL_HIGH;
	return 0;
}

static int aspeed_intc1_ic_map_irq_domain(struct irq_domain *domain, unsigned int irq,
					  irq_hw_number_t hwirq)
{
	irq_domain_set_info(domain, irq, hwirq, &aspeed_intc_chip,
			    domain->host_data, handle_level_irq, NULL, NULL);
	return 0;
}

static int aspeed_intc1_irq_domain_activate(struct irq_domain *domain,
					    struct irq_data *data, bool reserve)
{
	struct aspeed_intc_ic *intc_ic = irq_data_get_irq_chip_data(data);
	int bank = data->hwirq / INTC1_IRQS_PER_BANK;
	int bit = data->hwirq % INTC1_IRQS_PER_BANK;
	u32 mask = BIT(bit);

	guard(raw_spinlock_irqsave)(&intc_ic->intc_lock);
	for (int i = 0; i < 3; i++) {
		void __iomem *sel = intc_ic->base + 0x80 + bank * 4 + 0x20 * i;

		if (readl(sel) & mask) {
			writel(readl(sel) & ~mask, sel);
			if (readl(sel) & mask)
				return -EACCES;
		}
	}

	return 0;
}

static const struct irq_domain_ops aspeed_intc1_ic_irq_domain_ops = {
	.map = aspeed_intc1_ic_map_irq_domain,
	.translate	= aspeed_intc1_irq_domain_translate,
	.activate = aspeed_intc1_irq_domain_activate,
};

static int aspeed_intc1_interrupt_ranges(struct aspeed_intc_ic *intc_ic,
					 struct device_node *node,
					 struct device_node *parent_node)
{
	struct of_phandle_args parent_irq;
	int i, j, n, ret;

	if (!of_device_is_compatible(parent_node, "aspeed,ast2700-intc0-ic"))
		return -ENOENT;

	n = of_property_count_elems_of_size(node, "aspeed,interrupt-ranges", sizeof(u32));
	if (n <= 0 || n % 4)
		return -EINVAL;

	/*
	 * Each range is described as:
	 * <intm_pin count phandle base_irq>
	 * and we only care about ranges whose phandle matches
	 * this controller's interrupt-parent (&intc0).
	 */
	for (i = 0; i < n / 4; i++) {
		struct device_node *target;
		phandle parent_handle;
		u32 base_irq;
		u32 count;

		ret = of_property_read_u32_index(node, "aspeed,interrupt-ranges",
						 i * 4 + 2, &parent_handle);
		if (ret)
			return ret;

		target = of_find_node_by_phandle(parent_handle);
		if (!target)
			continue;

		if (target != parent_node) {
			of_node_put(target);
			continue;
		}

		ret = of_property_read_u32_index(node, "aspeed,interrupt-ranges",
						 i * 4 + 1, &count);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(node, "aspeed,interrupt-ranges",
						 i * 4 + 3, &base_irq);
		if (ret)
			return ret;

		for (j = 0; j < count; j++) {
			int irq;

			parent_irq.np = parent_node;
			parent_irq.args_count = 1;
			parent_irq.args[0] = base_irq + j;
			irq = irq_create_of_mapping(&parent_irq);
			if (!irq)
				continue;

			irq_set_chained_handler_and_data(irq,
							 aspeed_intc1_ic_irq_handler,
							 intc_ic);
		}

		of_node_put(target);
	}

	return 0;
}

static void aspeed_intc1_disable_int(struct aspeed_intc_ic *intc_ic)
{
	for (int i = 0; i < INTC1_BANK_NUM; i++)
		writel(0x0, intc_ic->base + INTC1_IER + (0x10 * i));
}

static int aspeed_intc1_ic_of_init(struct device_node *node, struct device_node *parent)
{
	struct aspeed_intc_ic *intc_ic;
	int ret = 0;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	if (!irq_find_host(parent))
		return -ENODEV;

	intc_ic = kzalloc(sizeof(*intc_ic), GFP_KERNEL);
	if (!intc_ic)
		return -ENOMEM;

	intc_ic->base = of_iomap(node, 0);
	if (!intc_ic->base) {
		pr_err("Failed to iomap intc_ic base\n");
		ret = -ENOMEM;
		goto err_free_ic;
	}

	raw_spin_lock_init(&intc_ic->intc_lock);

	aspeed_intc1_disable_int(intc_ic);
	intc_ic->irq_domain = irq_domain_create_linear(of_fwnode_handle(node),
						       INTC1_BANK_NUM * INTC1_IRQS_PER_BANK,
						       &aspeed_intc1_ic_irq_domain_ops, intc_ic);
	if (!intc_ic->irq_domain) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	ret = aspeed_intc1_interrupt_ranges(intc_ic, node, parent);
	if (ret < 0)
		goto err_iounmap;

	return 0;

err_iounmap:
	iounmap(intc_ic->base);
err_free_ic:
	kfree(intc_ic);
	return ret;
}

IRQCHIP_DECLARE(ast2700_intc1_ic, "aspeed,ast2700-intc1-ic", aspeed_intc1_ic_of_init);
