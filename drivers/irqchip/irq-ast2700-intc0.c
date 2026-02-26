// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fwnode.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kconfig.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

#include "irq-ast2700.h"

#define INT_NUM		480
#define INTM_NUM	50
#define SWINT_NUM	16

#define INTM_BASE	(INT_NUM)
#define SWINT_BASE	(INT_NUM + INTM_NUM)
#define INT0_NUM	(INT_NUM + INTM_NUM + SWINT_NUM)

#define INTC0_IN_NUM		480
#define INTC0_ROUTE_NUM		5
#define INTC0_INTM_NUM		50

#define GIC_P2P_SPI_END		128

#define INTC0_SWINT_IER		0x10
#define INTC0_SWINT_ISR		0x14
#define INTC0_INTBANKX_IER	0x1000
#define INTC0_INTBANK_GROUPS	11
#define INTC0_INTBANKS_PER_GRP	3
#define INTC0_INTMX_IER		0x1b00
#define INTC0_INTMX_ISR		0x1b04
#define INTC0_INTM_BANK_NUM	3
#define INTM_IRQS_PER_BANK	10
#define INTC0_SEL_BASE			0x200
#define INTC0_SEL_BANK_SIZE		0x4
#define INTC0_SEL_ROUTE_SIZE	0x100

struct aspeed_intc0 {
	struct device				*dev;
	void __iomem				*base;
	raw_spinlock_t				intc_lock;
	struct irq_domain			*local;
	struct device_node			*parent;
	struct aspeed_intc_interrupt_ranges	ranges;
};

static void aspeed_swint_irq_mask(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;
	u32 ier;

	guard(raw_spinlock)(&intc0->intc_lock);
	ier = readl(intc0->base + INTC0_SWINT_IER) & ~BIT(bit);
	writel(ier, intc0->base + INTC0_SWINT_IER);
	irq_chip_mask_parent(data);
}

static void aspeed_swint_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;
	u32 ier;

	guard(raw_spinlock)(&intc0->intc_lock);
	ier = readl(intc0->base + INTC0_SWINT_IER) | BIT(bit);
	writel(ier, intc0->base + INTC0_SWINT_IER);
	irq_chip_unmask_parent(data);
}

static void aspeed_swint_irq_eoi(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bit = data->hwirq - SWINT_BASE;

	writel(BIT(bit), intc0->base + INTC0_SWINT_ISR);
	irq_chip_eoi_parent(data);
}

static struct irq_chip aspeed_swint_chip = {
	.name			= "ast2700-swint",
	.irq_eoi		= aspeed_swint_irq_eoi,
	.irq_mask		= aspeed_swint_irq_mask,
	.irq_unmask		= aspeed_swint_irq_unmask,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static void aspeed_intc0_irq_mask(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;
	u32 ier;

	guard(raw_spinlock_irqsave)(&intc0->intc_lock);
	ier = readl(intc0->base + INTC0_INTMX_IER + bank * 0x10) & ~BIT(bit);
	writel(ier, intc0->base + INTC0_INTMX_IER + bank * 0x10);
	irq_chip_mask_parent(data);
}

static void aspeed_intc0_irq_unmask(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;
	u32 ier;

	guard(raw_spinlock_irqsave)(&intc0->intc_lock);
	ier = readl(intc0->base + INTC0_INTMX_IER + bank * 0x10) | BIT(bit);
	writel(ier, intc0->base + INTC0_INTMX_IER + bank * 0x10);
	irq_chip_unmask_parent(data);
}

static void aspeed_intc0_irq_eoi(struct irq_data *data)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);
	int bank = (data->hwirq - INTM_BASE) / INTM_IRQS_PER_BANK;
	int bit = (data->hwirq - INTM_BASE) % INTM_IRQS_PER_BANK;

	writel(BIT(bit), intc0->base + INTC0_INTMX_ISR + bank * 0x10);
	irq_chip_eoi_parent(data);
}

static struct irq_chip aspeed_intm_chip = {
	.name			= "ast2700-intmerge",
	.irq_eoi		= aspeed_intc0_irq_eoi,
	.irq_mask		= aspeed_intc0_irq_mask,
	.irq_unmask		= aspeed_intc0_irq_unmask,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static struct irq_chip linear_intr_irq_chip = {
	.name			= "ast2700-int",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.flags			= IRQCHIP_SET_TYPE_MASKED,
};

static const aspeed_intc_output_t aspeed_intc0_routes[INTC0_IN_NUM / 32][INTC0_ROUTE_NUM] = {
	[0] = { 0, 256, 426, AST2700_INTC_INVALID_ROUTE, AST2700_INTC_INVALID_ROUTE },
	[1] = { 32, 288, 458, AST2700_INTC_INVALID_ROUTE, AST2700_INTC_INVALID_ROUTE },
	[2] = { 64, 320, 490, AST2700_INTC_INVALID_ROUTE, AST2700_INTC_INVALID_ROUTE },
	[3] = { 96, 352, 522, AST2700_INTC_INVALID_ROUTE, AST2700_INTC_INVALID_ROUTE },
	[4] = { 128, 384, 554, 160, 176 },
	[5] = { 129, 385, 555, 161, 177 },
	[6] = { 130, 386, 556, 162, 178 },
	[7] = { 131, 387, 557, 163, 179 },
	[8] = { 132, 388, 558, 164, 180 },
	[9] = { 133, 544, 714, 165, 181 },
	[10] = { 134, 545, 715, 166, 182 },
	[11] = { 135, 546, 706, 167, 183 },
	[12] = { 136, 547, 707, 168, 184 },
	[13] = { 137, 548, 708, 169, 185 },
	[14] = { 138, 549, 709, 170, 186 },
};

static const aspeed_intc_output_t aspeed_intc0_intm_routes[INTC0_INTM_NUM / 10] = {
	[0] = 192, /* INTM00 ~ INTM09 */
	[1] = 416, /* INTM10 ~ INTM19 */
	[2] = 586, /* INTM20 ~ INTM29 */
	[3] = 208, /* INTM30 ~ INTM39 */
	[4] = 224, /* INTM40 ~ INTM49 */
};

static int resolve_input_from_child_ranges(const struct aspeed_intc0 *intc0,
					   const struct aspeed_intc_interrupt_range *range,
					   u32 outpin, u32 *input)
{
	u32 offset, base;

	if (!in_range32(outpin, range->start, range->count))
		return -ENOENT;

	if (range->upstream.param_count == 0)
		return -EINVAL;

	base = range->upstream.param[0];
	offset = outpin - range->start;
	if (offset && !in_range32(base, 0, U32_MAX - offset + 1)) {
		dev_warn(intc0->dev, "%s: Arithmetic overflow for input derivation: %u + %u\n",
			 __func__, base, offset);
		return -EINVAL;
	}

	*input = base + offset;
	return 0;
}

static bool resolve_parent_range_for_output(const struct aspeed_intc0 *intc0,
					    const struct fwnode_handle *parent,
					    u32 output,
					    struct aspeed_intc_interrupt_range *resolved)
{
	for (size_t i = 0; i < intc0->ranges.nranges; i++) {
		struct aspeed_intc_interrupt_range range =
			intc0->ranges.ranges[i];

		if (!in_range32(output, range.start, range.count))
			continue;

		if (range.upstream.fwnode != parent)
			continue;

		if (resolved) {
			resolved->start = output;
			resolved->count = 1;
			resolved->upstream = range.upstream;
			resolved->upstream.param[1] += output - range.start;
		}

		return true;
	}

	return false;
}

static int resolve_parent_route_for_input(const struct aspeed_intc0 *intc0,
					  const struct fwnode_handle *parent, u32 input,
					  struct aspeed_intc_interrupt_range *resolved)
{
	aspeed_intc_output_t c0o;
	int rc = -ENOENT;

	if (input < INT_NUM) {
		bool found;

		static_assert(INTC0_ROUTE_NUM < INT_MAX, "Broken cast");
		for (size_t i = 0; rc == -ENOENT && i < INTC0_ROUTE_NUM; i++) {
			c0o = aspeed_intc0_routes[input / 32][i];
			if (c0o == AST2700_INTC_INVALID_ROUTE)
				continue;

			if (input < GIC_P2P_SPI_END)
				c0o += input % 32;

			found = resolve_parent_range_for_output(intc0, parent, c0o, resolved);
			rc = found ? (int)i : -ENOENT;
		}
	} else if (input < (INT_NUM + INTM_NUM)) {
		bool found;

		c0o = aspeed_intc0_intm_routes[(input - INT_NUM) / INTM_IRQS_PER_BANK];
		c0o += ((input - INT_NUM) % INTM_IRQS_PER_BANK);

		found = resolve_parent_range_for_output(intc0, parent, c0o, resolved);
		rc = found ? 0 : -ENOENT;
	} else if (input < (INT_NUM + INTM_NUM + SWINT_NUM)) {
		bool found;

		c0o = input - SWINT_BASE + 144;
		found = resolve_parent_range_for_output(intc0, parent, c0o, resolved);
		rc = found ? 0 : -ENOENT;
	} else {
		return -ENOENT;
	}

	return rc;
}

/**
 * aspeed_intc0_resolve_route - Determine the necessary interrupt output at intc1
 * @c0domain: The pointer to intc0's irq_domain
 * @nc1outs: The number of valid intc1 outputs available for the input
 * @c1outs: The array of available intc1 output indices for the input
 * @nc1ranges: The number of interrupt range entries for intc1
 * @c1ranges: The array of configured intc1 interrupt ranges
 * @resolved: The fully resolved range entry after applying the resolution
 *            algorithm
 *
 * Returns: The intc1 route index associated with the intc1 output identified in
 * @resolved on success. Otherwise, a negative errno value.
 *
 * The AST2700 interrupt architecture allows any peripheral interrupt source
 * to be routed to one of up to four processors running in the SoC. A processor
 * binding a driver for a peripheral that requests an interrupt is (without
 * further design and effort) the destination for the requested interrupt.
 *
 * Routing a peripheral interrupt to its destination processor requires
 * coordination between INTC0 on the CPU die and one or more INTC1 instances.
 * At least one INTC1 instance exists in the SoC on the IO-die, however up
 * to two more instances may be integrated via LTPI (LVDS Tunneling Protocol
 * & Interface).
 *
 * Between the multiple destinations, various route constraints, and the
 * devicetree binding design, some information that's needed at INTC1 instances
 * to route inbound interrupts correctly to the destination processor is only
 * available at INTC0.
 *
 * aspeed_intc0_resolve_route() is to be invoked by INTC1 driver instances to
 * perform the route resolution. The implementation in INTC0 allows INTC0 to
 * encapsulate the information used to perform route selection, and provides it
 * with an opportunity to apply policy as part of the selection process. Such
 * policy may, for instance, choose to de-prioritise some interrupts destined
 * for the PSP (Primary Service Processor) GIC.
 */
int aspeed_intc0_resolve_route(const struct irq_domain *c0domain, size_t nc1outs,
			       const aspeed_intc_output_t c1outs[static nc1outs],
			       size_t nc1ranges,
			       const struct aspeed_intc_interrupt_range c1ranges[static nc1ranges],
			       struct aspeed_intc_interrupt_range *resolved)
{
	struct fwnode_handle *parent_fwnode;
	struct aspeed_intc0 *intc0;
	int ret;

	if (!c0domain || !resolved)
		return -EINVAL;

	if (nc1outs > INT_MAX)
		return -EINVAL;

	if (nc1outs == 0 || nc1ranges == 0)
		return -ENODEV;

	if (!fwnode_device_is_compatible(c0domain->fwnode,
					 "aspeed,ast2700-intc0-ic"))
		return -ENODEV;

	intc0 = c0domain->host_data;
	if (!intc0)
		return -EINVAL;

	parent_fwnode = of_fwnode_handle(intc0->parent);

	for (size_t i = 0; i < nc1outs; i++) {
		aspeed_intc_output_t c1o = c1outs[i];

		if (c1o == AST2700_INTC_INVALID_ROUTE)
			continue;

		for (size_t j = 0; j < nc1ranges; j++) {
			struct aspeed_intc_interrupt_range c1r = c1ranges[j];
			u32 input;

			/*
			 * Range match for intc1 output pin
			 *
			 * Assume a failed match is still a match for the purpose of testing,
			 * saves a bunch of mess in the test fixtures
			 */
			if (!(c0domain == irq_find_matching_fwspec(&c1r.upstream,
								   c0domain->bus_token) ||
			      IS_ENABLED(CONFIG_ASPEED_AST2700_INTC_TEST)))
				continue;

			ret = resolve_input_from_child_ranges(intc0, &c1r, c1o, &input);
			if (ret)
				continue;

			/*
			 * INTC1 should never request routes for peripheral interrupt sources
			 * directly attached to INTC0.
			 */
			if (input < GIC_P2P_SPI_END)
				continue;

			ret = resolve_parent_route_for_input(intc0, parent_fwnode, input, NULL);
			if (ret < 0)
				continue;

			/* Route resolution succeeded */
			resolved->start = c1o;
			resolved->count = 1;
			resolved->upstream = c1r.upstream;
			resolved->upstream.param[0] = input;
			/* Cast protected by prior test against nc1outs */
			return (int)i;
		}
	}

	ret = -EHOSTUNREACH;
	return ret;
}

static int aspeed_intc0_irq_domain_map(struct irq_domain *domain,
				       unsigned int irq, irq_hw_number_t hwirq)
{
	if (hwirq < GIC_P2P_SPI_END)
		irq_set_chip_and_handler(irq, &linear_intr_irq_chip, handle_level_irq);
	else if (hwirq < INTM_BASE)
		return -EINVAL;
	else if (hwirq < SWINT_BASE)
		irq_set_chip_and_handler(irq, &aspeed_intm_chip, handle_level_irq);
	else if (hwirq < INT0_NUM)
		irq_set_chip_and_handler(irq, &aspeed_swint_chip, handle_level_irq);
	else
		return -EINVAL;

	irq_set_chip_data(irq, domain->host_data);
	return 0;
}

static int aspeed_intc0_irq_domain_translate(struct irq_domain *domain,
					     struct irq_fwspec *fwspec,
					     unsigned long *hwirq,
					     unsigned int *type)
{
	if (fwspec->param_count != 1)
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type = IRQ_TYPE_NONE;
	return 0;
}

static int aspeed_intc0_irq_domain_alloc(struct irq_domain *domain,
					 unsigned int virq,
					 unsigned int nr_irqs, void *data)
{
	struct aspeed_intc0 *intc0 = domain->host_data;
	struct aspeed_intc_interrupt_range resolved;
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	struct irq_chip *chip;
	unsigned long hwirq;
	unsigned int type;
	int ret;

	ret = aspeed_intc0_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq >= GIC_P2P_SPI_END && hwirq < INT_NUM)
		return -EINVAL;

	if (hwirq < INTM_BASE)
		chip = &linear_intr_irq_chip;
	else if (hwirq < SWINT_BASE)
		chip = &aspeed_intm_chip;
	else
		chip = &aspeed_swint_chip;

	ret = resolve_parent_route_for_input(intc0, domain->parent->fwnode,
					     (u32)hwirq, &resolved);
	if (ret)
		return ret;

	parent_fwspec = resolved.upstream;
	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					   &parent_fwspec);
	if (ret)
		return ret;

	for (int i = 0; i < nr_irqs; ++i, ++hwirq, ++virq) {
		ret = irq_domain_set_hwirq_and_chip(domain, virq, hwirq, chip,
						    domain->host_data);
		if (ret)
			return ret;
	}

	return 0;
}

static int aspeed_intc0_irq_domain_activate(struct irq_domain *domain,
					    struct irq_data *data, bool reserve)
{
	struct aspeed_intc0 *intc0 = irq_data_get_irq_chip_data(data);

	if (data->hwirq < INT_NUM) {
		int bank = data->hwirq / 32;
		int bit = data->hwirq % 32;
		u32 mask = BIT(bit);
		int route;

		route = resolve_parent_route_for_input(intc0,
						       intc0->local->parent->fwnode,
						       data->hwirq, NULL);
		if (route < 0)
			return route;

		guard(raw_spinlock_irqsave)(&intc0->intc_lock);
		for (int i = 0; i < 3; i++) {
			void __iomem *sel = intc0->base + INTC0_SEL_BASE +
					    (bank * INTC0_SEL_BANK_SIZE) +
					    (INTC0_SEL_ROUTE_SIZE * i);
			u32 reg = readl(sel);

			if (route & BIT(i))
				reg |= mask;
			else
				reg &= ~mask;

			writel(reg, sel);
			if (readl(sel) != reg)
				return -EACCES;
		}
		return 0;
	}

	if (in_range32(data->hwirq, INTM_BASE, INTM_NUM + SWINT_NUM))
		return 0;

	return -EINVAL;
}

static const struct irq_domain_ops aspeed_intc0_ic_irq_domain_ops = {
	.translate	= aspeed_intc0_irq_domain_translate,
	.activate	= aspeed_intc0_irq_domain_activate,
	.alloc		= aspeed_intc0_irq_domain_alloc,
	.free		= irq_domain_free_irqs_common,
	.map		= aspeed_intc0_irq_domain_map,
};

static void aspeed_intc0_disable_swint(struct aspeed_intc0 *intc0)
{
	writel(0, intc0->base + INTC0_SWINT_IER);
}

static void aspeed_intc0_disable_intbank(struct aspeed_intc0 *intc0)
{
	for (int i = 0; i < INTC0_INTBANK_GROUPS; i++) {
		for (int j = 0; j < INTC0_INTBANKS_PER_GRP; j++) {
			u32 base = INTC0_INTBANKX_IER + (0x100 * i) + (0x10 * j);

			writel(0, intc0->base + base);
		}
	}
}

static void aspeed_intc0_disable_intm(struct aspeed_intc0 *intc0)
{
	int i;

	for (i = 0; i < INTC0_INTM_BANK_NUM; i++)
		writel(0, intc0->base + INTC0_INTMX_IER + (0x10 * i));
}

static int aspeed_intc0_ic_probe(struct platform_device *pdev,
				 struct device_node *parent)
{
	struct device_node *node = pdev->dev.of_node;
	struct irq_domain *parent_domain;
	struct aspeed_intc0 *intc0;
	int ret;

	if (!parent) {
		pr_err("missing parent interrupt node\n");
		return -ENODEV;
	}

	intc0 = devm_kzalloc(&pdev->dev, sizeof(*intc0), GFP_KERNEL);
	if (!intc0)
		return -ENOMEM;

	intc0->dev = &pdev->dev;
	intc0->parent = parent;
	intc0->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc0->base))
		return PTR_ERR(intc0->base);

	aspeed_intc0_disable_swint(intc0);
	aspeed_intc0_disable_intbank(intc0);
	aspeed_intc0_disable_intm(intc0);

	raw_spin_lock_init(&intc0->intc_lock);

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("unable to obtain parent domain\n");
		return -ENODEV;
	}

	if (!of_device_is_compatible(parent, "arm,gic-v3"))
		return -ENODEV;

	intc0->local = irq_domain_create_hierarchy(parent_domain, 0, INT0_NUM,
						   of_fwnode_handle(node),
						   &aspeed_intc0_ic_irq_domain_ops,
						   intc0);
	if (!intc0->local)
		return -ENOMEM;

	ret = aspeed_intc_populate_ranges(&pdev->dev, &intc0->ranges);
	if (ret < 0) {
		irq_domain_remove(intc0->local);
		return ret;
	}

	return 0;
}

IRQCHIP_PLATFORM_DRIVER_BEGIN(ast2700_intc0)
IRQCHIP_MATCH("aspeed,ast2700-intc0-ic", aspeed_intc0_ic_probe)
IRQCHIP_PLATFORM_DRIVER_END(ast2700_intc0)

#ifdef CONFIG_ASPEED_AST2700_INTC_TEST
#include "irq-ast2700-intc0-test.c"
#endif
