/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Aspeed AST2700 Interrupt Controller.
 *
 *  Copyright (C) 2023 ASPEED Technology Inc.
 */
#ifndef DRIVERS_IRQCHIP_AST2700
#define DRIVERS_IRQCHIP_AST2700

#include <linux/device.h>
#include <linux/irqdomain.h>

#define AST2700_INTC_INVALID_ROUTE (~0U)

struct aspeed_intc_interrupt_range {
	u32 start;
	u32 count;
	struct irq_fwspec upstream;
};

struct aspeed_intc_interrupt_ranges {
	struct aspeed_intc_interrupt_range *ranges;
	unsigned int nranges;
};

int aspeed_intc_populate_ranges(struct device *dev,
				struct aspeed_intc_interrupt_ranges *ranges);

typedef u32 aspeed_intc_output_t;

int aspeed_intc0_resolve_route(const struct irq_domain *c0domain,
			       size_t nc1outs,
			       const aspeed_intc_output_t c1outs[static nc1outs],
			       size_t nc1ranges,
			       const struct aspeed_intc_interrupt_range c1ranges[static nc1ranges],
			       struct aspeed_intc_interrupt_range *resolved);

#endif
