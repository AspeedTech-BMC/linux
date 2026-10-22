/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright ASPEED Technology
 */
#ifndef __LINUX_SOC_ASPEED_LTPI_H
#define __LINUX_SOC_ASPEED_LTPI_H

#include <linux/errno.h>
#include <linux/notifier.h>

struct device;

/*
 * An LTPI controller whose link partner is an AST1700 panics the system when
 * the link is lost. Notifier blocks registered with
 * aspeed_ltpi_register_panic_notifier() are called from the controller's hard
 * IRQ handler immediately before that panic(), with @action set to
 * %ASPEED_LTPI_LINK_LOST_PANIC and @data pointing to a
 * struct aspeed_ltpi_link_lost_info.
 *
 * The callback must not sleep, must not take locks the interrupted context
 * may hold, and must return NOTIFY_DONE so that every registered block runs.
 * Two LTPI instances can lose link at the same time and call the chain
 * concurrently on different CPUs, and panic() on one CPU may stop another
 * CPU in the middle of its callback, so the record should be a single write.
 *
 * Link loss with a non-AST1700 link partner only logs an error and does not
 * call the chain: a call always means panic() follows.
 */
#define ASPEED_LTPI_LINK_LOST_PANIC	1

/**
 * struct aspeed_ltpi_link_lost_info - LTPI instance whose link was lost
 * @dev: LTPI controller device. @dev->of_node identifies the board DT node.
 * @index: LTPI controller number, 0 for LTPI0 and 1 for LTPI1, matching the
 *	   "LTPI%d" reported by the link_status sysfs attribute.
 *
 * Valid only for the duration of the notifier call.
 */
struct aspeed_ltpi_link_lost_info {
	struct device *dev;
	unsigned int index;
};

#if IS_ENABLED(CONFIG_ASPEED_LTPI)
int aspeed_ltpi_register_panic_notifier(struct notifier_block *nb);
int aspeed_ltpi_unregister_panic_notifier(struct notifier_block *nb);
#else
static inline int aspeed_ltpi_register_panic_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}

static inline int aspeed_ltpi_unregister_panic_notifier(struct notifier_block *nb)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* __LINUX_SOC_ASPEED_LTPI_H */
