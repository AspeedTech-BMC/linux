/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_MMBI_INTERNAL_H__
#define __ASPEED_MMBI_INTERNAL_H__

#include "aspeed-mmbi.h"

struct mmbi_chan_priv {
	enum mmbi_state state;			/* Current MMBI state of this channel */
	struct miscdevice miscdev;		/* MMBI char device for this channel */
	wait_queue_head_t rx_wait;		/* Wait queue for rx data available */
	wait_queue_head_t tx_wait;		/* Wait queue for tx data available */
	bool rx_ready;				/* Flag indicating if has data to be read */
	bool tx_ready;				/* Flag indicating if ready to transmit data */
	spinlock_t rx_lock;			/* Lock to prevent rx ready flag race */
	spinlock_t tx_lock;			/* Lock to prevent tx ready flag race */
	u32 read_ptr;				/* Current cached read pointer */
	u32 write_ptr;				/* Current cached write pointer */
	bool peer_ready;			/* Flag indicating peer ready bit */
	struct delayed_work poll_work;		/* Work struct for polling check status */
	bool running;
	struct mmbi_chan_desc *chan;		/* Back pointer used by misc open */
};

#endif /* __ASPEED_MMBI_INTERNAL_H__ */
