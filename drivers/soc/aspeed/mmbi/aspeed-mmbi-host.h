/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_MMBI_HOST_H__
#define __ASPEED_MMBI_HOST_H__

#include "aspeed-mmbi.h"

int mmbi_instance_init_host(struct mmbi_ins_desc *mmbi);
void mmbi_channel_irq_host(struct mmbi_chan_desc *chan);

#endif /* __ASPEED_MMBI_HOST_H__ */
