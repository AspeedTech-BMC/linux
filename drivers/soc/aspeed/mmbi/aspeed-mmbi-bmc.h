/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_MMBI_BMC_H__
#define __ASPEED_MMBI_BMC_H__

#include "aspeed-mmbi.h"

int mmbi_instance_init_bmc(struct mmbi_ins_desc *mmbi);
void mmbi_instance_irq_bmc(struct mmbi_ins_desc *mmbi);

#endif /* __ASPEED_MMBI_BMC_H__ */
