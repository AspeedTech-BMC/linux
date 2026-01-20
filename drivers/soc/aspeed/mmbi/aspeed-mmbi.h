/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_MMBI_H__
#define __ASPEED_MMBI_H__

#include <linux/types.h>

#define MMBI_SIGNATURE "#MMBI$"
#define MMBI_CHANNEL_DESC_SIZE 32

#define MMBI_VERSION_1_0 0x01
#define MMBI_VERSION_1_1 0x02
#define MMBI_VERSION_MASK GENMASK(3, 0)
#define MMBI_MAX_CHANNELS 7
#define MMBI_NOI_MASK GENMASK(6, 4)
#define MMBI_OS_USE_MASK BIT(0)
#define MMBI_BUF_ADDR_ALIGN(addr) (ALIGN_DOWN(addr, 8))
#define MMBI_BUF_TYPE_MASK GENMASK(3, 0)
#define MMBI_INT_VAL_MASK GENMASK(3, 0)
#define MMBI_INT_TYPE_MASK GENMASK(2, 0)

enum mmbi_state {				/* B_U B_R H_U H_R */
	INIT_IN_PROGRESS	= 0x00,		/* 0   0   0   0   */
	INIT_COMPLETED		= 0x08,		/* 1   0   0   0   */
	NORMAL_RUNTIME		= 0x0A,		/* 1   0   1   0   */
	RESET_REQ_BY_BMC	= 0x0E,		/* 1   1   1   0   */
	RESET_REQ_BY_HOST	= 0x0B,		/* 1   0   1   1   */
	RESET_ACKED		= 0x0F,		/* 1   1   1   1   */
	TRANS_TO_INIT		= 0x07,		/* 0   1   1   1   */
	INIT_MISMATCH		= 0x09,		/* 1   0   0   1   */
	POWER_UP_OR_ERROR	= 0x80000000,
};

enum mmbi_buffer_type {
	MMBI_BUFFER_TYPE_VPSCB = 0x01,
};

struct mmbi_buf_vpscb {
	u32 h_ros_p;		/* Host Read Offset Pointer address offset */
	u32 h_rws_p;		/* Host Read Write Pointer address offset */
};

struct mmbi_chan_desc {
	u32 b2h_ba_offset;		/* B2H Buffer Base Address */
	u32 h2b_ba_offset;		/* H2B Buffer Base Address */
	u32 b2h_l;			/* B2H Buffer Length */
	u32 h2b_l;			/* H2B Buffer Length */
	u8 buffer_type;			/* Buffer Type defined by enum mmbi_buffer_type */
	u8 index;			/* Channel Index filled by mmbi_instance_init */
	u32 buffer_desc_offset;		/* Mapped virtual address offset to start of this channel */
	void *buffer_desc;		/* buffer descriptor information */
};

struct mmbi_ins_desc {
	u8 __iomem *desc_virt;		/* MMBI Instance Base Address */
	u8 mmbi_version;
	u8 num_of_instances;
	u8 os_use;
	u8 host_int_type;
	u8 host_int_location;
	u8 bmc_int_type;
	u32 bmc_int_location;
	u8 host_int_value;
	u8 bmc_int_value;
	struct mmbi_chan_desc chan_desc[MMBI_MAX_CHANNELS];
	struct device *dev;
};

int mmbi_instance_init(struct mmbi_ins_desc *mmbi);

#endif
