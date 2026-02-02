/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 Aspeed Technology Inc.
 */
#ifndef __ASPEED_MMBI_H__
#define __ASPEED_MMBI_H__

#include "asm-generic/int-ll64.h"
#include <linux/io.h>
#include "linux/miscdevice.h"
#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define MMBI_SIGNATURE "#MMBI$"
#define MMBI_DESC_SIZE_PREFIX 8
#define MMBI_DESC_SIZE_CHANNEL 32
#define MMBI_DESC_SIZE_INTERRUPT 24

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

#define MMBI_STATE_RDY_MSK BIT(0)
#define MMBI_STATE_RST_MSK BIT(0)
#define MMBI_STATE_IF_UP_MSK BIT(1)

#define MMBI_STATE_GET_RDY(val) ((val) & MMBI_STATE_RDY_MSK)
#define MMBI_STATE_GET_RST(val) ((val) & MMBI_STATE_RST_MSK)
#define MMBI_STATE_GET_IF_UP(val) ((val) & MMBI_STATE_IF_UP_MSK)

/* BMC interrupt value location is fixed at offset 55, only used in v1.1 */
#define MMBI_BMC_INT_VAL_OFFSET 55
/* Host interrupt value location is fixed at offset 45, only used in v1.1 */
#define MMBI_HOST_INT_VAL_OFFSET 45

static __always_inline void mmbi_set_ready(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt + sizeof(u32)) | MMBI_STATE_RDY_MSK,
		 devm_virt + sizeof(u32));
}

static __always_inline void mmbi_clr_ready(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt + sizeof(u32)) & ~MMBI_STATE_RDY_MSK,
		 devm_virt + sizeof(u32));
}

static __always_inline void mmbi_set_up(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt) | MMBI_STATE_IF_UP_MSK, devm_virt);
}

static __always_inline void mmbi_clr_up(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt) & ~MMBI_STATE_IF_UP_MSK, devm_virt);
}

static __always_inline void mmbi_set_rst(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt) | MMBI_STATE_RST_MSK, devm_virt);
}

static __always_inline void mmbi_clr_rst(u8 __iomem *devm_virt)
{
	iowrite8(ioread8(devm_virt) & ~MMBI_STATE_RST_MSK, devm_virt);
}

enum mmbi_role {
	MMBI_ROLE_HOST = 0,
	MMBI_ROLE_BMC  = 1,
};

enum mmbi_host_int_type {
	MMBI_HOST_INT_NONE = 0,
	MMBI_HOST_INT_PCIE = 1,
	MMBI_HOST_INT_GPIO = 2,
	MMBI_HOST_INT_ESPI_VW = 3,
};

enum mmbi_bmc_int_type {
	MMBI_BMC_INT_NONE = 0,
	MMBI_BMC_INT_MEM = 1,
	MMBI_BMC_INT_INBAND = 2,
};

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
	u32 h_ros_p;		/* Host Read Pointer address offset */
	u32 h_rws_p;		/* Host Read Write Pointer address offset */
	u8 host_int_val_ch;	/* Host interrupt value, only used in MMBI v1.1 */
	u8 bmc_int_val_ch;	/* Host interrupt value, only used in MMBI v1.1 */
};

struct mmbi_chan_desc {
	u32 b2h_ba_offset;			/* B2H Buffer Base Address */
	u32 h2b_ba_offset;			/* H2B Buffer Base Address */
	u32 b2h_l;				/* B2H Buffer Length */
	u32 h2b_l;				/* H2B Buffer Length */
	u8 buffer_type;				/* Buffer Type defined by enum mmbi_buffer_type */
	u8 index;				/* Channel Index filled by mmbi_instance_init */
	struct mmbi_buf_vpscb buffer_desc;	/* buffer descriptor information */
	enum mmbi_state state;			/* Current MMBI State of this channel */
	struct mmbi_ins_desc *mmbi;		/* Back pointer to instance descriptor */
	struct miscdevice miscdev;		/* MMBI char device for this channel */
};

struct mmbi_ins_desc {
	u8 __iomem *desc_virt;	/* MMBI Instance Base Address */
	u8 mmbi_version;
	u8 num_of_channels;	/* NOI field, recording number of channels */
	u8 os_use;
	u8 host_int_type;
	u8 host_int_location;
	u8 host_int_value;
	u8 bmc_int_type;
	u8 bmc_int_value;
	u32 bmc_int_location;
	int ins_id;
	struct mmbi_chan_desc chan_desc[MMBI_MAX_CHANNELS];
	struct device *dev;
	struct work_struct work;
	u8 pending_int;		/* Pending interrupt value, only for MMBI_VERSION_1_1 */
	spinlock_t irq_lock;	/* IRQ lock to prevent int_value race */
	enum mmbi_role role;
	void (*raise_interrupt)(struct device *dev, u8 __iomem *desc_virt, u32 location, u8 value);
};

void mmbi_channel_state_update(struct mmbi_chan_desc *chan,
			       u8 __iomem *desc_virt);
void mmbi_set_int_value(struct mmbi_ins_desc *mmbi, u32 val_location,
			u32 location, u8 val);
void mmbi_clr_pending_int(struct mmbi_ins_desc *mmbi, u8 idx);
int mmbi_instance_init(struct mmbi_ins_desc *mmbi);

#endif
