// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED MMBI (Memory Mapped Bus Interface) Protocol Driver
 *
 * Copyright (c) 2026 ASPEED Technology Inc.
 */

#include "aspeed-mmbi.h"
#include <linux/export.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/bitfield.h>
#include <linux/bits.h>

#define MMBI_PUT_U32_MSB(base, off, val)                                   \
	do {                                                               \
		for (int __i = 0; __i < 4; __i++) {                        \
			(base)[(off)++] = (u8)((val) >> (24 - (__i * 8))); \
		}                                                          \
	} while (0)

static int mmbi_buffer_init_vpscb(u8 __iomem *buf_virt, struct mmbi_buf_vpscb *buf_desc, u8 index)
{
	u32 offset;

	offset = 0;

	iowrite8(MMBI_BUFFER_TYPE_VPSCB & MMBI_BUF_TYPE_MASK, buf_virt + offset++);
	iowrite8(index & MMBI_INT_VAL_MASK, buf_virt + offset++);
	iowrite8(index & MMBI_INT_VAL_MASK, buf_virt + offset++);

	offset += 5; /* reserved bytes */

	MMBI_PUT_U32_MSB(buf_virt, offset, MMBI_BUF_ADDR_ALIGN(buf_desc->h_ros_p));
	MMBI_PUT_U32_MSB(buf_virt, offset, MMBI_BUF_ADDR_ALIGN(buf_desc->h_rws_p));
	return 0;
}

static int mmbi_channel_init(u8 __iomem *desc_virt, struct mmbi_chan_desc *chan_desc)
{
	u32 offset = 0;

	MMBI_PUT_U32_MSB(desc_virt, offset, MMBI_BUF_ADDR_ALIGN(chan_desc->b2h_ba_offset));
	MMBI_PUT_U32_MSB(desc_virt, offset, MMBI_BUF_ADDR_ALIGN(chan_desc->h2b_ba_offset));
	MMBI_PUT_U32_MSB(desc_virt, offset, chan_desc->b2h_l);
	MMBI_PUT_U32_MSB(desc_virt, offset, chan_desc->h2b_l);

	if (chan_desc->buffer_type == MMBI_BUFFER_TYPE_VPSCB) {
		mmbi_buffer_init_vpscb(desc_virt + chan_desc->buffer_desc_offset,
				       (struct mmbi_buf_vpscb *)chan_desc->buffer_desc,
					  chan_desc->index);
	} else {
		pr_err("\t%s: unsupported buffer type %d\n", __func__,
		       chan_desc->buffer_type);
		return -EINVAL;
	}

	return 0;
}

int mmbi_instance_init(struct mmbi_ins_desc *mmbi)
{
	int rc, i, offset;
	u8 __iomem *desc_virt;

	if (!mmbi)
		return -EINVAL;

	if (mmbi->mmbi_version != MMBI_VERSION_1_0 &&
	    mmbi->mmbi_version != MMBI_VERSION_1_1) {
		dev_err(mmbi->dev, "%s: unsupported MMBI version %d\n",
			__func__, mmbi->mmbi_version);
		return -EINVAL;
	}

	if (mmbi->num_of_instances > MMBI_MAX_CHANNELS) {
		dev_err(mmbi->dev, "%s: invalid number of instances %d\n",
			__func__, mmbi->num_of_instances);
		return -EINVAL;
	}

	if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		if (mmbi->host_int_value || mmbi->bmc_int_value) {
			dev_warn(mmbi->dev,
				 "%s: invalid interrupt value (host: %d, bmc: %d)\n",
				__func__, mmbi->host_int_value,
				mmbi->bmc_int_value);
			mmbi->host_int_value = 0;
			mmbi->bmc_int_value = 0;
		}
	}

	desc_virt = mmbi->desc_virt;
	i = FIELD_PREP((u32)MMBI_NOI_MASK, mmbi->num_of_instances);
	i |= FIELD_PREP((u32)MMBI_OS_USE_MASK, mmbi->os_use);

	memcpy_toio(desc_virt, MMBI_SIGNATURE, sizeof(MMBI_SIGNATURE));
	desc_virt += sizeof(MMBI_SIGNATURE);

	iowrite8(mmbi->mmbi_version & MMBI_VERSION_MASK, desc_virt++);
	iowrite8(i, desc_virt++);

	mmbi->chan_desc[0].index = 0;
	rc = mmbi_channel_init(desc_virt, &mmbi->chan_desc[0]);
	if (rc) {
		dev_err(mmbi->dev, "%s: failed to init channel 0\n", __func__);
		return rc;
	}
	desc_virt += MMBI_CHANNEL_DESC_SIZE;

	iowrite8(mmbi->host_int_type & MMBI_INT_TYPE_MASK, desc_virt++);
	iowrite8(mmbi->host_int_location, desc_virt++);
	desc_virt += 3; /* reserved bytes */
	iowrite8(mmbi->host_int_value, desc_virt++);
	iowrite8(mmbi->host_int_value, desc_virt++);
	iowrite8(mmbi->bmc_int_type & MMBI_INT_TYPE_MASK, desc_virt++);
	offset = desc_virt - mmbi->desc_virt;
	desc_virt += sizeof(mmbi->bmc_int_location);
	MMBI_PUT_U32_MSB(mmbi->desc_virt, offset, mmbi->bmc_int_location);

	desc_virt += 4; /* reserved bytes */
	iowrite8(mmbi->bmc_int_value, desc_virt++);

	if (mmbi->num_of_instances > 1) {
		desc_virt += sizeof(struct mmbi_chan_desc);
		for (i = 1; i < mmbi->num_of_instances; i++) {
			mmbi->chan_desc[i].index = i;
			rc = mmbi_channel_init(desc_virt, &mmbi->chan_desc[1]);
			if (rc) {
				dev_err(mmbi->dev, "%s: failed to init channel %d\n", __func__, i);
				return rc;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmbi_instance_init);
