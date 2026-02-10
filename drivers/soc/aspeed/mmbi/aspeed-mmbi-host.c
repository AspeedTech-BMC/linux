// SPDX-License-Identifier: GPL-2.0+
/* Implements MMBI protocol for Host side with VPSCB buffer type
 * Copyright 2026 Aspeed Technology Inc.
 */

#include "linux/compiler_types.h"
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/printk.h>
#include "aspeed-mmbi.h"
#include "aspeed-mmbi-host.h"

#define MMBI_HOST_INT_VAL_OFFSET 45

static void mmbi_channel_state_handler(struct mmbi_chan_desc *chan, u8 __iomem *desc_virt)
{
	enum mmbi_state prev_state, cur_state;
	struct mmbi_buf_vpscb *vpscb;
	u8 __iomem *host_rws_virt;
	u32 host_int_location;

	/* avoid pending interrupt triggered while doing state handling */
	mmbi_clr_pending_int(chan->mmbi, chan->index);
	prev_state = chan->state;

	host_int_location = chan->mmbi->host_int_location;
	vpscb = &chan->buffer_desc;
	host_rws_virt = desc_virt + vpscb->h_rws_p;
	mmbi_channel_state_update(chan, desc_virt);
	cur_state = chan->state;

	switch (cur_state) {
	case INIT_COMPLETED:
		mmbi_set_ready(host_rws_virt);
		mmbi_set_up(host_rws_virt);
		chan->state = NORMAL_RUNTIME;
		mmbi_set_int_value(chan->mmbi, MMBI_HOST_INT_VAL_OFFSET,
				   host_int_location, BIT(chan->index));
		break;
	case NORMAL_RUNTIME:
		mmbi_set_ready(host_rws_virt);
		// TODO: check if has data to receive from BMC
		break;
	case RESET_REQ_BY_BMC:
		// TODO: consume all data from B2H buffer
		mmbi_clr_ready(host_rws_virt);
		mmbi_set_rst(host_rws_virt);
		chan->state = RESET_ACKED;
		mmbi_set_int_value(chan->mmbi, MMBI_HOST_INT_VAL_OFFSET,
				   host_int_location, BIT(chan->index));
		break;
	default:
		/* other states do not require action from host side */
		break;
	}
}

static int mmbi_instance_signature_check(u8 __iomem *desc_virt)
{
	char signature[sizeof(MMBI_SIGNATURE) - 1];

	memcpy_fromio(signature, desc_virt, sizeof(MMBI_SIGNATURE) - 1);
	if (memcmp(signature, MMBI_SIGNATURE, sizeof(MMBI_SIGNATURE) - 1) != 0)
		return -EINVAL;

	return 0;
}

static int mmbi_chan_init_vpscb_host(u8 __iomem *buf_virt, struct mmbi_buf_vpscb *buf_desc)
{
	u32 offset = 0;

	buf_desc->host_int_val_ch = ioread8(buf_virt + offset++) & MMBI_INT_VAL_MASK;
	buf_desc->bmc_int_val_ch = ioread8(buf_virt + offset++) & MMBI_INT_VAL_MASK;

	offset += 5; /* reserved bytes */

	memcpy_fromio(&buf_desc->h_ros_p, buf_virt + offset, sizeof(buf_desc->h_ros_p));
	memcpy_fromio(&buf_desc->h_rws_p, buf_virt + offset + 4, sizeof(buf_desc->h_rws_p));
	buf_desc->h_ros_p = be32_to_cpu(buf_desc->h_ros_p);
	buf_desc->h_rws_p = be32_to_cpu(buf_desc->h_rws_p);
	buf_desc->h_ros_p = MMBI_BUF_ADDR_ALIGN(buf_desc->h_ros_p);
	buf_desc->h_rws_p = MMBI_BUF_ADDR_ALIGN(buf_desc->h_rws_p);

	pr_info("%s:   h_ros_p=0x%x, h_rws_p=0x%x\n", __func__,
		buf_desc->h_ros_p, buf_desc->h_rws_p);
	return 0;
}

static int mmbi_channel_init_host(u8 __iomem *desc_virt, struct mmbi_chan_desc *chan_desc)
{
	struct mmbi_buf_vpscb *buffer_desc;

	if (!desc_virt || !IS_ALIGNED((unsigned long)desc_virt, sizeof(long))) {
		pr_err("%s: invalid desc_virt address %p\n", __func__, desc_virt);
		return -EINVAL;
	}

	mmbi_clr_pending_int(chan_desc->mmbi, chan_desc->index);
	memcpy_fromio(&chan_desc->b2h_ba_offset, desc_virt, sizeof(chan_desc->b2h_ba_offset));
	memcpy_fromio(&chan_desc->h2b_ba_offset, desc_virt + 4, sizeof(chan_desc->h2b_ba_offset));
	memcpy_fromio(&chan_desc->b2h_l, desc_virt + 8, sizeof(chan_desc->b2h_l));
	memcpy_fromio(&chan_desc->h2b_l, desc_virt + 12, sizeof(chan_desc->h2b_l));
	chan_desc->b2h_ba_offset = be32_to_cpu(chan_desc->b2h_ba_offset);
	chan_desc->h2b_ba_offset = be32_to_cpu(chan_desc->h2b_ba_offset);
	chan_desc->b2h_l = be32_to_cpu(chan_desc->b2h_l);
	chan_desc->h2b_l = be32_to_cpu(chan_desc->h2b_l);
	chan_desc->b2h_ba_offset = MMBI_BUF_ADDR_ALIGN(chan_desc->b2h_ba_offset);
	chan_desc->h2b_ba_offset = MMBI_BUF_ADDR_ALIGN(chan_desc->h2b_ba_offset);
	desc_virt += 16;
	chan_desc->buffer_type = ioread8(desc_virt++) & MMBI_BUF_TYPE_MASK;

	pr_info("%s:   b2h_ba_offset=0x%x, h2b_ba_offset=0x%x\n", __func__,
		chan_desc->b2h_ba_offset, chan_desc->h2b_ba_offset);
	pr_info("%s:   b2h_l=0x%x, h2b_l=0x%x\n", __func__, chan_desc->b2h_l, chan_desc->h2b_l);
	pr_info("%s:   buffer_type=%u\n", __func__, chan_desc->buffer_type);

	if (chan_desc->buffer_type == MMBI_BUFFER_TYPE_VPSCB) {
		buffer_desc = &chan_desc->buffer_desc;
		return mmbi_chan_init_vpscb_host(desc_virt, buffer_desc);
	}

	pr_err("\t%s: unsupported buffer type %d\n", __func__,
	       chan_desc->buffer_type);
	return -EINVAL;
}

void mmbi_instance_irq_host(struct mmbi_ins_desc *mmbi)
{
	if (mmbi->mmbi_version == MMBI_VERSION_1_0) {
		mmbi_channel_state_handler(&mmbi->chan_desc[0], mmbi->desc_virt);
	} else if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		u8 bmc_int_val;

		bmc_int_val = ioread8(mmbi->desc_virt + MMBI_BMC_INT_VAL_OFFSET);
		pr_info("%s: MMBI instance %u received interrupt with bmc_int_val 0x%x\n",
			__func__, mmbi->ins_id, bmc_int_val);

		for (int i = 0; i < mmbi->num_of_channels; i++) {
			if (bmc_int_val & (1 << i)) {
				mmbi_channel_state_handler(&mmbi->chan_desc[i],
							   mmbi->desc_virt);
			}
		}
		iowrite8(0, mmbi->desc_virt + MMBI_BMC_INT_VAL_OFFSET);
	}

	if (mmbi->pending_int) {
		/* trigger pending interrupt if exists */
		mmbi_set_int_value(mmbi, MMBI_HOST_INT_VAL_OFFSET, mmbi->host_int_location, 0);
	}
}
EXPORT_SYMBOL_GPL(mmbi_instance_irq_host);

int mmbi_instance_init_host(struct mmbi_ins_desc *mmbi)
{
	struct device *dev = mmbi->dev;
	struct mmbi_chan_desc *chan;
	int rc;
	u8 __iomem *desc_virt;
	u8 chan_idx, val;

	desc_virt = mmbi->desc_virt;
	if (mmbi_instance_signature_check(desc_virt)) {
		dev_err(dev, "Invalid MMBI signature\n");
		return -EINVAL;
	}

	desc_virt += sizeof(MMBI_SIGNATURE) - 1; // exclude signature from descriptor parsing
	chan_idx = 0;

	mmbi->mmbi_version = ioread8(desc_virt++) & MMBI_VERSION_MASK;
	if (mmbi->mmbi_version < MMBI_VERSION_1_0 || mmbi->mmbi_version > MMBI_VERSION_1_1) {
		dev_err(dev, "Unsupported MMBI version: %d\n", mmbi->mmbi_version);
		return -EINVAL;
	}

	if (mmbi->mmbi_version == MMBI_VERSION_1_1) {
		val = ioread8(desc_virt++);
		mmbi->os_use = val & MMBI_OS_USE_MASK;
		mmbi->num_of_channels = (val & MMBI_NOI_MASK) >> 4;
		if (mmbi->num_of_channels > MMBI_MAX_CHANNELS || mmbi->num_of_channels == 0) {
			dev_err(dev, "Invalid number of instances: %d\n", mmbi->num_of_channels);
			return -EINVAL;
		}
	} else {
		mmbi->os_use = ioread8(desc_virt++) & MMBI_OS_USE_MASK;
	}
	dev_info(dev, "MMBI version 0x%x, os_use %u, num_of_channels %u\n",
		 mmbi->mmbi_version, mmbi->os_use, mmbi->num_of_channels);

	chan = &mmbi->chan_desc[chan_idx];
	chan->index = chan_idx;
	chan->mmbi = mmbi;
	rc = mmbi_channel_init_host(desc_virt, chan);
	if (rc) {
		dev_err(dev, "Failed to initialize MMBI channel %d\n", chan_idx);
		return rc;
	}

	desc_virt += MMBI_DESC_SIZE_CHANNEL;
	chan_idx++;

	mmbi->host_int_type = ioread8(desc_virt++) & MMBI_INT_TYPE_MASK;
	mmbi->host_int_location = ioread8(desc_virt++);
	desc_virt += 3; // reserved 3 bytes
	mmbi->host_int_value = ioread8(desc_virt++);
	mmbi->bmc_int_type = ioread8(desc_virt++) & MMBI_INT_TYPE_MASK;
	memcpy_fromio(&mmbi->bmc_int_location, desc_virt, sizeof(mmbi->bmc_int_location));
	mmbi->bmc_int_location = be32_to_cpu(mmbi->bmc_int_location);
	desc_virt += sizeof(mmbi->bmc_int_location) + 4; // reserved 4 bytes
	mmbi->bmc_int_value = ioread8(desc_virt++);

	desc_virt += 8; // reserved 8 bytes

	dev_info(dev, "host_int_type=%u, host_int_location=0x%x, host_int_value=0x%x\n",
		 mmbi->host_int_type, mmbi->host_int_location, mmbi->host_int_value);
	dev_info(dev, "bmc_int_type=%u, bmc_int_location=0x%x, bmc_int_value=0x%x\n",
		 mmbi->bmc_int_type, mmbi->bmc_int_location, mmbi->bmc_int_value);
	dev_info(dev, "offset after init header=0x%lx\n",
		 (unsigned long)(desc_virt - mmbi->desc_virt));

	if (chan_idx < mmbi->num_of_channels) {
		dev_info(dev, "Initializing additional MMBI channels\n");
		for (; chan_idx < mmbi->num_of_channels; chan_idx++) {
			chan = &mmbi->chan_desc[chan_idx];
			chan->index = chan_idx;
			chan->mmbi = mmbi;
			rc = mmbi_channel_init_host(desc_virt, chan);
			if (rc) {
				dev_err(dev, "Failed to initialize MMBI channel %d\n", chan_idx);
				return rc;
			}
			desc_virt += MMBI_DESC_SIZE_CHANNEL;
		}
	}

	val = 0;
	for (chan_idx = 0; chan_idx < mmbi->num_of_channels; chan_idx++) {
		chan = &mmbi->chan_desc[chan_idx];
		mmbi_channel_state_handler(chan, mmbi->desc_virt);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmbi_instance_init_host);
