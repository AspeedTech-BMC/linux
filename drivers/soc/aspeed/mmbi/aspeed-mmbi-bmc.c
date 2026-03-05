// SPDX-License-Identifier: GPL-2.0+
/* Implements MMBI protocol for BMC side with VPSCB buffer type
 * Copyright 2026 Aspeed Technology Inc.
 */
#include <linux/compiler_types.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/printk.h>
#include "aspeed-mmbi.h"
#include "aspeed-mmbi-bmc.h"

#define MMBI_PUT_U32_MSB(base, off, val)                          \
	do {                                                      \
		for (int __i = 0; __i < 4; __i++) {               \
			iowrite8((u8)((val) >> (24 - (__i * 8))), \
				 (base) + ((off)++));             \
		}                                                 \
	} while (0)

static void mmbi_channel_state_handler(struct mmbi_chan_desc *chan, u8 __iomem *desc_virt)
{
	enum mmbi_state prev_state, cur_state;
	struct mmbi_buf_vpscb *vpscb;
	u8 __iomem *host_ros_virt, *host_rws_virt;
	u32 unhandled_len, avail_len;

	prev_state = chan->state;

	vpscb = &chan->buffer_desc;
	host_ros_virt = desc_virt + vpscb->h_ros_p;
	host_rws_virt = desc_virt + vpscb->h_rws_p;
	mmbi_channel_state_update(chan, desc_virt);
	cur_state = chan->state;

	switch (cur_state) {
	case INIT_MISMATCH:
		memset_io(host_ros_virt, 0, 8);
		memset_io(host_rws_virt, 0, 8);
		mmbi_set_up(host_ros_virt);
		chan->state = INIT_COMPLETED;
		break;
	case NORMAL_RUNTIME:
		mmbi_set_ready(host_ros_virt);

		unhandled_len = mmbi_channel_unhandled_length(host_ros_virt, host_rws_virt, chan->h2b_l);
		if (unhandled_len >= MMBI_PKT_MIN_SIZE)
			chan->rx_ready = true;
		else
			chan->rx_ready = false;
		avail_len = mmbi_channel_avail_length(host_rws_virt, host_ros_virt, chan->b2h_l);
		if (avail_len < MMBI_PKT_MIN_SIZE)
			chan->tx_ready = false;
		else
			chan->tx_ready = true;

		break;
	case RESET_REQ_BY_HOST:
		mmbi_clr_ready(host_ros_virt);
		mmbi_set_rst(host_ros_virt);
		chan->state = RESET_ACKED;
		// TODO: consume all pending data from host and then trigger initialization
		break;
	case RESET_ACKED:
		mmbi_clr_up(host_ros_virt);
		chan->state = TRANS_TO_INIT;
		memset_io(host_ros_virt, 0, 8);
		memset_io(host_rws_virt, 0, 8);
		mmbi_set_up(host_ros_virt);
		chan->state = INIT_COMPLETED;
	default:
		/* other states do not require action from BMC side */
		break;
	}

	return;
}

static int mmbi_buffer_init_vpscb_bmc(u8 __iomem *buf_virt, struct mmbi_buf_vpscb *buf_desc)
{
	u32 offset;

	offset = 0;

	iowrite8(MMBI_BUFFER_TYPE_VPSCB & MMBI_BUF_TYPE_MASK, buf_virt + offset++);
	iowrite8(buf_desc->host_int_val_ch & MMBI_INT_VAL_MASK, buf_virt + offset++);
	iowrite8(buf_desc->bmc_int_val_ch & MMBI_INT_VAL_MASK, buf_virt + offset++);

	memset_io(buf_virt + offset, 0, 5);
	offset += 5; /* reserved bytes */

	MMBI_PUT_U32_MSB(buf_virt, offset, MMBI_BUF_ADDR_ALIGN(buf_desc->h_ros_p));
	MMBI_PUT_U32_MSB(buf_virt, offset, MMBI_BUF_ADDR_ALIGN(buf_desc->h_rws_p));

	pr_info("%s:   h_ros_p=0x%x, h_rws_p=0x%x\n", __func__,
		buf_desc->h_ros_p, buf_desc->h_rws_p);
	return 0;
}

static int mmbi_channel_init_bmc(u8 __iomem *desc_virt, struct mmbi_chan_desc *chan_desc)
{
	u32 offset = 0;
	struct mmbi_buf_vpscb *buffer_desc;

	chan_desc->peer_ready = false;
	MMBI_PUT_U32_MSB(desc_virt, offset, MMBI_BUF_ADDR_ALIGN(chan_desc->b2h_ba_offset));
	MMBI_PUT_U32_MSB(desc_virt, offset, MMBI_BUF_ADDR_ALIGN(chan_desc->h2b_ba_offset));
	MMBI_PUT_U32_MSB(desc_virt, offset, chan_desc->b2h_l);
	MMBI_PUT_U32_MSB(desc_virt, offset, chan_desc->h2b_l);

	pr_info("%s:   b2h_ba_offset=0x%x, h2b_ba_offset=0x%x\n", __func__,
		chan_desc->b2h_ba_offset, chan_desc->h2b_ba_offset);
	pr_info("%s:   b2h_l=0x%x, h2b_l=0x%x\n", __func__, chan_desc->b2h_l,
		chan_desc->h2b_l);
	pr_info("%s:   buffer_type=%u\n", __func__, chan_desc->buffer_type);

	if (chan_desc->buffer_type == MMBI_BUFFER_TYPE_VPSCB) {
		buffer_desc = &chan_desc->buffer_desc;
		return mmbi_buffer_init_vpscb_bmc(desc_virt + offset, buffer_desc);
	}

	pr_err("\t%s: unsupported buffer type %d\n", __func__,
	       chan_desc->buffer_type);
	return -EINVAL;
}

void mmbi_channel_irq_bmc(struct mmbi_chan_desc *chan)
{
	mmbi_channel_state_handler(chan, chan->mmbi->desc_virt);
	if (chan->rx_ready)
		wake_up_interruptible(&chan->rx_wait);
	if (chan->tx_ready)
		wake_up_interruptible(&chan->tx_wait);

}
EXPORT_SYMBOL_GPL(mmbi_channel_irq_bmc);

int mmbi_instance_init_bmc(struct mmbi_ins_desc *mmbi)
{
	int rc, offset;
	u32 i;
	u8 __iomem *desc_virt;
	struct mmbi_buf_vpscb *vpscb;

	if (mmbi->num_of_channels > MMBI_MAX_CHANNELS) {
		dev_err(mmbi->dev, "%s: invalid number of instances %d\n",
			__func__, mmbi->num_of_channels);
		return -EINVAL;
	}

	desc_virt = mmbi->desc_virt;
	i = FIELD_PREP((u32)MMBI_NOI_MASK, mmbi->num_of_channels);
	i |= FIELD_PREP((u32)MMBI_OS_USE_MASK, mmbi->os_use);
	dev_info(mmbi->dev, "%s: MMBI version 0x%x, os_use %u, num_of_channels %u i %u\n",
		 __func__, mmbi->mmbi_version, mmbi->os_use, mmbi->num_of_channels, i);

	memcpy_toio(desc_virt, MMBI_SIGNATURE, sizeof(MMBI_SIGNATURE) - 1);
	desc_virt += sizeof(MMBI_SIGNATURE) - 1;

	iowrite8(mmbi->mmbi_version & MMBI_VERSION_MASK, desc_virt++);
	iowrite8(i, desc_virt++);

	mmbi->chan_desc[0].state = INIT_IN_PROGRESS;
	mmbi->chan_desc[0].index = 0;
	mmbi->chan_desc[0].mmbi = mmbi;
	rc = mmbi_channel_init_bmc(desc_virt, &mmbi->chan_desc[0]);
	if (rc) {
		dev_err(mmbi->dev, "%s: failed to init channel 0\n", __func__);
		return rc;
	}
	desc_virt += MMBI_DESC_SIZE_CHANNEL;

	iowrite8(mmbi->host_int_type & MMBI_INT_TYPE_MASK, desc_virt++);
	iowrite8(mmbi->host_int_location, desc_virt++);

	memset_io(desc_virt, 0, 3);
	desc_virt += 3; /* reserved bytes */
	iowrite8(mmbi->host_int_value, desc_virt++);
	iowrite8(mmbi->bmc_int_type & MMBI_INT_TYPE_MASK, desc_virt++);
	offset = desc_virt - mmbi->desc_virt;
	desc_virt += sizeof(mmbi->bmc_int_location);
	MMBI_PUT_U32_MSB(mmbi->desc_virt, offset, mmbi->bmc_int_location);

	memset_io(desc_virt, 0, 4);
	desc_virt += 4; /* reserved bytes */

	iowrite8(mmbi->bmc_int_value, desc_virt++);

	memset_io(desc_virt, 0, 8);
	desc_virt += 8; /* reserved bytes */

	if (mmbi->num_of_channels > 1) {
		for (i = 1; i < mmbi->num_of_channels; i++) {
			mmbi->chan_desc[i].state = INIT_IN_PROGRESS;
			mmbi->chan_desc[i].index = i;
			mmbi->chan_desc[i].mmbi = mmbi;
			rc = mmbi_channel_init_bmc(desc_virt, &mmbi->chan_desc[i]);
			if (rc) {
				dev_err(mmbi->dev, "%s: failed to init channel %d\n", __func__, i);
				return rc;
			}
			desc_virt += MMBI_DESC_SIZE_CHANNEL;
		}
	}

	for (i = 0; i < mmbi->num_of_channels; i++) {
		vpscb = &mmbi->chan_desc[i].buffer_desc;
		memset_io(mmbi->desc_virt + vpscb->h_ros_p, 0, 8);
		memset_io(mmbi->desc_virt + vpscb->h_rws_p, 0, 8);

		mmbi_set_up(mmbi->desc_virt + vpscb->h_ros_p);
		mmbi_set_ready(mmbi->desc_virt + vpscb->h_ros_p);
		mmbi->chan_desc[i].state = INIT_COMPLETED;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(mmbi_instance_init_bmc);
