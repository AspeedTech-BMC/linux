// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) ASPEED Technology Inc.
 */
#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/compiler_attributes.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gfp_types.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <dt-bindings/mmbi/protocols.h>

#include "ast2700-espi.h"
#include "ast2700-intel-espi-mmbi.h"

#define HOST_READ_SCI_STATUS_BIT(x)		((x) & ESPI_CH1_EVT0_SCI_EVT)
#define HOST_READ_RESET_REQ_BIT(x)		((x) & BIT(0))
#define HOST_READ_IF_UP_BIT(x)			((x) & BIT(1))
#define HOST_READ_RDY_BIT(x)			((x) & BIT(0))
#define HOST_READ_BUF_PTR(x)			((x) & GENMASK(31, 2))

/* LPC register for SCI interrupt */
#define AST_LPC_SWCR0300 0x180
#define LPC_BMC_TRIG_WAKEUP_EVT_STS BIT(8)

#define AST_LPC_SWCR0704 0x184
#define LPC_BMC_TRIG_WAKEUP_EVT_EN BIT(8)

#define AST_LPC_SWCR0B08 0x188
#define LPC_BMC_TRIG_WAKEUP_EVT BIT(6)

#define AST_LPC_ACPIB3B0 0x1A8
#define LPC_BMC_TRIG_SCI_EVT_STS BIT(8)

#define AST_LPC_ACPIB7B4 0x1AC
#define LPC_BMC_TRIG_SCI_EVT_EN BIT(8)

#define MMBI_SIGNATURE_STRING			"#MMBI$"
#define MMBI_BUF_TYPE_VPSCB_V1			0x01 /* Variable Packet Size Circular Buffer */
#define MMBI_MEMORY_SIZE				SZ_64M
#define MMBI_MAX_SUPPORT_PROTOCOLS		5
#define MMBI_HDR_LEN_MASK				GENMASK(23, 2)
#define MMBI_HDR_PADDING_MASK			GENMASK(1, 0)
#define MMBI_HDR_TYPE_MASK				GENMASK(27, 24)
#define AST2700_ESPI_MMBI_MAX_CHANNELS	8

enum ast2700_espi_mmbi_state {				/* B_U B_R H_U H_R */
	ESPI_MMBI_INIT_IN_PROGRESS	= 0x00,		/* 0   0   0   0   */
	ESPI_MMBI_INIT_COMPLETED	= 0x08,		/* 1   0   0   0   */
	ESPI_MMBI_NORMAL_RUNTIME	= 0x0A,		/* 1   0   1   0   */
	ESPI_MMBI_RESET_REQ_BY_BMC	= 0x0E,		/* 1   1   1   0   */
	ESPI_MMBI_RESET_REQ_BY_HOST	= 0x0B,		/* 1   0   1   1   */
	ESPI_MMBI_RESET_ACKED		= 0x0F,		/* 1   1   1   1   */
	ESPI_MMBI_TRANS_TO_INIT		= 0x07,		/* 0   1   1   1   */
	ESPI_MMBI_INIT_MISMATCH		= 0x09,		/* 1   0   0   1   */
	ESPI_MMBI_POWER_UP_OR_ERROR	= 0x10,		/*  other states   */
};

// addresses below, if not specified, are all virtual addresses
struct aspeed_mmbi_host_rws {
	u32 host_rst_req: 1;
	u32 host_if_up: 1;
	u32 host_buf_wr_ptr: 30;	/* Address that host should write data into */
	u32 host_rdy: 1;
	u32 rsvd: 1;
	u32 bmc_buf_rd_ptr: 30;		/* Address that host should read data from */
};

struct aspeed_mmbi_host_ros {
	u32 bmc_rst_req: 1;
	u32 bmc_if_up: 1;
	u32 bmc_buf_wr_ptr: 30;		/* Address that BMC should write data into */
	u32 bmc_rdy: 1;
	u32 rsvd: 1;
	u32 host_buf_rd_ptr: 30;	/* Address that BMC should read data from */
};

/* MMBI Variable Packet Size Circular Buffer(VPSCB) v1 descriptor
 * memory addresses are all offset
 */
struct aspeed_mmbi_vpscb_desc {
	u32 host_ros_pointer;	/* Host ROS Offset */
	u32 host_rws_pointer;	/* Host RWS Offset */
	u8 host_int_type;		/* how BMC interrupt host */
	u8 host_int_location;	/* which channel BMC interrupt host given int_type */
	u8 rsvd1[3];
	u8 host_int_val;		/* what BMC will write to interrupt host */
	u8 bmc_int_type;		/* how host interrupt BMC */
	u32 bmc_int_location;	/* which address host should write to interrupt BMC */
	u8 rsvd2[4];
	u8 bmc_int_val;			/* what host will write to interrupt BMC */
} __packed;

/* memory addresses are all physical */
struct aspeed_mmbi_desc {
	u8 mmbi_signature[6];
	u8 mmbi_version;
	u8 os_use;
	u32 b2h_buf_base_addr;
	u32 h2b_buf_base_addr;
	u32 b2h_buf_len;
	u32 h2b_buf_len;
	u8 buf_type;
	u8 rsvd1[7];
	struct aspeed_mmbi_vpscb_desc buf_desc;
	u8 rsvd2[8];
};

struct aspeed_mmbi_protocol_dev {
	struct miscdevice miscdev;
	struct aspeed_espi_mmbi_channel *channel;
	u8 protocol_type;
	bool data_available;
	/*
	 * If user space application is opened for read, then only process
	 * the data and copy to userspace. Otherwise, discard the command and
	 * process the remaining commands (can be different protocol type)
	 */
	bool process_data;
	wait_queue_head_t queue;
};

struct aspeed_espi_mmbi_channel {
	u8 *desc;
	u8 *host_ros;
	u8 *host_rws;
	u8 *b2h_circular_buf;
	u32 b2h_circular_buf_size;
	u8 *h2b_circular_buf;
	u32 h2b_circular_buf_size;
	struct ast2700_espi_mmbi *priv;

	struct aspeed_mmbi_protocol_dev protocols[MMBI_MAX_SUPPORT_PROTOCOLS];

	enum ast2700_espi_mmbi_state state;

	bool enabled;
	int chann_idx;
};

struct ast2700_espi_mmbi {
	struct regmap *mmbi_map;
	struct regmap *espi_map;
	struct regmap *lpc_map;
	struct device *dev;
	struct aspeed_espi_mmbi_channel channels[AST2700_ESPI_MMBI_MAX_CHANNELS];
	phys_addr_t host_map_addr;
	dma_addr_t mmbi_phys_addr;
	u8 *mmbi_virt_addr;
	resource_size_t mmbi_size;
	u32 mmbi_instances;
};

struct ast2700_espi_mmbi_hdr {
	u32 data;
};

static void raise_sci_interrupt(struct aspeed_espi_mmbi_channel *channel)
{
	u32 val;
	int retry;
	struct regmap *lpc_regmap = channel->priv->lpc_map;

	dev_dbg(channel->priv->dev, "Raising SCI interrupt...\n");

	regmap_write_bits(lpc_regmap, AST_LPC_ACPIB7B4, LPC_BMC_TRIG_SCI_EVT_EN,
			  LPC_BMC_TRIG_SCI_EVT_EN);

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0704,
			  LPC_BMC_TRIG_WAKEUP_EVT_EN,
			  LPC_BMC_TRIG_WAKEUP_EVT_EN);

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0B08, LPC_BMC_TRIG_WAKEUP_EVT,
			  LPC_BMC_TRIG_WAKEUP_EVT);

	/*
	 * Just asserting the SCI VW will trigger the SCI event continuosly.
	 * So BMC must deassert SCI VW to avoid it.
	 * ESPI098[24] reading will confirm Host read data or not.
	 * - 0 means host read the data
	 * - 1 means host not yet read data, so retry with 1us delay.
	 */
	retry = 30;
	while (retry) {
		if (regmap_read(channel->priv->espi_map, ESPI_CH1_EVT0, &val)) {
			dev_err(channel->priv->dev, "Unable to read ESPI210\n");
			break;
		}

		if (HOST_READ_SCI_STATUS_BIT(val) == 0)
			break;

		retry--;
		dev_dbg(channel->priv->dev,
			"Host SCI handler not invoked(ESPI210: 0x%0x), so retry(%d) after 1us...\n",
			val, retry);
		udelay(1);
	}

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0300,
			  LPC_BMC_TRIG_WAKEUP_EVT_STS,
			  LPC_BMC_TRIG_WAKEUP_EVT_STS);

	regmap_write_bits(lpc_regmap, AST_LPC_ACPIB3B0,
			  LPC_BMC_TRIG_SCI_EVT_STS,
			  LPC_BMC_TRIG_SCI_EVT_STS);
}

static void raise_missing_sci(struct aspeed_espi_mmbi_channel *channel)
{
	int chan_idx;
	u32 h_rwp1;
	u32 b2h_rp, b2h_wp;
	struct aspeed_mmbi_host_ros *h_ros_ptr;

	chan_idx = channel->chann_idx;
	if (regmap_read(channel->priv->mmbi_map, ESPI_MMBI_HOST_READ_RWP1(chan_idx), &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP1\n");
		return;
	}
	if (!HOST_READ_RDY_BIT(h_rwp1)) {
		// Host is not ready, no point in raising the SCI
		return;
	}

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	b2h_wp = h_ros_ptr->bmc_buf_wr_ptr << 2;
	b2h_rp = HOST_READ_BUF_PTR(h_rwp1);

	if (b2h_wp == b2h_rp) {
		// Host has read all outstanding SCI data,
		// Do not raise another SCI.
		return;
	}

	dev_dbg(channel->priv->dev,
		"Host not read the data yet, so rising SCI interrupt again...\n");
	raise_sci_interrupt(channel);
}

static int ast2700_espi_mmbi_parse_hdr(struct aspeed_espi_mmbi_channel *channel,
					u32 *data_length, u8 *type,
					u32 *unread_data_len, u8 *padding)
{
	u32 h2b_rp, h2b_wp;
	struct ast2700_espi_mmbi_hdr header;
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	struct aspeed_mmbi_host_rws *h_rws_ptr;

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	h_rws_ptr = (struct aspeed_mmbi_host_rws *)channel->host_rws;

	h2b_wp = h_rws_ptr->host_buf_wr_ptr << 2;
	h2b_rp = h_ros_ptr->host_buf_rd_ptr << 2;

	if (h2b_wp >= h2b_rp)
		*unread_data_len = h2b_wp - h2b_rp;
	else
		*unread_data_len = channel->h2b_circular_buf_size - h2b_rp + h2b_wp;

	if (*unread_data_len < sizeof(struct ast2700_espi_mmbi_hdr)) {
		dev_dbg(channel->priv->dev, "No data to read(%d -%d)\n", h2b_wp,
			h2b_rp);
		return -EAGAIN;
	}

	dev_dbg(channel->priv->dev, "READ MMBI header from: 0x%0lx\n",
		(ssize_t)(channel->h2b_circular_buf + h2b_rp));

	if (h2b_rp + sizeof(struct ast2700_espi_mmbi_hdr) <= channel->h2b_circular_buf_size) {
		memcpy_fromio((void *)&header, channel->h2b_circular_buf + h2b_rp,
			       sizeof(struct ast2700_espi_mmbi_hdr));
	} else {
		ssize_t chunk_len = channel->h2b_circular_buf_size - h2b_rp;

		memcpy_fromio((void *)&header, channel->h2b_circular_buf + h2b_rp,
			      chunk_len);
		memcpy_fromio(((u8 *)&header) + chunk_len, channel->h2b_circular_buf,
			      sizeof(struct ast2700_espi_mmbi_hdr) - chunk_len);
	}

	*padding = FIELD_GET(MMBI_HDR_PADDING_MASK, header.data);
	*data_length = FIELD_GET(MMBI_HDR_LEN_MASK, header.data) * 4 - *padding;
	*type = FIELD_GET(MMBI_HDR_TYPE_MASK, header.data);

	return 0;
}

static void ast2700_espi_mmbi_update_host_ros(struct aspeed_espi_mmbi_channel *channel,
					  u32 read_len, u32 write_len)
{
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	u32 rd_ptr, wr_ptr;

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	rd_ptr = h_ros_ptr->host_buf_rd_ptr << 2;
	wr_ptr = h_ros_ptr->bmc_buf_wr_ptr << 2;

	/* Advance the B2H CB offset for next write */
	if ((wr_ptr + write_len) <= channel->b2h_circular_buf_size)
		wr_ptr += write_len;
	else
		wr_ptr = wr_ptr + write_len - channel->b2h_circular_buf_size;

	/* Advance the H2B CB offset till where BMC read data */
	if ((rd_ptr + read_len) <= channel->h2b_circular_buf_size)
		rd_ptr += read_len;
	else
		rd_ptr = rd_ptr + read_len - channel->h2b_circular_buf_size;

	h_ros_ptr->bmc_buf_wr_ptr = wr_ptr >> 2;
	h_ros_ptr->host_buf_rd_ptr = rd_ptr >> 2;

	dev_dbg(channel->priv->dev, "Updating HROS - h2b_rp: 0x%0x, b2h_wp: 0x%0x\n",
		rd_ptr, wr_ptr);

	if (write_len != 0)
		raise_sci_interrupt(channel);
}

static void ast2700_espi_mmbi_host_ifup_handler(struct aspeed_espi_mmbi_channel *channel)
{
	struct aspeed_mmbi_host_ros *hros;

	dev_info(channel->priv->dev,
		 "Handle Host interface up request on MMBI channel(%d), state %d\n",
		 channel->chann_idx, channel->state);

	switch (channel->state) {
	case ESPI_MMBI_INIT_COMPLETED:
		hros = (struct aspeed_mmbi_host_ros *)channel->host_ros;
		hros->bmc_rdy = 1;
		channel->state = ESPI_MMBI_NORMAL_RUNTIME;
		break;
	default:
		dev_err(channel->priv->dev,
			"MMBI channel(%d) host state error\n",
			channel->chann_idx);
		channel->state = ESPI_MMBI_POWER_UP_OR_ERROR;
		break;
	}
}

static void ast2700_espi_mmbi_host_reset_handler(struct aspeed_espi_mmbi_channel *channel)
{
	dev_info(channel->priv->dev,
			"Handle Host reset request on MMBI channel(%d), state %d\n",
			channel->chann_idx, channel->state);

	switch (channel->state) {
	case ESPI_MMBI_INIT_COMPLETED:
		memset(channel->host_ros, 0, sizeof(struct aspeed_mmbi_host_ros));
		dev_err(channel->priv->dev,
			"Initialization mismatch on MMBI channel(%d)\n",
			channel->chann_idx);
		channel->state = ESPI_MMBI_INIT_MISMATCH;
		break;
	case ESPI_MMBI_NORMAL_RUNTIME:
		/*  Host requested for MMBI buffer reset */
		memset(channel->host_ros, 0, sizeof(struct aspeed_mmbi_host_ros));
		channel->state = ESPI_MMBI_INIT_IN_PROGRESS;

		dev_info(channel->priv->dev,
			"Handle Host reset request on MMBI channel(%d)\n",
			channel->chann_idx);
		break;
	default:
		dev_err(channel->priv->dev,
			"Invalid state(%d) on MMBI channel(%d) to handle Host reset request\n",
			channel->state, channel->chann_idx);
		channel->state = ESPI_MMBI_POWER_UP_OR_ERROR;
		break;
	}
}

static void ast2700_espi_mmbi_host_write_handler(struct aspeed_espi_mmbi_channel *channel)
{
	u32 data_length, unread_data_len;
	u8 type, padding;
	int rc, i;
	bool handled = false;

	dev_info(channel->priv->dev,
		 "Handle Host write on MMBI channel(%d)\n",
		 channel->chann_idx);

	rc = ast2700_espi_mmbi_parse_hdr(channel, &data_length, &type, &unread_data_len, &padding);
	if (rc) {
		dev_dbg(channel->priv->dev,
			"Failed to parse MMBI header on MMBI channel(%d)\n",
			channel->chann_idx);
		return;
	}

	for (i = 0; i < MMBI_MAX_SUPPORT_PROTOCOLS; i++) {
		if (channel->protocols[i].protocol_type == type) {
			if (channel->protocols[i].process_data) {
				handled = true;
				channel->protocols[i].data_available = true;
				wake_up_interruptible(&channel->protocols[i].queue);
			} else {
				ast2700_espi_mmbi_update_host_ros(channel,
					data_length +
						sizeof(struct ast2700_espi_mmbi_hdr) +
						padding,
					0);
			}
		}
	}

	if (!handled) {
		dev_warn(channel->priv->dev,
			 "No handler for protocol type %d on MMBI channel(%d), discard data\n",
			 type, channel->chann_idx);
		ast2700_espi_mmbi_update_host_ros(channel, data_length +
						sizeof(struct ast2700_espi_mmbi_hdr) +
						padding, 0);
		raise_sci_interrupt(channel);
	}

	dev_dbg(channel->priv->dev, "MMBI header parsed on MMBI channel(%d)\n",
		channel->chann_idx);
	dev_dbg(channel->priv->dev, "data len %d unread %d, type %d, padding %d\n",
		data_length, unread_data_len, type, padding);
}

static void ast2700_espi_mmbi_host_rwp0_handler(struct aspeed_espi_mmbi_channel *channel)
{
	int chan_idx;
	u32 h_rws_cur, h_rws_prev;

	memcpy_fromio(&h_rws_prev, channel->host_rws, sizeof(h_rws_prev));
	chan_idx = channel->chann_idx;
	if (regmap_read(channel->priv->mmbi_map, ESPI_MMBI_HOST_READ_RWP0(chan_idx), &h_rws_cur)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP0\n");
		return;
	}

	dev_dbg(channel->priv->dev, "Host RWP0 changed on MMBI channel(%d): 0x%08x -> 0x%08x\n",
		 channel->chann_idx, h_rws_prev, h_rws_cur);

	memcpy_fromio(channel->host_rws, &h_rws_cur, sizeof(h_rws_cur));
	if (HOST_READ_RESET_REQ_BIT(h_rws_cur) & ~HOST_READ_RESET_REQ_BIT(h_rws_prev))
		ast2700_espi_mmbi_host_reset_handler(channel);
	else if (HOST_READ_IF_UP_BIT(h_rws_cur) & ~HOST_READ_IF_UP_BIT(h_rws_prev))
		ast2700_espi_mmbi_host_ifup_handler(channel);
	else
		ast2700_espi_mmbi_host_write_handler(channel);
}

static void ast2700_espi_mmbi_host_rwp1_handler(struct aspeed_espi_mmbi_channel *channel)
{
	int chan_idx;
	u32 h_rws_cur, h_rws_prev;

	memcpy_fromio(&h_rws_prev, channel->host_rws + 4, sizeof(h_rws_prev));
	chan_idx = channel->chann_idx;
	if (regmap_read(channel->priv->mmbi_map, ESPI_MMBI_HOST_READ_RWP1(chan_idx), &h_rws_cur)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP1\n");
		return;
	}

	dev_dbg(channel->priv->dev, "Host RWP1 changed on MMBI channel(%d): 0x%08x -> 0x%08x\n",
		 channel->chann_idx, h_rws_prev, h_rws_cur);

	memcpy_fromio(channel->host_rws + 4, &h_rws_cur, sizeof(h_rws_cur));
}

static void send_bmc_up_request(struct aspeed_espi_mmbi_channel *channel)
{
	struct aspeed_mmbi_host_ros *hros;

	if (!channel->enabled || channel->state != ESPI_MMBI_INIT_IN_PROGRESS) {
		dev_err(channel->priv->dev,
			"Invalid state(%d) on MMBI channel(%d) to send BMC interface up request\n",
			channel->state, channel->chann_idx);
		return;
	}

	hros = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	hros->bmc_if_up = 1;
	channel->state = ESPI_MMBI_INIT_COMPLETED;

	dev_info(channel->priv->dev,
		 "Send BMC interface up request on MMBI channel(%d)\n",
		 channel->chann_idx);

	raise_sci_interrupt(channel);
}

static int send_bmc_reset_request(struct aspeed_espi_mmbi_channel *channel)
{
	struct aspeed_mmbi_host_ros *hros;

	hros = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	if (!channel->enabled) {
		dev_err(channel->priv->dev,
			"Invalid state(%d) on MMBI channel(%d) to send BMC reset request\n",
			channel->enabled, channel->chann_idx);
		return -EINVAL;
	}
	memset(channel->host_ros, 0, sizeof(struct aspeed_mmbi_host_ros));
	hros->bmc_rst_req = 1;
	raise_sci_interrupt(channel);

	return 0;
}

static struct aspeed_mmbi_protocol_dev *file_aspeed_espi_mmbi(struct file *file)
{
	return container_of(file->private_data, struct aspeed_mmbi_protocol_dev,
			    miscdev);
}

static int ast2700_espi_mmbi_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ast2700_espi_mmbi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int ast2700_espi_mmbi_poll(struct file *filp, poll_table *wait)
{
	struct aspeed_mmbi_protocol_dev *protocol = file_aspeed_espi_mmbi(filp);

	poll_wait(filp, &protocol->queue, wait);

	return protocol->data_available ? POLLIN : 0;
}

static int get_mmbi_config(struct aspeed_espi_mmbi_channel *channel, void __user *userbuf)
{
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	struct aspeed_mmbi_host_rws *h_rws_ptr;
	struct aspeed_mmbi_get_config get_conf;

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	h_rws_ptr = (struct aspeed_mmbi_host_rws *)channel->host_rws;

	get_conf.h_rdy = h_rws_ptr->host_rdy ? true : false;
	get_conf.h2b_wp = h_rws_ptr->host_buf_wr_ptr << 2;
	get_conf.b2h_rp = h_rws_ptr->bmc_buf_rd_ptr << 2;
	get_conf.h2b_rp = h_ros_ptr->host_buf_rd_ptr << 2;
	get_conf.b2h_wp = h_ros_ptr->bmc_buf_wr_ptr << 2;

	if (copy_to_user(userbuf, &get_conf, sizeof(get_conf))) {
		dev_err(channel->priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int get_b2h_empty_space(struct aspeed_espi_mmbi_channel *channel,
			       ssize_t *length)
{
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	struct aspeed_mmbi_host_rws *h_rws_ptr;
	ssize_t avail_buf_len;

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	h_rws_ptr = (struct aspeed_mmbi_host_rws *)channel->host_rws;

	if (h_ros_ptr->bmc_buf_wr_ptr >= h_rws_ptr->bmc_buf_rd_ptr)
		avail_buf_len = channel->b2h_circular_buf_size - (h_ros_ptr->bmc_buf_wr_ptr << 2) +
				(h_rws_ptr->bmc_buf_rd_ptr << 2);
	else
		avail_buf_len = (h_rws_ptr->bmc_buf_rd_ptr << 2) - (h_ros_ptr->bmc_buf_wr_ptr << 2);

	dev_dbg(channel->priv->dev, "B2H buffer empty space: %ld\n", avail_buf_len);

	*length = avail_buf_len;

	return 0;
}

static long ast2700_espi_mmbi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_mmbi_protocol_dev *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_espi_mmbi_channel *channel = protocol->channel;
	void __user *userbuf = (void __user *)arg;
	ssize_t length;
	int ret;

	switch (cmd) {
	case ASPEED_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE:
		ret = get_b2h_empty_space(channel, &length);
		if (ret == 0 && copy_to_user(userbuf, &length, sizeof(length))) {
			dev_err(channel->priv->dev, "copy to user failed\n");
			ret = -EFAULT;
		}
		break;

	case ASPEED_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST:
		ret = send_bmc_reset_request(channel);
		break;

	case ASPEED_MMBI_CTRL_IOCTL_GET_CONFIG:
		ret = get_mmbi_config(channel, userbuf);
		break;

	default:
		dev_err(channel->priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}

	return ret;
}

static ssize_t ast2700_espi_mmbi_read(struct file *filp, char *buff, size_t count,
			 loff_t *offp)
{
	struct aspeed_mmbi_protocol_dev *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_espi_mmbi_channel *channel = protocol->channel;
	struct ast2700_espi_mmbi *priv = channel->priv;
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	u32 data_length, unread_data_len;
	u32 buf_rd_offset;
	u8 type, padding;
	ssize_t rd_offset, rd_len;
	ssize_t ret;

	protocol->process_data = true;
	if (!protocol->data_available && (filp->f_flags & O_NONBLOCK)) {
		raise_missing_sci(channel);
		return -EAGAIN;
	}

	dev_dbg(priv->dev, "%s: count:%ld, Type: %d\n", __func__, count,
		protocol->protocol_type);

	ret = wait_event_interruptible(protocol->queue, protocol->data_available);
	if (ret == -ERESTARTSYS) {
		ret = -EINTR;
		goto err_out;
	}

	ret = ast2700_espi_mmbi_parse_hdr(channel, &data_length, &type, &unread_data_len, &padding);
	if (ret) {
		dev_err(priv->dev, "%s: Failed to parse MMBI header\n", __func__);
		goto err_out;
	}

	dev_dbg(priv->dev, "%s: PKT Length: %d, Type: %d, Unread Data Length: %d, Padding: %d\n",
		__func__, data_length, type, unread_data_len, padding);

	if (data_length > count) {
		dev_err(priv->dev, "%s: Buffer too small, need %d\n", __func__,
			data_length);
		ret = -EMSGSIZE;
		/* discard this packet */
		ast2700_espi_mmbi_update_host_ros(channel, data_length + sizeof(struct ast2700_espi_mmbi_hdr) + padding, 0);
		goto err_out;
	}

	if (type != protocol->protocol_type) {
		dev_err(priv->dev, "%s: Protocol type mismatch, expected %d\n",
			__func__, protocol->protocol_type);
		ret = -EPROTO;
		goto err_out;
	}

	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;
	buf_rd_offset = h_ros_ptr->host_buf_rd_ptr << 2;
	if (buf_rd_offset + sizeof(struct ast2700_espi_mmbi_hdr) <= channel->h2b_circular_buf_size)
		rd_offset = buf_rd_offset + sizeof(struct ast2700_espi_mmbi_hdr);
	else
		rd_offset = (buf_rd_offset + sizeof(struct ast2700_espi_mmbi_hdr)) - channel->h2b_circular_buf_size;

	rd_len = data_length;

	dev_dbg(priv->dev, "%s: Read from offset: 0x%0lx, length: %ld\n",
		__func__, rd_offset, rd_len);

	if (unread_data_len < rd_len + sizeof(struct ast2700_espi_mmbi_hdr)) {
		dev_err(priv->dev, "%s: Not enough data to read, need %ld\n",
			__func__, rd_len + sizeof(struct ast2700_espi_mmbi_hdr));
		ret = -EAGAIN;
		goto err_out;
	}

	if ((channel->h2b_circular_buf_size - rd_offset) >= rd_len) {
		if (copy_to_user(buff, channel->h2b_circular_buf + rd_offset, rd_len)) {
			dev_err(priv->dev, "%s: copy to user failed\n", __func__);
			ret = -EFAULT;
			goto err_out;
		}
	} else {
		ssize_t chunk_len = channel->h2b_circular_buf_size - rd_offset;

		if (copy_to_user(buff, channel->h2b_circular_buf + rd_offset, chunk_len) ||
		    copy_to_user(buff + chunk_len, channel->h2b_circular_buf,
				 rd_len - chunk_len)) {
			dev_err(priv->dev, "%s: copy to user failed\n", __func__);
			ret = -EFAULT;
			goto err_out;
		}
	}

	*offp += rd_len;
	ret = rd_len;
	ast2700_espi_mmbi_update_host_ros(channel, rd_len + sizeof(struct ast2700_espi_mmbi_hdr) + padding, 0);
	dev_dbg(priv->dev, "%s: Read %ld bytes from MMBI\n", __func__, rd_len);
err_out:
	/*
	 * Raise the missing SCI's by checking pointer for host
	 * read acknowledgment. This will work around the Missing
	 * SCI bug on host side. *
	 */
	dev_warn(priv->dev, "%s: Check and raise missing SCI\n", __func__);
	raise_missing_sci(channel);

	protocol->data_available = false;

	ast2700_espi_mmbi_host_write_handler(channel);

	return ret;
}

static ssize_t ast2700_espi_mmbi_write(struct file *filp, const char *buffer, size_t len,
			  loff_t *offp)
{
	struct aspeed_mmbi_protocol_dev *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_espi_mmbi_channel *channel = protocol->channel;
	struct ast2700_espi_mmbi *priv = channel->priv;
	struct aspeed_mmbi_host_rws *h_rws_ptr;
	struct aspeed_mmbi_host_ros *h_ros_ptr;
	struct ast2700_espi_mmbi_hdr header;

	ssize_t wr_offset, avail_buf_len, padding, chunk_len;

	h_rws_ptr = (struct aspeed_mmbi_host_rws *)channel->host_rws;
	h_ros_ptr = (struct aspeed_mmbi_host_ros *)channel->host_ros;

	dev_dbg(priv->dev, "%s: length:%ld , type: %d\n", __func__, len,
		protocol->protocol_type);

	if (!h_rws_ptr->host_if_up) {
		dev_err(priv->dev, "%s: Host interface not up\n", __func__);
		return -EAGAIN;
	}

	if (get_b2h_empty_space(channel, &avail_buf_len)) {
		dev_err(priv->dev, "%s: Failed to get B2H empty space\n", __func__);
		return -EFAULT;
	}

	if ((len + sizeof(struct ast2700_espi_mmbi_hdr)) > avail_buf_len) {
		dev_err(priv->dev, "%s: No enough space in B2H buffer, need %ld but have %ld\n",
			__func__, len + sizeof(struct ast2700_espi_mmbi_hdr), avail_buf_len);
		return -ENOSPC;
	}
	padding = (4 - (len % 4)) % 4;
	header.data = (protocol->protocol_type << 24) | (len & MMBI_HDR_LEN_MASK) | (padding & MMBI_HDR_PADDING_MASK);
	wr_offset = h_ros_ptr->bmc_buf_wr_ptr << 2;

	if (wr_offset + sizeof(struct ast2700_espi_mmbi_hdr) <= channel->b2h_circular_buf_size) {
		memcpy_toio(channel->b2h_circular_buf + wr_offset, (void *)&header,
			    sizeof(struct ast2700_espi_mmbi_hdr));
		wr_offset += sizeof(struct ast2700_espi_mmbi_hdr);
		if (wr_offset == channel->b2h_circular_buf_size)
			wr_offset = 0;
	} else {
		chunk_len = channel->b2h_circular_buf_size - wr_offset;

		memcpy_toio(channel->b2h_circular_buf + wr_offset, (void *)&header,
			    chunk_len);
		memcpy_toio(channel->b2h_circular_buf, ((u8 *)&header) + chunk_len,
			    sizeof(struct ast2700_espi_mmbi_hdr) - chunk_len);
		wr_offset = sizeof(struct ast2700_espi_mmbi_hdr) - chunk_len;
	}

	if (wr_offset + len <= channel->b2h_circular_buf_size) {
		if (copy_from_user(channel->b2h_circular_buf + wr_offset, buffer, len)) {
			dev_err(priv->dev, "%s: copy from user failed\n", __func__);
			return -EFAULT;
		}
		wr_offset += len;
		if (wr_offset == channel->b2h_circular_buf_size)
			wr_offset = 0;
	} else {
		chunk_len = channel->b2h_circular_buf_size - wr_offset;

		if (copy_from_user(channel->b2h_circular_buf + wr_offset, buffer, chunk_len) ||
		    copy_from_user(channel->b2h_circular_buf,
				   buffer + chunk_len, len - chunk_len)) {
			dev_err(priv->dev, "%s: copy from user failed\n", __func__);
			return -EFAULT;
		}
		wr_offset = len - chunk_len;
	}

	*offp += len;
	ast2700_espi_mmbi_update_host_ros(channel, 0, len + sizeof(struct ast2700_espi_mmbi_hdr) + padding);
	dev_dbg(priv->dev, "%s: Write %ld bytes to MMBI\n", __func__, len);

	return len;
}

static const struct file_operations ast2700_espi_mmbi_fops = {
	.owner = THIS_MODULE,
	.open = ast2700_espi_mmbi_open,
	.release = ast2700_espi_mmbi_release,
	.read = ast2700_espi_mmbi_read,
	.write = ast2700_espi_mmbi_write,
	.unlocked_ioctl = ast2700_espi_mmbi_ioctl,
	.poll = ast2700_espi_mmbi_poll
};

static const struct regmap_config aspeed_espi_mmbi_regmap_cfg = {
	.name = "mmbi",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x04C,
};

static const struct regmap_config aspeed_espi_regmap_cfg = {
	.name = "espi",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x7fc,
};

static const struct of_device_id aspeed_espi_mmbi_of_match[] = {
	{ .compatible = "aspeed,ast2700-intel-espi-mmbi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aspeed_espi_mmbi_of_match);

static char *get_protocol_suffix(u8 type)
{
	switch (type) {
	case MMBI_PROTOCOL_IPMI:
		return "ipmi";
	case MMBI_PROTOCOL_SEAMLESS:
		return "seamless";
	case MMBI_PROTOCOL_RAS_OFFLOAD:
		return "ras_offload";
	case MMBI_PROTOCOL_MCTP:
		return "mctp";
	case MMBI_PROTOCOL_VENDOR:
		return "vendor";
	default:
		break;
	}

	return NULL;
}

static irqreturn_t ast2700_espi_mmbi_irq_handler(int irq, void *arg)
{
	struct ast2700_espi_mmbi *priv = arg;
	u32 status;
	int idx;

	regmap_read(priv->mmbi_map, ESPI_MMBI_INT_STATUS, &status);
	/* Clear interrupt */
	regmap_write(priv->mmbi_map, ESPI_MMBI_INT_STATUS, status);

	for (idx = 0; idx < AST2700_ESPI_MMBI_MAX_CHANNELS; idx++) {
		/*
		 * Host RWP 0: It gets updated after Host reads data and also
		 * when host want to send reset MMBI buffer request. So
		 * Handle reset request and ignore read pointer update.
		 * Host RWP 1: It gets updated when host write data on H2B,
		 * So process the request by invoking corresponding device.
		 */
		if (!priv->channels[idx].enabled)
			continue;

		if ((status >> (idx * 2)) & ESPI_MMBI_HOST_RWS0_INT) {
			// Host_RWS[0:3] update
			ast2700_espi_mmbi_host_rwp0_handler(&priv->channels[idx]);
		} else if (status >> (idx * 2) & ESPI_MMBI_HOST_RWS1_INT) {
			// Host_RWS[4:7] update
			ast2700_espi_mmbi_host_rwp1_handler(&priv->channels[idx]);
		}
	}
	dev_info(priv->dev, "MMBI IRQ Status: %d\n", status);

	return IRQ_HANDLED;
}

static void aspeed_espi_mmbi_release_rmem(void *d)
{
	of_reserved_mem_device_release(d);
}

static void aspeed_espi_mmbi_dma_finish(struct ast2700_espi_mmbi *priv)
{
	dma_free_coherent(priv->dev, MMBI_MEMORY_SIZE, priv->mmbi_virt_addr, priv->mmbi_phys_addr);
}

static int aspeed_espi_mmbi_dma_init(struct ast2700_espi_mmbi *priv)
{
	int rc;

	rc = dma_set_mask_and_coherent(priv->dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_err(priv->dev, "cannot set 64-bits DMA mask\n");
		return rc;
	}

	/* DMA pool initialization */
	rc = of_reserved_mem_device_init(priv->dev);
	if (rc) {
		dev_err(priv->dev, "device does not have specific DMA pool: %d\n",
			rc);
		return rc;
	}

	rc = devm_add_action_or_reset(priv->dev, aspeed_espi_mmbi_release_rmem,
				       priv->dev);
	if (rc) {
		dev_err(priv->dev, "Failed to add action to release reserved memory: %d\n",
			rc);
		return rc;
	}

	priv->mmbi_virt_addr = dma_alloc_coherent(priv->dev, MMBI_MEMORY_SIZE,
							&priv->mmbi_phys_addr, GFP_KERNEL);

	if (!priv->mmbi_virt_addr) {
		dev_err(priv->dev, "Failed to allocate DMA memory for MMBI\n");
		return -ENOMEM;
	}
	priv->mmbi_size = MMBI_MEMORY_SIZE;

	return 0;
}

static int aspeed_espi_mmbi_hw_init(struct ast2700_espi_mmbi *priv)
{
	u64 mask;
	u32 reg;

	/* disable interrupt and clear status */
	regmap_write(priv->espi_map, ESPI_CH0_INT_EN, 0x0);
	regmap_write(priv->espi_map, ESPI_CH0_INT_STS, 0xffffffff);

	regmap_write(priv->mmbi_map, ESPI_MMBI_INT_ENABLE, 0);
	regmap_write(priv->mmbi_map, ESPI_MMBI_INT_STATUS, 0xffffffff);

	/* disable address masking, in here we use memory cycle 0 */
	regmap_read(priv->espi_map, ESPI_CH0_MCYC0_MASKL, &reg);
	reg &= ~ESPI_CH0_MCYC0_MASKL_EN;
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_MASKL, reg);

	regmap_read(priv->espi_map, ESPI_CH0_MCYC0_MASKL, &reg);
	reg &= ~ESPI_CH0_MCYC0_MASKL_EN;
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_MASKL, reg);

	/* disable memory w/r and clear eSPI CH0 queues */
	regmap_read(priv->espi_map, ESPI_CH0_CTRL, &reg);
	reg |= (ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
	reg &= ~(ESPI_CH0_CTRL_NP_TX_RST
		 | ESPI_CH0_CTRL_NP_RX_RST
		 | ESPI_CH0_CTRL_PC_TX_RST
		 | ESPI_CH0_CTRL_PC_RX_RST
		 | ESPI_CH0_CTRL_NP_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_RX_DMA_EN
		 | ESPI_CH0_CTRL_SW_RDY);
	regmap_write(priv->espi_map, ESPI_CH0_CTRL, reg);

	udelay(1);

	reg |= (ESPI_CH0_CTRL_NP_TX_RST
		| ESPI_CH0_CTRL_NP_RX_RST
		| ESPI_CH0_CTRL_PC_TX_RST
		| ESPI_CH0_CTRL_PC_RX_RST);
	regmap_write(priv->espi_map, ESPI_CH0_CTRL, reg);

	/* disable espi-mmbi controller before configuring espi address */
	regmap_read(priv->mmbi_map, ESPI_MMBI_CTRL, &reg);
	reg &= ~ESPI_MMBI_CTRL_EN;
	regmap_write(priv->mmbi_map, ESPI_MMBI_CTRL, reg);

	mask = ~(priv->mmbi_size - 1);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_MASKH, mask >> 32);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_MASKL, mask & 0xffffffff);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_SADDRH, priv->host_map_addr >> 32);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_SADDRL, priv->host_map_addr & 0xffffffff);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_TADDRH, priv->mmbi_phys_addr >> 32);
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_TADDRL, priv->mmbi_phys_addr & 0xffffffff);

	/* enable espi-mmbi controller and configure instances */
	reg = FIELD_PREP(ESPI_MMBI_CTRL_INST_NUM, (priv->mmbi_instances - 1)) | ESPI_MMBI_CTRL_EN;
	regmap_write(priv->mmbi_map, ESPI_MMBI_CTRL, reg);

	/* enable eSPI memory cycle 0 */
	regmap_read(priv->espi_map, ESPI_CH0_MCYC0_MASKL, &reg);
	reg |= ESPI_CH0_MCYC0_MASKL_EN;
	regmap_write(priv->espi_map, ESPI_CH0_MCYC0_MASKL, reg);

	/* keep Peripheral Channel Mem Read/Write after WDT reset */
	regmap_read(priv->espi_map, ESPI_CH0_CTRL, &reg);
	reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS_WDT | ESPI_CH0_CTRL_MCYC_WR_DIS_WDT);
	regmap_write(priv->espi_map, ESPI_CH0_CTRL, reg);

	return 0;
}

static struct aspeed_mmbi_desc aspeed_espi_mmbi_desc_init(struct aspeed_espi_mmbi_channel channel)
{
	struct aspeed_mmbi_desc desc;

	memset(&desc, 0, sizeof(desc));
	memcpy_fromio(desc.mmbi_signature, MMBI_SIGNATURE_STRING, sizeof(desc.mmbi_signature));
	desc.mmbi_version = 1;
	desc.os_use = 1;
	desc.buf_type = MMBI_BUF_TYPE_VPSCB_V1;

	desc.buf_desc.host_ros_pointer = (channel.host_ros - channel.desc) >> 3;
	desc.buf_desc.host_rws_pointer = (channel.host_rws - channel.desc) >> 3;

	desc.buf_desc.host_int_type = 3;
	desc.buf_desc.host_int_location = 0x06;
	desc.buf_desc.host_int_val = 0x11;
	desc.buf_desc.bmc_int_type = 0;
	desc.buf_desc.bmc_int_location = 0x00;
	desc.buf_desc.bmc_int_val = 0x00;

	desc.b2h_buf_base_addr = (u32)(channel.b2h_circular_buf - channel.desc) >> 3;
	desc.h2b_buf_base_addr = (u32)(channel.h2b_circular_buf - channel.desc) >> 3;
	desc.b2h_buf_len = channel.b2h_circular_buf_size;
	desc.h2b_buf_len = channel.h2b_circular_buf_size;

	return desc;
}

static int aspeed_espi_mmbi_chann_init(struct ast2700_espi_mmbi *priv, struct device_node *node, int seq, u8 idx)
{
	struct aspeed_mmbi_desc desc;
	int rc, i;
	u8 protocol_num;
	u8 protocol_supported[MMBI_MAX_SUPPORT_PROTOCOLS];
	u32 b2h_offset, h2b_offset;
	u32 instance_size;

	memset(&priv->channels[idx], 0, sizeof(struct aspeed_espi_mmbi_channel));

	/* configure virtual memory addresses
	 * In AST2700, memory layout is divided into B2H and H2B,
	 * B2H allocate top half and H2B allocate bottom half.
	 * For multiple instances, each instance is allocated separately in B2H and H2B.
	 * MMBI Capability Descriptor is at the top of B2H
	 */
	instance_size = MMBI_MEMORY_SIZE / 2 / priv->mmbi_instances;
	b2h_offset = idx * instance_size;
	h2b_offset = MMBI_MEMORY_SIZE / 2 + idx * instance_size;

	priv->channels[idx].desc = (u8 *)priv->mmbi_virt_addr + b2h_offset;
	priv->channels[idx].host_ros = priv->channels[idx].desc + sizeof(struct aspeed_mmbi_desc);
	priv->channels[idx].b2h_circular_buf = priv->channels[idx].host_ros + sizeof(struct aspeed_mmbi_host_ros);
	priv->channels[idx].host_rws = (u8 *)priv->mmbi_virt_addr + h2b_offset;
	priv->channels[idx].h2b_circular_buf = priv->channels[idx].host_rws + sizeof(struct aspeed_mmbi_host_rws);

	/* make sure buffer starts 4byte-aligned */
	priv->channels[idx].b2h_circular_buf = (u8 *)ALIGN((uintptr_t)priv->channels[idx].b2h_circular_buf, 4);
	priv->channels[idx].h2b_circular_buf = (u8 *)ALIGN((uintptr_t)priv->channels[idx].h2b_circular_buf, 4);

	priv->channels[idx].b2h_circular_buf_size = instance_size - sizeof(struct aspeed_mmbi_desc) -
		sizeof(struct aspeed_mmbi_host_ros);
	priv->channels[idx].h2b_circular_buf_size = instance_size - sizeof(struct aspeed_mmbi_host_rws);

	memset(priv->channels[idx].host_ros, 0, sizeof(struct aspeed_mmbi_host_ros));
	memset(priv->channels[idx].host_rws, 0, sizeof(struct aspeed_mmbi_host_rws));
	priv->channels[idx].state = ESPI_MMBI_INIT_IN_PROGRESS;

	dev_dbg(priv->dev, "MMBI: Channel(%d) desc: 0x%llx, host_ros: 0x%llx, host_rws: 0x%llx\n",
			idx, (unsigned long long)priv->channels[idx].desc, (unsigned long long)priv->channels[idx].host_ros,
			(unsigned long long)priv->channels[idx].host_rws);
	dev_dbg(priv->dev, "MMBI: Channel(%d) b2h_circular_buf: 0x%llx, h2b_circular_buf: 0x%llx\n",
			idx, (unsigned long long)priv->channels[idx].b2h_circular_buf,
			(unsigned long long)priv->channels[idx].h2b_circular_buf);

	desc = aspeed_espi_mmbi_desc_init(priv->channels[idx]);
	memcpy_fromio(priv->channels[idx].desc, &desc, sizeof(desc));

	priv->channels[idx].enabled = true;
	priv->channels[idx].chann_idx = idx;

	if (!node) {
		dev_err(priv->dev, "Fail to read instance");
		return -ENODEV;
	}

	protocol_num = of_property_count_u8_elems(node, "protocols");
	if (protocol_num < 0 || protocol_num > MMBI_MAX_SUPPORT_PROTOCOLS) {
		dev_err(priv->dev, "Invalid number of protocols: %d\n", protocol_num);
		return -EINVAL;
	}

	rc = of_property_read_u8_array(node, "protocols", protocol_supported, protocol_num);
	if (rc) {
		dev_err(priv->dev, "Failed to read protocols for channel %d: %d\n", idx, rc);
		return rc;
	}

	memset(priv->channels[idx].protocols, 0, sizeof(priv->channels[idx].protocols));
	for (i = 0; i < protocol_num; i++) {
		u8 proto;
		char *proto_name;

		proto = protocol_supported[i];
		proto_name = get_protocol_suffix(proto);

		if (!proto_name) {
			dev_err(priv->dev, "Unsupported protocol type: %d\n", proto);
			continue;
		}

		priv->channels[idx].protocols[i].miscdev.name =
			devm_kasprintf(priv->dev, GFP_KERNEL, "%s%d-%s%d-%s", "espi-mmbi", seq, "chann", idx, proto_name);
		priv->channels[idx].protocols[i].miscdev.minor = MISC_DYNAMIC_MINOR;
		priv->channels[idx].protocols[i].miscdev.fops = &ast2700_espi_mmbi_fops; /* to be implemented */
		priv->channels[idx].protocols[i].miscdev.parent = priv->dev;
		rc = misc_register(&priv->channels[idx].protocols[i].miscdev);
		if (rc) {
			dev_err(priv->dev, "Failed to register miscdev for protocol %s: %d\n",
				proto_name, rc);
			continue;
		}
		priv->channels[idx].protocols[i].protocol_type = proto;
		priv->channels[idx].protocols[i].channel = &priv->channels[idx];
		priv->channels[idx].protocols[i].data_available = false;
		priv->channels[idx].protocols[i].process_data = false;

		init_waitqueue_head(&priv->channels[idx].protocols[i].queue);
	}
	priv->channels[idx].priv = priv;

	dev_info(priv->dev, "MMBI: Channel(%d) protocols: %*ph\n", idx,
			protocol_num, protocol_supported);

	return 0;
}

static int aspeed_espi_mmbi_probe(struct platform_device *pdev)
{
	const struct of_device_id *dev_id;
	struct ast2700_espi_mmbi *priv;
	struct aspeed_espi *espi;
	struct device_node *node;
	void __iomem *regs;
	int rc, seq, irq;
	u32 i, mmbi_irq_en;

	dev_info(&pdev->dev, "ASPEED eSPI MMBI driver probe\n");
	if (!pdev->dev.parent) {
		dev_err(&pdev->dev, "Device has no parent\n");
		return -ENODEV;
	}

	espi = dev_get_drvdata(pdev->dev.parent);
	if (!espi) {
		dev_err(&pdev->dev, "Failed to get eSPI controller data\n");
		return -EPROBE_DEFER;
	}

	dev_id = of_match_device(aspeed_espi_mmbi_of_match, &pdev->dev);
	if (!dev_id) {
		rc = PTR_ERR(dev_id);
		goto out;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(struct ast2700_espi_mmbi), GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto out;
	}

	priv->dev = &pdev->dev;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		rc = PTR_ERR(regs);
		goto out;
	}

	/* MMBI register map */
	priv->mmbi_map = devm_regmap_init_mmio(&pdev->dev, regs, &aspeed_espi_mmbi_regmap_cfg);
	if (IS_ERR(priv->mmbi_map)) {
		rc = PTR_ERR(priv->mmbi_map);
		goto out;
	}

	/* ESPI register map */
	priv->espi_map = devm_regmap_init_mmio(priv->dev, espi->regs,
				       &aspeed_espi_regmap_cfg);
	if (IS_ERR(priv->espi_map)) {
		rc = PTR_ERR(priv->espi_map);
		goto out;
	}

	/* LPC register map */
	priv->lpc_map = syscon_regmap_lookup_by_phandle(priv->dev->of_node,
							"aspeed,lpc");
	if (IS_ERR(priv->lpc_map)) {
		rc = PTR_ERR(priv->lpc_map);
		goto out;
	}

	rc = aspeed_espi_mmbi_dma_init(priv);
	if (rc)
		goto out_dma;

	rc = of_property_read_u64(priv->dev->of_node, "host-map-addr", &priv->host_map_addr);
	if (rc) {
		dev_err(priv->dev, "Failed to read host-map-addr property\n");
		goto out_dma;
	}

	rc = of_property_read_u32(priv->dev->of_node, "instances", &priv->mmbi_instances);
	if (rc) {
		dev_err(priv->dev, "Failed to read mmbi instances property, set to 1\n");
		priv->mmbi_instances = 1;
	}

	/* check priv->mmbi_instances is less than 8 and is power of 2 */
	if (priv->mmbi_instances > 8 ||
		!(priv->mmbi_instances && !(priv->mmbi_instances & (priv->mmbi_instances - 1)))) {
		dev_err(priv->dev, "Invalid number of MMBI instances: %d\n", priv->mmbi_instances);
		rc = -EINVAL;
		goto out_dma;
	}

	dev_info(priv->dev, "MMBI: HostAddr:0x%llx, Physical addr:0x%llx, instances: %d\n",
		priv->host_map_addr, priv->mmbi_phys_addr, priv->mmbi_instances);

	rc = aspeed_espi_mmbi_hw_init(priv);
	if (rc) {
		dev_err(priv->dev, "Failed to initialize MMBI hardware: %d\n", rc);
		goto out_dma;
	}

	seq = of_alias_get_id(priv->dev->of_node, "espi-mmbi");
	if (seq < 0) {
		dev_err(priv->dev, "Failed to get device sequence id\n");
		seq = 0;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		rc = irq;
		goto out_dma;
	}

	rc = devm_request_irq(priv->dev, irq, ast2700_espi_mmbi_irq_handler, IRQF_SHARED,
			     dev_name(priv->dev), priv);
	if (rc) {
		dev_err(priv->dev, "Failed to request IRQ: %d\n", rc);
		goto out_dma;
	}

	mmbi_irq_en = 0;
	for_each_child_of_node(priv->dev->of_node, node) {
		rc = of_property_read_u32(node, "channel", &i);
		if (rc || i >= AST2700_ESPI_MMBI_MAX_CHANNELS || priv->channels[i].enabled)
			continue;
		rc = aspeed_espi_mmbi_chann_init(priv, node, seq, i);
		if (rc)
			dev_err(priv->dev, "MMBI: Channel(%d) init failed\n",
				i);
		else
			mmbi_irq_en += (0x03 << (i * 2));
	}

	regmap_write(priv->mmbi_map, ESPI_MMBI_INT_ENABLE, mmbi_irq_en);

	for (i = 0; i < AST2700_ESPI_MMBI_MAX_CHANNELS; i++) {
		if (priv->channels[i].enabled)
			send_bmc_up_request(&priv->channels[i]);
	}

	dev_set_drvdata(priv->dev, priv);
	return 0;
out_dma:
	if (priv->mmbi_virt_addr)
		aspeed_espi_mmbi_dma_finish(priv);
out:
	dev_err(&pdev->dev, "Failed to probe ASPEED eSPI MMBI: %d\n", rc);
	return rc;
}

static void aspeed_espi_mmbi_remove(struct platform_device *pdev)
{
	struct ast2700_espi_mmbi *priv = dev_get_drvdata(&pdev->dev);
	u32 i, j, reg;

	if (!priv)
		return;

	regmap_write(priv->mmbi_map, ESPI_MMBI_INT_ENABLE, 0);

	regmap_read(priv->mmbi_map, ESPI_MMBI_CTRL, &reg);
	reg &= ~ESPI_MMBI_CTRL_EN;
	regmap_write(priv->mmbi_map, ESPI_MMBI_CTRL, reg);

	for (i = 0; i < AST2700_ESPI_MMBI_MAX_CHANNELS; i++) {
		if (!priv->channels[i].enabled)
			continue;

		for (j = 0; j < MMBI_MAX_SUPPORT_PROTOCOLS; j++) {
			if (!priv->channels[i].protocols[j].miscdev.this_device)
				continue;

			misc_deregister(&priv->channels[i].protocols[j].miscdev);
		}
	}

	if (priv->mmbi_virt_addr)
		aspeed_espi_mmbi_dma_finish(priv);

	dev_info(&pdev->dev, "ASPEED eSPI MMBI driver removed\n");
}

static struct platform_driver aspeed_espi_mmbi_driver = {
	.driver = {
		.name = "ast2700-intel-espi-mmbi",
		.of_match_table = aspeed_espi_mmbi_of_match,
	},
	.probe = aspeed_espi_mmbi_probe,
	.remove = aspeed_espi_mmbi_remove,
};

module_platform_driver(aspeed_espi_mmbi_driver);

MODULE_AUTHOR("Your Name <your.email@example.com>");
MODULE_DESCRIPTION("ASPEED eSPI MMBI driver");
MODULE_LICENSE("GPL");
