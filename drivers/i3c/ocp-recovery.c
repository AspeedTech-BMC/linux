// SPDX-License-Identifier: GPL-2.0-only
/*
 * OCP Secure Firmware Recovery - I3C binding
 *
 * Binds to I3C targets that advertise DCR I3C_DCR_OCP_RECOVERY (0xBD) and
 * exposes a thin, raw command/response sysfs interface under
 * <i3c-device>/ocp_recovery/ so that userspace recovery tooling (libocp /
 * ocp-recovery-tool) can reuse the exact same [cmd, len, payload...] /
 * [count, payload...] framing it already builds for the I2C transport -
 * no protocol-layer code changes needed on the userspace side.
 *
 *   cmd       - write-only.  One SDR private write per write(2), used for
 *               commands that expect no reply (RECOVERY_CTRL,
 *               INDIRECT_CTRL, INDIRECT_DATA writes, DEVICE_RESET).
 *   cmd_resp  - write(2) takes the single command byte the recovery
 *               protocol always sends for a read (PROT_CAP, DEVICE_ID,
 *               DEVICE_STATUS, RECOVERY_STATUS, INDIRECT_STATUS,
 *               INDIRECT_DATA reads) and performs the write AND the
 *               repeated-start read as one atomic i3c_priv_xfer pair -
 *               the same combined transaction ioctl(I2C_RDWR) /
 *               ioctl(I3C_IOC_PRIV_XFER) give today.  The decoded
 *               response is cached; a following read(2) just drains it
 *               with no further bus activity.
 *
 * Copyright (C) 2026 ASPEED Technology Inc.
 */

#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/i3c/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

/* Matches ocp::recovery::chunkSizeMax in libocp/include/ocp/ocp_recovery.hpp */
#define OCP_RECOVERY_MAX_CMD_PAYLOAD	252
/* cmd byte + len byte + payload: the shape Target::command() in libocp
 * builds, unrelated to how a write is actually framed on the I3C wire
 * (see OCP_RECOVERY_LEN_PREFIX below) - this is what cmd's write(2)
 * receives from userspace before this driver re-frames it.
 */
#define OCP_RECOVERY_MAX_CMD		(2 + OCP_RECOVERY_MAX_CMD_PAYLOAD)
/* One full SMBus block payload */
#define OCP_RECOVERY_MAX_RESP_PAYLOAD	255
/* count byte + payload: the shape Target::commandRead() in libocp expects,
 * unrelated to how the response is actually framed on the I3C wire (see
 * OCP_RECOVERY_LEN_PREFIX below) - this is what cmd_resp's read(2) hands
 * back to userspace after this driver has already translated it.
 */
#define OCP_RECOVERY_MAX_RESP		(1 + OCP_RECOVERY_MAX_RESP_PAYLOAD)

/* Confirmed against a real target by waveform capture: unlike the SMBus
 * framing libocp was written against (a single byte count/length, no
 * separate length field on writes beyond that one byte), this device's
 * I3C wire framing uses a 2-byte little-endian length field in both
 * directions, then the PEC byte:
 *   write: [cmd, len_lsb, len_msb, payload...]
 *   read:  [len_lsb, len_msb, payload...]
 * cmd_write() and cmd_resp_write() translate libocp's 1-byte-length
 * shape to/from this on the way to/from the bus, so libocp itself needs
 * no I3C-specific code.
 */
#define OCP_RECOVERY_LEN_PREFIX		2

/* cmd byte + 2-byte length prefix + payload + PEC byte */
#define OCP_RECOVERY_MAX_CMD_WIRE	\
	(1 + OCP_RECOVERY_LEN_PREFIX + OCP_RECOVERY_MAX_CMD_PAYLOAD + 1)
/* 2-byte length prefix + payload + PEC byte */
#define OCP_RECOVERY_MAX_RESP_WIRE	\
	(OCP_RECOVERY_LEN_PREFIX + OCP_RECOVERY_MAX_RESP_PAYLOAD + 1)

#define OCP_RECOVERY_CRC8_POLYNOMIAL	0x07
DECLARE_CRC8_TABLE(ocp_recovery_crc8_table);

struct ocp_recovery {
	struct i3c_device *i3c;
	/* Serialises the whole cmd_resp write-then-read bus transaction
	 * and protects resp_buf/resp_len against a concurrent opener.
	 */
	struct mutex lock;
	u8 resp_buf[OCP_RECOVERY_MAX_RESP];
	size_t resp_len;
};

static struct ocp_recovery *kobj_to_ocp_recovery(struct kobject *kobj)
{
	return dev_get_drvdata(kobj_to_dev(kobj));
}

/* addr_rnw = (dynamic address << 1) | R/W, mirroring classic SMBus PEC,
 * which seeds the CRC with the address+R/W byte before the message
 * bytes.  Appends the computed PEC at buf[len].
 */
static void ocp_recovery_pec_append(u8 addr_rnw, u8 *buf, size_t len)
{
	u8 pec = crc8(ocp_recovery_crc8_table, &addr_rnw, 1, 0);

	pec = crc8(ocp_recovery_crc8_table, buf, len, pec);
	buf[len] = pec;
}

/* Verifies the trailing PEC byte in a len-byte buffer (payload + PEC).
 * Returns 0 when it matches, -EBADMSG otherwise.
 */
static int ocp_recovery_pec_verify(u8 addr_rnw, const u8 *buf, size_t len)
{
	u8 pec;

	if (len < 2)
		return -EBADMSG;

	pec = crc8(ocp_recovery_crc8_table, &addr_rnw, 1, 0);
	pec = crc8(ocp_recovery_crc8_table, buf, len - 1, pec);

	return pec == buf[len - 1] ? 0 : -EBADMSG;
}

static ssize_t cmd_write(struct file *filp, struct kobject *kobj,
			 const struct bin_attribute *bin_attr, char *buf,
			 loff_t off, size_t count)
{
	struct ocp_recovery *ocp = kobj_to_ocp_recovery(kobj);
	struct i3c_device_info info;
	struct i3c_priv_xfer xfer = { .rnw = false };
	u8 wire[OCP_RECOVERY_MAX_CMD_WIRE];
	size_t payload_len;
	size_t wire_len;
	int ret;

	if (off)
		return -EINVAL;
	/* userspace shape (Target::command() in libocp): [cmd, len, payload] */
	if (count < 2 || count > OCP_RECOVERY_MAX_CMD)
		return -EINVAL;

	payload_len = count - 2;
	if ((u8)buf[1] != payload_len) {
		dev_dbg(i3cdev_to_dev(ocp->i3c),
			"cmd 0x%02x: declared len %u != %zu bytes of payload written\n",
			(u8)buf[0], (u8)buf[1], payload_len);
		return -EINVAL;
	}

	/* Wire shape (confirmed by waveform):
	 * [cmd, len_lsb, len_msb, payload...], then PEC.
	 */
	wire[0] = buf[0];
	wire[1] = (u8)(payload_len & 0xff);
	wire[2] = (u8)((payload_len >> 8) & 0xff);
	memcpy(&wire[1 + OCP_RECOVERY_LEN_PREFIX], &buf[2], payload_len);
	wire_len = 1 + OCP_RECOVERY_LEN_PREFIX + payload_len;

	i3c_device_get_info(ocp->i3c, &info);
	ocp_recovery_pec_append(info.dyn_addr << 1 | 0, wire, wire_len);

	xfer.len = wire_len + 1;
	xfer.data.out = wire;

	mutex_lock(&ocp->lock);
	ret = i3c_device_do_priv_xfers(ocp->i3c, &xfer, 1);
	mutex_unlock(&ocp->lock);

	return ret ? ret : count;
}

static ssize_t cmd_resp_write(struct file *filp, struct kobject *kobj,
			      const struct bin_attribute *bin_attr, char *buf,
			      loff_t off, size_t count)
{
	struct ocp_recovery *ocp = kobj_to_ocp_recovery(kobj);
	struct i3c_device_info info;
	struct i3c_priv_xfer xfers[2] = { };
	u8 cmd_wire[2];
	u8 resp_wire[OCP_RECOVERY_MAX_RESP_WIRE];
	size_t actual;
	int ret;

	if (off)
		return -EINVAL;
	/* The recovery protocol's read commands always send a single,
	 * bare command byte (Target::commandRead() in libocp) - reject
	 * anything else rather than silently truncating.
	 */
	if (count != 1)
		return -EINVAL;

	i3c_device_get_info(ocp->i3c, &info);

	cmd_wire[0] = buf[0];
	ocp_recovery_pec_append(info.dyn_addr << 1 | 0, cmd_wire, 1);

	xfers[0].rnw = false;
	xfers[0].len = sizeof(cmd_wire);
	xfers[0].data.out = cmd_wire;

	xfers[1].rnw = true;
	xfers[1].len = sizeof(resp_wire);
	xfers[1].data.in = resp_wire;

	mutex_lock(&ocp->lock);

	ret = i3c_device_do_priv_xfers(ocp->i3c, xfers, 2);
	if (ret) {
		ocp->resp_len = 0;
		goto out_unlock;
	}

	actual = xfers[1].actual_len ?: xfers[1].len;
	if (actual > sizeof(resp_wire))
		actual = sizeof(resp_wire);

	ret = ocp_recovery_pec_verify(info.dyn_addr << 1 | 1, resp_wire,
				      actual);
	if (ret) {
		const u8 dyn_seed_byte = (u8)(info.dyn_addr << 1 | 1);
		const u8 static_seed_byte = (u8)(info.static_addr << 1 | 1);
		const u8 seed_dyn = crc8(ocp_recovery_crc8_table,
					 &dyn_seed_byte, 1, 0);
		const u8 seed_static = crc8(ocp_recovery_crc8_table,
					    &static_seed_byte, 1, 0);
		const u8 pec_dyn = crc8(ocp_recovery_crc8_table, resp_wire,
					actual - 1, seed_dyn);
		const u8 pec_static = crc8(ocp_recovery_crc8_table, resp_wire,
					   actual - 1, seed_static);
		const u8 pec_noaddr = crc8(ocp_recovery_crc8_table, resp_wire,
					   actual - 1, 0);

		ocp->resp_len = 0;
		print_hex_dump_debug("ocp-recovery resp: ", DUMP_PREFIX_NONE,
				     16, 1, resp_wire, actual, false);
		dev_dbg(i3cdev_to_dev(ocp->i3c),
			"PEC mismatch on cmd 0x%02x: got 0x%02x, candidates dyn=0x%02x static=0x%02x noaddr=0x%02x (dyn_addr=0x%02x static_addr=0x%02x)\n",
			buf[0], resp_wire[actual - 1], pec_dyn, pec_static,
			pec_noaddr, info.dyn_addr, info.static_addr);
		goto out_unlock;
	}

	/* Wire shape (PEC already verified and excluded above) is
	 * [len_lsb, len_msb, payload...]; re-pack it into the 1-byte-count
	 * [count, payload...] shape Target::commandRead() in libocp expects,
	 * so libocp needs no I3C-specific parsing.
	 */
	if (actual - 1 < OCP_RECOVERY_LEN_PREFIX) {
		dev_dbg(i3cdev_to_dev(ocp->i3c),
			"response to cmd 0x%02x too short for the length prefix (%zu bytes)\n",
			buf[0], actual - 1);
		ocp->resp_len = 0;
		ret = -EBADMSG;
		goto out_unlock;
	}

	{
		const size_t wire_payload_len = actual - 1 - OCP_RECOVERY_LEN_PREFIX;
		const u16 hdr_payload_len = resp_wire[0] | ((u16)resp_wire[1] << 8);
		size_t payload_len = min_t(size_t, hdr_payload_len,
					   wire_payload_len);

		if (hdr_payload_len != wire_payload_len)
			dev_dbg(i3cdev_to_dev(ocp->i3c),
				"cmd 0x%02x: length prefix %u != %zu bytes actually received, using %zu\n",
				buf[0], hdr_payload_len, wire_payload_len,
				payload_len);

		if (payload_len > OCP_RECOVERY_MAX_RESP_PAYLOAD)
			payload_len = OCP_RECOVERY_MAX_RESP_PAYLOAD;

		ocp->resp_buf[0] = (u8)payload_len;
		memcpy(&ocp->resp_buf[1], &resp_wire[OCP_RECOVERY_LEN_PREFIX],
		       payload_len);
		ocp->resp_len = 1 + payload_len;
	}

out_unlock:
	mutex_unlock(&ocp->lock);

	return ret ? ret : count;
}

static ssize_t cmd_resp_read(struct file *filp, struct kobject *kobj,
			     const struct bin_attribute *bin_attr, char *buf,
			     loff_t off, size_t count)
{
	struct ocp_recovery *ocp = kobj_to_ocp_recovery(kobj);
	size_t n;

	if (off)
		return -EINVAL;

	mutex_lock(&ocp->lock);
	n = min(count, ocp->resp_len);
	memcpy(buf, ocp->resp_buf, n);
	mutex_unlock(&ocp->lock);

	return n;
}

static BIN_ATTR_WO(cmd, OCP_RECOVERY_MAX_CMD);
static BIN_ATTR_RW(cmd_resp, OCP_RECOVERY_MAX_RESP);

static const struct bin_attribute *ocp_recovery_bin_attrs[] = {
	&bin_attr_cmd,
	&bin_attr_cmd_resp,
	NULL,
};

static const struct attribute_group ocp_recovery_group = {
	.name = "ocp_recovery",
	.bin_attrs = ocp_recovery_bin_attrs,
};

static int ocp_recovery_probe(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);
	struct ocp_recovery *ocp;
	int ret;

	ocp = devm_kzalloc(dev, sizeof(*ocp), GFP_KERNEL);
	if (!ocp)
		return -ENOMEM;

	ocp->i3c = i3cdev;
	mutex_init(&ocp->lock);

	i3cdev_set_drvdata(i3cdev, ocp);

	ret = sysfs_create_group(&dev->kobj, &ocp_recovery_group);
	if (ret)
		return ret;

	/* Informational only today - see the file header comment.  Not
	 * fatal: plenty of recovery ROM firmware predates PEC entirely.
	 */
	if (i3c_device_control_pec(i3cdev, true))
		dev_dbg(dev, "hardware PEC not available, using software PEC\n");

	/* Best-effort MRL/MWL negotiation so both endpoints can carry a
	 * full chunk + framing + PEC.  Recovery ROM firmware is often
	 * minimal and may not implement GETMRL/GETMWL/SETMRL/SETMWL at
	 * all, so a failure here is logged and left to surface as a
	 * normal transfer error later rather than blocking bind.
	 */
	{
		struct i3c_device_info info;

		i3c_device_get_info(i3cdev, &info);

		if (i3c_device_getmrl_ccc(i3cdev, &info) ||
		    info.max_read_len < OCP_RECOVERY_MAX_RESP_WIRE)
			if (i3c_device_setmrl_ccc(i3cdev, &info,
						  OCP_RECOVERY_MAX_RESP_WIRE, 0))
				dev_dbg(dev, "SETMRL not supported/accepted\n");

		if (i3c_device_getmwl_ccc(i3cdev, &info) ||
		    info.max_write_len < OCP_RECOVERY_MAX_CMD_WIRE)
			if (i3c_device_setmwl_ccc(i3cdev, &info,
						  OCP_RECOVERY_MAX_CMD_WIRE))
				dev_dbg(dev, "SETMWL not supported/accepted\n");
	}

	dev_info(dev, "ocp-recovery bound; sysfs at %s/ocp_recovery/\n",
		 dev_name(dev));

	return 0;
}

static void ocp_recovery_remove(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);

	sysfs_remove_group(&dev->kobj, &ocp_recovery_group);
}

static const struct i3c_device_id ocp_recovery_ids[] = {
	I3C_CLASS(I3C_DCR_OCP_RECOVERY, 0),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, ocp_recovery_ids);

static struct i3c_driver ocp_recovery_driver = {
	.driver = {
		.name = "ocp-recovery",
	},
	.probe = ocp_recovery_probe,
	.remove = ocp_recovery_remove,
	.id_table = ocp_recovery_ids,
};

static int __init ocp_recovery_init(struct i3c_driver *drv)
{
	crc8_populate_msb(ocp_recovery_crc8_table, OCP_RECOVERY_CRC8_POLYNOMIAL);
	i3c_driver_register(drv);

	return 0;
}

static void __exit ocp_recovery_exit(struct i3c_driver *drv)
{
	i3c_driver_unregister(drv);
}

module_driver(ocp_recovery_driver, ocp_recovery_init, ocp_recovery_exit);

MODULE_AUTHOR("Billy Tsai <billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("OCP Secure Firmware Recovery I3C binding");
MODULE_LICENSE("GPL");
