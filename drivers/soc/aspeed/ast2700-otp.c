// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Aspeed Technology Inc.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/module.h>
#include <asm/io.h>
#include <uapi/linux/otp_ast2700.h>

static DEFINE_SPINLOCK(otp_state_lock);

/***********************
 *                     *
 * OTP regs definition *
 *                     *
 ***********************/
#define OTP_REG_SIZE			0x200

#define OTP_PASSWD			0x349fe38a
#define OTP_CMD_READ			0x23b1e361
#define OTP_CMD_PROG			0x23b1e364
#define OTP_CMD_PROG_MULTI		0x23b1e365
#define OTP_CMD_CMP			0x23b1e363
#define OTP_CMD_BIST			0x23b1e368

#define OTP_CMD_OFFSET			0x20
#define OTP_MASTER			OTP_M1

#define OTP_KEY				0x0
#define OTP_CMD				(OTP_MASTER * OTP_CMD_OFFSET + 0x4)
#define OTP_WDATA_0			(OTP_MASTER * OTP_CMD_OFFSET + 0x8)
#define OTP_WDATA_1			(OTP_MASTER * OTP_CMD_OFFSET + 0xc)
#define OTP_WDATA_2			(OTP_MASTER * OTP_CMD_OFFSET + 0x10)
#define OTP_WDATA_3			(OTP_MASTER * OTP_CMD_OFFSET + 0x14)
#define OTP_STATUS			(OTP_MASTER * OTP_CMD_OFFSET + 0x18)
#define OTP_ADDR			(OTP_MASTER * OTP_CMD_OFFSET + 0x1c)
#define OTP_RDATA			(OTP_MASTER * OTP_CMD_OFFSET + 0x20)

#define OTP_DBG00			0x0C4
#define OTP_DBG01			0x0C8
#define OTP_MASTER_PID			0x0D0
#define OTP_ECC_EN			0x0D4
#define OTP_CMD_LOCK			0x0D8
#define OTP_SW_RST			0x0DC
#define OTP_SLV_ID			0x0E0
#define OTP_PMC_CQ			0x0E4
#define OTP_FPGA			0x0EC
#define OTP_CLR_FPGA			0x0F0
#define OTP_REGION_ROM_PATCH		0x100
#define OTP_REGION_OTPCFG		0x104
#define OTP_REGION_OTPSTRAP		0x108
#define OTP_REGION_OTPSTRAP_EXT		0x10C
#define OTP_REGION_SECURE0		0x120
#define OTP_REGION_SECURE0_RANGE	0x124
#define OTP_REGION_SECURE1		0x128
#define OTP_REGION_SECURE1_RANGE	0x12C
#define OTP_REGION_SECURE2		0x130
#define OTP_REGION_SECURE2_RANGE	0x134
#define OTP_REGION_SECURE3		0x138
#define OTP_REGION_SECURE3_RANGE	0x13C
#define OTP_REGION_USR0			0x140
#define OTP_REGION_USR0_RANGE		0x144
#define OTP_REGION_USR1			0x148
#define OTP_REGION_USR1_RANGE		0x14C
#define OTP_REGION_USR2			0x150
#define OTP_REGION_USR2_RANGE		0x154
#define OTP_REGION_USR3			0x158
#define OTP_REGION_USR3_RANGE		0x15C
#define OTP_REGION_CALIPTRA_0		0x160
#define OTP_REGION_CALIPTRA_0_RANGE	0x164
#define OTP_REGION_CALIPTRA_1		0x168
#define OTP_REGION_CALIPTRA_1_RANGE	0x16C
#define OTP_REGION_CALIPTRA_2		0x170
#define OTP_REGION_CALIPTRA_2_RANGE	0x174
#define OTP_REGION_CALIPTRA_3		0x178
#define OTP_REGION_CALIPTRA_3_RANGE	0x17C
#define OTP_RBP_SOC_SVN			0x180
#define OTP_RBP_SOC_KEYRETIRE		0x184
#define OTP_RBP_CALIP_SVN		0x188
#define OTP_RBP_CALIP_KEYRETIRE		0x18C
#define OTP_PUF				0x1A0
#define OTP_MASTER_ID			0x1B0
#define OTP_MASTER_ID_EXT		0x1B4
#define OTP_R_MASTER_ID			0x1B8
#define OTP_R_MASTER_ID_EXT		0x1BC
#define OTP_SOC_ECCKEY			0x1C0
#define OTP_SEC_BOOT_EN			0x1C4
#define OTP_SOC_KEY			0x1C8
#define OTP_CALPITRA_MANU_KEY		0x1CC
#define OTP_CALPITRA_OWNER_KEY		0x1D0
#define OTP_FW_ID_LSB			0x1D4
#define OTP_FW_ID_MSB			0x1D8
#define OTP_CALIP_FMC_SVN		0x1DC
#define OTP_CALIP_RUNTIME_SVN0		0x1E0
#define OTP_CALIP_RUNTIME_SVN1		0x1E4
#define OTP_CALIP_RUNTIME_SVN2		0x1E8
#define OTP_CALIP_RUNTIME_SVN3		0x1EC
#define OTP_SVN_WLOCK			0x1F0
#define OTP_INTR_EN			0x200
#define OTP_INTR_STS			0x204
#define OTP_INTR_MID			0x208
#define OTP_INTR_FUNC_INFO		0x20C
#define OTP_INTR_M_INFO			0x210
#define OTP_INTR_R_INFO			0x214

#define OTP_PMC				0x400
#define OTP_DAP				0x500

#define OTP_DAP_CFG_RQ			0x538

/* OTP status: [0] */
#define OTP_STS_IDLE			0x0
#define OTP_STS_BUSY			0x1

/* OTP cmd status: [7:4] */
#define OTP_GET_CMD_STS(x)		(((x) & 0xF0) >> 4)
#define OTP_STS_PASS			0x0
#define OTP_STS_FAIL			0x1
#define OTP_STS_CMP_FAIL		0x2
#define OTP_STS_REGION_FAIL		0x3
#define OTP_STS_MASTER_FAIL		0x4

/*
 * OTP_DBG01 ECC status: [5] single-bit error (corrected), [4:0] ECC syndrome
 *   S[5]=0, S[4:0]=0    -> no error
 *   S[5]=0, S[4:0]!=0   -> dual-bit error in Data or ECC[4:0] (uncorrectable)
 *   S[5]=1, S[4:0]!=0   -> single-bit error in Data or ECC[4:0] (corrected)
 */
#define OTP_ECC_STS_SINGLE_ERR		BIT(5)
#define OTP_ECC_STS_SYNDROME(x)		((x) & GENMASK(4, 0))

/* OTP ECC EN */
#define ECC_ENABLE			0x1
#define ECC_DISABLE			0x0
#define ECCBRP_EN			BIT(0)

/* AST2700 region layout (word addresses) */
#define AST2700_ROM_REGION_START_ADDR		0x0
#define AST2700_ROM_REGION_END_ADDR		0x3e0
#define AST2700_RBP_REGION_START_ADDR		AST2700_ROM_REGION_END_ADDR
#define AST2700_RBP_REGION_END_ADDR		0x400
#define AST2700_CONF_REGION_START_ADDR		AST2700_RBP_REGION_END_ADDR
#define AST2700_CONF_REGION_END_ADDR		0x420
#define AST2700_STRAP_REGION_START_ADDR		AST2700_CONF_REGION_END_ADDR
#define AST2700_STRAP_REGION_END_ADDR		0x430
#define AST2700_STRAPEXT_REGION_START_ADDR	AST2700_STRAP_REGION_END_ADDR
#define AST2700_STRAPEXT_REGION_END_ADDR	0x440
#define AST2700_USER_REGION_START_ADDR		AST2700_STRAPEXT_REGION_END_ADDR
#define AST2700_USER_REGION_END_ADDR		0x1000
#define AST2700_SEC_REGION_START_ADDR		AST2700_USER_REGION_END_ADDR
#define AST2700_SEC_REGION_END_ADDR		0x1c00
#define AST2700_CAL_REGION_START_ADDR		AST2700_SEC_REGION_END_ADDR
#define AST2700_CAL_REGION_END_ADDR		0x1f80
#define AST2700_SW_PUF_REGION_START_ADDR	AST2700_CAL_REGION_END_ADDR
#define AST2700_SW_PUF_REGION_END_ADDR		0x1fc0
#define AST2700_HW_PUF_REGION_START_ADDR	AST2700_SW_PUF_REGION_END_ADDR
#define AST2700_HW_PUF_REGION_END_ADDR		0x2000
#define AST2700_OTP_MEM_SIZE			AST2700_HW_PUF_REGION_END_ADDR

/* AST2705 region layout (word addresses) - TBD */
#define AST2705_ROM_REGION_START_ADDR		0x0
#define AST2705_ROM_REGION_END_ADDR		0x0
#define AST2705_RBP_REGION_START_ADDR		0x0
#define AST2705_RBP_REGION_END_ADDR		0x0
#define AST2705_CONF_REGION_START_ADDR		0x0
#define AST2705_CONF_REGION_END_ADDR		0x0
#define AST2705_STRAP_REGION_START_ADDR		0x0
#define AST2705_STRAP_REGION_END_ADDR		0x0
#define AST2705_STRAPEXT_REGION_START_ADDR	0x0
#define AST2705_STRAPEXT_REGION_END_ADDR	0x0
#define AST2705_USER_REGION_START_ADDR		0x0
#define AST2705_USER_REGION_END_ADDR		0x0
#define AST2705_SEC_REGION_START_ADDR		0x0
#define AST2705_SEC_REGION_END_ADDR		0x0
#define AST2705_CAL_REGION_START_ADDR		0x0
#define AST2705_CAL_REGION_END_ADDR		0x0
#define AST2705_SW_PUF_REGION_START_ADDR	0x0
#define AST2705_SW_PUF_REGION_END_ADDR		0x0
#define AST2705_HW_PUF_REGION_START_ADDR	0x0
#define AST2705_HW_PUF_REGION_END_ADDR		0x0
#define AST2705_OTP_MEM_SIZE			0x1000

#define OTP_TIMEOUT_US			10000

/* OTPCAL: Vendor key hash at w_offset 0x12, 48 bytes (24 words) */
#define OTPCAL_VKEY_HASH_W_OFFSET	0x12
#define OTPCAL_VKEY_HASH_WORDS		24

/* OTPSTRAP (AST2700) */
#define OTPSTRAP0_ADDR			AST2700_STRAP_REGION_START_ADDR
#define OTPSTRAP14_ADDR			(OTPSTRAP0_ADDR + 0xe)

#define OTPTOOL_VERSION(a, b, c)	(((a) << 24) + ((b) << 12) + (c))
#define OTPTOOL_VERSION_MAJOR(x)	(((x) >> 24) & 0xff)
#define OTPTOOL_VERSION_PATCHLEVEL(x)	(((x) >> 12) & 0xfff)
#define OTPTOOL_VERSION_SUBLEVEL(x)	((x) & 0xfff)
#define OTPTOOL_COMPT_VERSION		2

enum otp_error_code {
	OTP_SUCCESS,

	/*
	 * Dedicated error codes for OTP_STATUS[7:4] command results, kept
	 * out of the POSIX errno range so callers can tell them apart from
	 * generic I/O errors.
	 */
	OTP_CMD_ERR_BASE = 200,
	OTP_CMD_ERR_FAIL,		/* prog fail or soak limit exceeded */
	OTP_CMD_ERR_CMP_FAIL,		/* compare mismatch */
	OTP_CMD_ERR_REGION_FAIL,	/* region write/read protected */
	OTP_CMD_ERR_MASTER_FAIL,	/* master protection error */
	OTP_ECC_ERR_DUAL,		/* uncorrectable dual-bit ECC error */
};

enum aspeed_otp_master_id {
	OTP_M0 = 0,
	OTP_M1,
	OTP_M2,
	OTP_M3,
	OTP_M4,
	OTP_M5,
	OTP_MID_MAX,
};

struct otp_region_ecc {
	u32	start;
	u32	end;
	bool	ecc_supported;
	bool	ecc_en;
};

/**
 * struct aspeed_otp_plat_data - per-SoC OTP hardware configuration
 * @gran_bits:           OTP word width in bits (16 or 32)
 * @region_ecc:          per-region ECC policy defaults
 * @mem_words:           total OTP address space in words
 * @has_vendor_key_hash: vendor_key_hash sysfs attribute is supported
 * @ecc_strap_addr:      OTP word address of the ECC-enable strap bit;
 *                       0 means unknown (skip ecc_init, default disabled)
 */
struct aspeed_otp_plat_data {
	u8				gran_bits;
	const struct otp_region_ecc	*region_ecc;
	u32				mem_words;
	bool				has_vendor_key_hash;
	u32				ecc_strap_addr;
};

struct aspeed_otp {
	struct miscdevice			miscdev;
	struct device				*dev;
	void __iomem				*base;
	const struct aspeed_otp_plat_data	*plat;
	u32					chip_revid0;
	u32					chip_revid1;
	bool					is_open;
	int					gbl_ecc_en;
	u8					*data;

	/* per-instance live copy of region_ecc from platform data */
	struct otp_region_ecc			region_ecc[OTP_REGION_MAX];
};

enum otp_ioctl_cmds {
	GET_ECC_STATUS = 1,
	SET_ECC_ENABLE,
};

enum otp_ecc_codes {
	OTP_ECC_MISMATCH = -1,
	OTP_ECC_DISABLE = 0,
	OTP_ECC_ENABLE = 1,
};

/*
 * Per-region ECC policy defaults, applied when ctx->gbl_ecc_en (force ECC)
 * is off. Copied into each aspeed_otp instance's region_ecc[] at probe time
 * so multiple probed instances don't share (and fight over) one live table.
 * OTPRBP/OTPSTRAP don't support ECC in hardware, so ecc_supported is false
 * and ecc_en can never be set for them, even by force. OTPSTRAPEXT does
 * support ECC, unlike OTPSTRAP.
 *
 * start/end are OTP word addresses (not byte offsets).
 */
static const struct otp_region_ecc ast2700_region_ecc[OTP_REGION_MAX] = {
	[OTP_REGION_ROM]      = { AST2700_ROM_REGION_START_ADDR,      AST2700_ROM_REGION_END_ADDR,       true,  true  },
	[OTP_REGION_RBP]      = { AST2700_RBP_REGION_START_ADDR,      AST2700_RBP_REGION_END_ADDR,       false, false },
	[OTP_REGION_CFG]      = { AST2700_CONF_REGION_START_ADDR,     AST2700_CONF_REGION_END_ADDR,      true,  false },
	[OTP_REGION_STRAP]    = { AST2700_STRAP_REGION_START_ADDR,    AST2700_STRAP_REGION_END_ADDR,     false, false },
	[OTP_REGION_STRAPEXT] = { AST2700_STRAPEXT_REGION_START_ADDR, AST2700_STRAPEXT_REGION_END_ADDR,  true,  false },
	[OTP_REGION_USR]      = { AST2700_USER_REGION_START_ADDR,     AST2700_USER_REGION_END_ADDR,      true,  false },
	[OTP_REGION_SEC]      = { AST2700_SEC_REGION_START_ADDR,      AST2700_SEC_REGION_END_ADDR,       true,  false },
	[OTP_REGION_CAL]      = { AST2700_CAL_REGION_START_ADDR,      AST2700_CAL_REGION_END_ADDR,       true,  false },
	[OTP_REGION_PUF]      = { AST2700_SW_PUF_REGION_START_ADDR,   AST2700_HW_PUF_REGION_END_ADDR,    true,  true  },
};

static const struct aspeed_otp_plat_data ast2700_otp_plat_data = {
	.gran_bits           = 16,
	.region_ecc          = ast2700_region_ecc,
	.mem_words           = AST2700_OTP_MEM_SIZE,
	.has_vendor_key_hash = true,
	.ecc_strap_addr      = OTPSTRAP14_ADDR,
};

static const struct aspeed_otp_plat_data ast2705_otp_plat_data = {
	.gran_bits  = 32,
	.region_ecc = NULL,
	.mem_words  = AST2705_OTP_MEM_SIZE,
};

static bool otp_region_ecc_active(struct aspeed_otp *ctx, u32 offset)
{
	struct otp_region_ecc *region;
	int i;

	if (!ctx->plat->region_ecc)
		return ctx->gbl_ecc_en;

	for (i = 0; i < OTP_REGION_MAX; i++) {
		region = &ctx->region_ecc[i];
		if (offset < region->start || offset >= region->end)
			continue;

		if (!region->ecc_supported)
			return false;

		return ctx->gbl_ecc_en || region->ecc_en;
	}

	return false;
}

static void otp_unlock(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);

	writel(OTP_PASSWD, ctx->base + OTP_KEY);
}

/*
 * OTP_KEY (offset 0x0) is a single lock register shared by all OTP
 * masters, not per-master, so calling this after a normal read/prog
 * could lock out other masters concurrently accessing OTP. All call
 * sites are disabled for that reason; kept (__maybe_unused) in case
 * a caller intentionally needs to force-lock the whole OTP block.
 */
static void __maybe_unused otp_lock(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);

	writel(0x1, ctx->base + OTP_KEY);
}

static int wait_complete(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	u32 cmd_sts, addr, val;
	int ret;

	ret = readl_poll_timeout(ctx->base + OTP_STATUS, val,
				  !(val & OTP_STS_BUSY), 1, OTP_TIMEOUT_US);
	if (ret) {
		dev_warn(dev, "timeout. sts:0x%x\n", val);
		return ret;
	}

	cmd_sts = OTP_GET_CMD_STS(val);
	if (cmd_sts == OTP_STS_PASS)
		return OTP_SUCCESS;

	addr = readl(ctx->base + OTP_ADDR);

	switch (cmd_sts) {
	case OTP_STS_FAIL:
		dev_warn(dev, "prog fail or soak limit exceeded at addr 0x%x\n", addr);
		return -OTP_CMD_ERR_FAIL;
	case OTP_STS_CMP_FAIL:
		dev_warn(dev, "compare mismatch at addr 0x%x\n", addr);
		return -OTP_CMD_ERR_CMP_FAIL;
	case OTP_STS_REGION_FAIL:
		dev_warn(dev, "region write/read protected at addr 0x%x\n", addr);
		return -OTP_CMD_ERR_REGION_FAIL;
	case OTP_STS_MASTER_FAIL:
		dev_warn(dev, "master protection error at addr 0x%x\n", addr);
		return -OTP_CMD_ERR_MASTER_FAIL;
	default:
		dev_warn(dev, "unknown cmd sts:0x%x\n", cmd_sts);
		return -EIO;
	}
}

static int otp_check_ecc_status(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	u32 status, addr, syndrome;

	status = readl(ctx->base + OTP_DBG01);
	syndrome = OTP_ECC_STS_SYNDROME(status);

	if (!syndrome)
		return 0;

	addr = readl(ctx->base + OTP_ADDR);

	if (status & OTP_ECC_STS_SINGLE_ERR) {
		dev_dbg(dev, "single-bit ECC error corrected, addr:0x%x, syndrome:0x%x\n",
			addr, syndrome);
		return 0;
	}

	dev_warn(dev, "uncorrectable dual-bit ECC error, addr:0x%x, syndrome:0x%x\n",
		 addr, syndrome);
	return -OTP_ECC_ERR_DUAL;
}

static void otp_ecc_cfg(struct device *dev, bool ecc_en, bool auto_cfg)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	bool self_cfg = ecc_en && !auto_cfg;

	writel(ecc_en, ctx->base + OTP_ECC_EN);

	/* Self config or auto config */
	writel(self_cfg ? 0x4 : 0x0, ctx->base + OTP_PMC_CQ);
	/* Clearing OTP_PMC_CQ auto-reverts OTP_DAP_CFG_RQ, no explicit write needed to disable */
	if (self_cfg)
		writel(0x40008, ctx->base + OTP_DAP_CFG_RQ);
}

static int otp_read_data(struct aspeed_otp *ctx, u32 offset, u32 *data)
{
	struct device *dev = ctx->dev;
	bool ecc_en = otp_region_ecc_active(ctx, offset);
	int ret;

	writel(offset, ctx->base + OTP_ADDR);
	otp_ecc_cfg(dev, ecc_en, false);

	writel(OTP_CMD_READ, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (!ret)
		data[0] = readl(ctx->base + OTP_RDATA);

	if (!ret && ecc_en)
		ret = otp_check_ecc_status(dev);

	/* Restore ECC config to default */
	otp_ecc_cfg(dev, ecc_en, true);

	return ret;
}

static int otp_prog_data(struct aspeed_otp *ctx, u32 offset, u32 data)
{
	struct device *dev = ctx->dev;

	writel(otp_region_ecc_active(ctx, offset), ctx->base + OTP_ECC_EN);
	writel(0x0, ctx->base + OTP_PMC_CQ);

	writel(offset, ctx->base + OTP_ADDR);
	writel(data, ctx->base + OTP_WDATA_0);
	writel(OTP_CMD_PROG, ctx->base + OTP_CMD);

	return wait_complete(dev);
}

static int otp_prog_multi_data(struct aspeed_otp *ctx, u32 offset, u32 *data, int count)
{
	struct device *dev = ctx->dev;

	if (count > 4)
		return -EINVAL;

	writel(otp_region_ecc_active(ctx, offset), ctx->base + OTP_ECC_EN);
	writel(0x0, ctx->base + OTP_PMC_CQ);

	writel(offset, ctx->base + OTP_ADDR);
	for (int i = 0; i < count; i++)
		writel(data[i], ctx->base + OTP_WDATA_0 + 4 * i);

	writel(OTP_CMD_PROG_MULTI, ctx->base + OTP_CMD);

	return wait_complete(dev);
}

static int aspeed_otp_read(struct aspeed_otp *ctx, int offset,
			   void *buf, int size)
{
	struct device *dev = ctx->dev;
	u8 gran = ctx->plat->gran_bits;
	u32 rdata;
	int ret = 0;

	otp_unlock(dev);
	for (int i = 0; i < size; i++) {
		ret = otp_read_data(ctx, offset + i, &rdata);
		if (ret) {
			dev_warn(ctx->dev, "read failed\n");
			break;
		}
		if (gran == 32)
			((u32 *)buf)[i] = rdata;
		else
			((u16 *)buf)[i] = (u16)rdata;
	}

	/* otp_lock(dev); see comment on otp_lock() */
	return ret;
}

static int aspeed_otp_write(struct aspeed_otp *ctx, int offset,
			    const void *buf, int size)
{
	struct device *dev = ctx->dev;
	u8 gran = ctx->plat->gran_bits;
	int ret;

	otp_unlock(dev);

	if (gran == 32) {
		const u32 *data = buf;

		if (size == 1)
			ret = otp_prog_data(ctx, offset, data[0]);
		else
			ret = otp_prog_multi_data(ctx, offset, (u32 *)data, size);
	} else {
		const u16 *data = buf;

		if (size == 1)
			ret = otp_prog_data(ctx, offset, data[0]);
		else
			ret = otp_prog_multi_data(ctx, offset, (u32 *)data, size / 2);
	}

	if (ret)
		dev_warn(ctx->dev, "prog failed\n");

	/* otp_lock(dev); see comment on otp_lock() */
	return ret;
}

static int aspeed_otp_ecc_en(struct aspeed_otp *ctx, int enable)
{
	ctx->gbl_ecc_en = enable ? ECC_ENABLE : ECC_DISABLE;
	return 0;
}

#ifdef CONFIG_AST2700_OTP_SYSFS
static ssize_t vendor_key_hash_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	u32 offset = AST2700_CAL_REGION_START_ADDR + OTPCAL_VKEY_HASH_W_OFFSET;
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	u16 data[OTPCAL_VKEY_HASH_WORDS];
	u8 *bytes = (u8 *)data;
	ssize_t len = 0;
	int ret, i;

	ret = aspeed_otp_read(ctx, offset, data, ARRAY_SIZE(data));
	if (ret)
		return -EIO;

	for (i = 0; i < sizeof(data); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02x", bytes[i]);

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}
static DEVICE_ATTR_RO(vendor_key_hash);

static struct attribute *aspeed_otp_attrs[] = {
	&dev_attr_vendor_key_hash.attr,
	NULL,
};

static const struct attribute_group aspeed_otp_attr_group = {
	.attrs = aspeed_otp_attrs,
};
#endif /* CONFIG_AST2700_OTP_SYSFS */

static long aspeed_otp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_otp *ctx = container_of(c, struct aspeed_otp, miscdev);
	void __user *argp = (void __user *)arg;
	struct otp_revid revid;
	struct otp_read rdata;
	struct otp_prog pdata;
	struct otp_ecc_policy policy;
	unsigned int gran_bytes;
	int ret = 0;

	gran_bytes = ctx->plat->gran_bits / 8;

	switch (cmd) {
	case ASPEED_OTP_READ_DATA:
		if (copy_from_user(&rdata, argp, sizeof(struct otp_read)))
			return -EFAULT;

		if (rdata.len == 0 || rdata.len > ctx->plat->mem_words ||
		    rdata.offset > ctx->plat->mem_words - rdata.len)
			return -EINVAL;

		ret = aspeed_otp_read(ctx, rdata.offset, ctx->data, rdata.len);
		if (ret)
			return ret;

		if (copy_to_user(rdata.data, ctx->data, rdata.len * gran_bytes))
			return -EFAULT;

		break;

	case ASPEED_OTP_PROG_DATA:
		if (copy_from_user(&pdata, argp, sizeof(struct otp_prog)))
			return -EFAULT;

		if (pdata.len == 0 || pdata.len > ctx->plat->mem_words ||
		    pdata.w_offset > ctx->plat->mem_words - pdata.len)
			return -EINVAL;

		if (copy_from_user(ctx->data, (const void __user *)pdata.data,
				   pdata.len * gran_bytes))
			return -EFAULT;

		ret = aspeed_otp_write(ctx, pdata.w_offset, ctx->data, pdata.len);
		break;

	case ASPEED_OTP_GET_ECC:
		if (copy_to_user(argp, &ctx->gbl_ecc_en, sizeof(ctx->gbl_ecc_en)))
			return -EFAULT;
		break;

	case ASPEED_OTP_SET_ECC:
		ret = aspeed_otp_ecc_en(ctx, (int)arg);
		break;

	case ASPEED_OTP_GET_REVID:
		revid.revid0 = ctx->chip_revid0;
		revid.revid1 = ctx->chip_revid1;
		if (copy_to_user(argp, &revid, sizeof(struct otp_revid)))
			return -EFAULT;
		break;

	case ASPEED_OTP_GET_ECC_POLICY:
		if (!ctx->plat->region_ecc)
			return -EOPNOTSUPP;

		if (copy_from_user(&policy, argp, sizeof(policy)))
			return -EFAULT;

		if (policy.region >= OTP_REGION_MAX)
			return -EINVAL;

		policy.ecc_en = ctx->region_ecc[policy.region].ecc_en;
		policy.ecc_supported = ctx->region_ecc[policy.region].ecc_supported;

		if (copy_to_user(argp, &policy, sizeof(policy)))
			return -EFAULT;
		break;

	case ASPEED_OTP_SET_ECC_POLICY:
		if (!ctx->plat->region_ecc)
			return -EOPNOTSUPP;

		if (copy_from_user(&policy, argp, sizeof(policy)))
			return -EFAULT;

		if (policy.region >= OTP_REGION_MAX)
			return -EINVAL;

		if (policy.ecc_en && !ctx->region_ecc[policy.region].ecc_supported)
			return -EINVAL;

		ctx->region_ecc[policy.region].ecc_en = !!policy.ecc_en;
		break;

	default:
		dev_warn(ctx->dev, "cmd 0x%x is not supported\n", cmd);
		break;
	}

	return ret;
}

static int aspeed_otp_ecc_init(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	int ret;
	u32 val;

	if (!ctx->plat->ecc_strap_addr) {
		ctx->gbl_ecc_en = 0;
		return 0;
	}

	otp_unlock(dev);

	/* Check cfg_ecc_en */
	writel(0, ctx->base + OTP_ECC_EN);
	writel(ctx->plat->ecc_strap_addr, ctx->base + OTP_ADDR);
	writel(OTP_CMD_READ, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return ret;

	val = readl(ctx->base + OTP_RDATA);
	if (val & 0x1)
		ctx->gbl_ecc_en = 0x1;
	else
		ctx->gbl_ecc_en = 0x0;

	/* otp_lock(dev); see comment on otp_lock() */

	return 0;
}

static int aspeed_otp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_otp *ctx = container_of(c, struct aspeed_otp, miscdev);

	spin_lock(&otp_state_lock);

	if (ctx->is_open) {
		spin_unlock(&otp_state_lock);
		return -EBUSY;
	}

	ctx->is_open = true;

	spin_unlock(&otp_state_lock);

	return 0;
}

static int aspeed_otp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_otp *ctx = container_of(c, struct aspeed_otp, miscdev);

	spin_lock(&otp_state_lock);

	ctx->is_open = false;

	spin_unlock(&otp_state_lock);

	return 0;
}

static const struct file_operations otp_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	aspeed_otp_ioctl,
	.open =			aspeed_otp_open,
	.release =		aspeed_otp_release,
};

static const struct of_device_id aspeed_otp_of_matches[] = {
	{ .compatible = "aspeed,ast2700-otp", .data = &ast2700_otp_plat_data },
	{ .compatible = "aspeed,ast2705-otp", .data = &ast2705_otp_plat_data },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_otp_of_matches);

static int aspeed_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *scu0, *scu1;
	struct aspeed_otp *priv;
	struct resource *res;
	int rc;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->plat = device_get_match_data(dev);
	if (!priv->plat)
		return -ENODEV;

	if (priv->plat->region_ecc)
		memcpy(priv->region_ecc, priv->plat->region_ecc, sizeof(priv->region_ecc));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (!priv->base)
		return -EIO;

	scu0 = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu0");
	scu1 = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu1");
	if (IS_ERR(scu0) || IS_ERR(scu1)) {
		dev_err(dev, "failed to find SCU regmap\n");
		return PTR_ERR(scu0) || PTR_ERR(scu1);
	}

	regmap_read(scu0, 0x0, &priv->chip_revid0);
	regmap_read(scu1, 0x0, &priv->chip_revid1);

	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	/* OTP ECC init */
	rc = aspeed_otp_ecc_init(dev);
	if (rc)
		return -EIO;

	priv->data = kmalloc(priv->plat->mem_words * (priv->plat->gran_bits / 8), GFP_KERNEL);
	if (!priv->data)
		return -ENOMEM;

	/* Set up the miscdevice */
	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = "aspeed-otp";
	priv->miscdev.fops = &otp_fops;

	/* Register the device */
	rc = misc_register(&priv->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

#ifdef CONFIG_AST2700_OTP_SYSFS
	if (priv->plat->has_vendor_key_hash) {
		rc = devm_device_add_group(dev, &aspeed_otp_attr_group);
		if (rc) {
			dev_err(dev, "failed to add sysfs attributes\n");
			misc_deregister(&priv->miscdev);
			return rc;
		}
	}
#endif

	dev_info(dev, "Aspeed OTP driver successfully registered\n");

	return 0;
}

static void aspeed_otp_remove(struct platform_device *pdev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(&pdev->dev);

	kfree(ctx->data);
	misc_deregister(&ctx->miscdev);
}

static struct platform_driver aspeed_otp_driver = {
	.probe = aspeed_otp_probe,
	.remove = aspeed_otp_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_otp_of_matches,
	},
};

module_platform_driver(aspeed_otp_driver);

MODULE_AUTHOR("Neal Liu <neal_liu@aspeedtech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASPEED OTP Driver");
