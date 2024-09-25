// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Aspeed Technology Inc.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/module.h>
#include <asm/io.h>
#include <uapi/linux/aspeed-otp.h>

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
#define OTP_MASTER			OTP_M0

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

/* OTP ECC EN */
#define ECC_ENABLE			0x1
#define ECC_DISABLE			0x0
#define ECCBRP_EN			BIT(0)

#define ROM_REGION_START_ADDR		0x0
#define ROM_REGION_END_ADDR		0x3e0
#define RBP_REGION_START_ADDR		ROM_REGION_END_ADDR
#define RBP_REGION_END_ADDR		0x400
#define CONF_REGION_START_ADDR		RBP_REGION_END_ADDR
#define CONF_REGION_END_ADDR		0x420
#define STRAP_REGION_START_ADDR		CONF_REGION_END_ADDR
#define STRAP_REGION_END_ADDR		0x430
#define STRAPEXT_REGION_START_ADDR	STRAP_REGION_END_ADDR
#define STRAPEXT_REGION_END_ADDR	0x440
#define USER_REGION_START_ADDR		STRAPEXT_REGION_END_ADDR
#define USER_REGION_END_ADDR		0x1000
#define SEC_REGION_START_ADDR		USER_REGION_END_ADDR
#define SEC_REGION_END_ADDR		0x1c00
#define CAL_REGION_START_ADDR		SEC_REGION_END_ADDR
#define CAL_REGION_END_ADDR		0x1f80
#define SW_PUF_REGION_START_ADDR	CAL_REGION_END_ADDR
#define SW_PUF_REGION_END_ADDR		0x1fc0
#define HW_PUF_REGION_START_ADDR	SW_PUF_REGION_END_ADDR
#define HW_PUF_REGION_END_ADDR		0x2000

#define OTP_MEMORY_SIZE			(HW_PUF_REGION_END_ADDR * 2)

#define OTP_TIMEOUT_US			10000

/* OTPSTRAP */
#define OTPSTRAP0_ADDR			STRAP_REGION_START_ADDR
#define OTPSTRAP14_ADDR			(OTPSTRAP0_ADDR + 0xe)

#define OTPTOOL_VERSION(a, b, c)	(((a) << 24) + ((b) << 12) + (c))
#define OTPTOOL_VERSION_MAJOR(x)	(((x) >> 24) & 0xff)
#define OTPTOOL_VERSION_PATCHLEVEL(x)	(((x) >> 12) & 0xfff)
#define OTPTOOL_VERSION_SUBLEVEL(x)	((x) & 0xfff)
#define OTPTOOL_COMPT_VERSION		2

enum otp_error_code {
	OTP_SUCCESS,
	OTP_READ_FAIL,
	OTP_PROG_FAIL,
	OTP_CMP_FAIL,
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

struct aspeed_otp {
	struct miscdevice	miscdev;
	struct device		*dev;
	void __iomem		*base;
	u32			chip_revid0;
	u32			chip_revid1;
	bool			is_open;
	int			gbl_ecc_en;
	u8			*data;
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

static void otp_unlock(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);

	writel(OTP_PASSWD, ctx->base + OTP_KEY);
}

static void otp_lock(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);

	writel(0x1, ctx->base + OTP_KEY);
}

static int wait_complete(struct device *dev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(dev);
	int ret;
	u32 val;

	ret = readl_poll_timeout(ctx->base + OTP_STATUS, val, (val == 0x0),
				 1, OTP_TIMEOUT_US);
	if (ret)
		dev_warn(dev, "timeout. sts:0x%x\n", val);

	return ret;
}

static int otp_read_data(struct aspeed_otp *ctx, u32 offset, u16 *data)
{
	struct device *dev = ctx->dev;
	int ret;

	writel(ctx->gbl_ecc_en, ctx->base + OTP_ECC_EN);
	writel(offset, ctx->base + OTP_ADDR);
	writel(OTP_CMD_READ, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_READ_FAIL;

	data[0] = readl(ctx->base + OTP_RDATA);

	return 0;
}

int otp_prog_data(struct aspeed_otp *ctx, u32 offset, u16 data)
{
	struct device *dev = ctx->dev;
	int ret;

	writel(ctx->gbl_ecc_en, ctx->base + OTP_ECC_EN);
	writel(offset, ctx->base + OTP_ADDR);
	writel(data, ctx->base + OTP_WDATA_0);
	writel(OTP_CMD_PROG, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_PROG_FAIL;

	return 0;
}

int otp_prog_multi_data(struct aspeed_otp *ctx, u32 offset, u32 *data, int count)
{
	struct device *dev = ctx->dev;
	int ret;

	writel(ctx->gbl_ecc_en, ctx->base + OTP_ECC_EN);
	writel(offset, ctx->base + OTP_ADDR);
	for (int i = 0; i < count; i++)
		writel(data[i], ctx->base + OTP_WDATA_0 + 4 * i);

	writel(OTP_CMD_PROG_MULTI, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_PROG_FAIL;

	return 0;
}

static int aspeed_otp_read(struct aspeed_otp *ctx, int offset,
			   void *buf, int size)
{
	struct device *dev = ctx->dev;
	u16 *data = buf;
	int ret;

	otp_unlock(dev);
	for (int i = 0; i < size; i++) {
		ret = otp_read_data(ctx, offset + i, data + i);
		if (ret) {
			dev_warn(ctx->dev, "read failed\n");
			break;
		}
	}

	otp_lock(dev);
	return ret;
}

static int aspeed_otp_write(struct aspeed_otp *ctx, int offset,
			    const void *buf, int size)
{
	struct device *dev = ctx->dev;
	u32 *data32 = (u32 *)buf;
	u16 *data = (u16 *)buf;
	int ret;

	otp_unlock(dev);

	if (size == 1)
		ret = otp_prog_data(ctx, offset, data[0]);
	else
		ret = otp_prog_multi_data(ctx, offset, data32, size / 2);

	if (ret)
		dev_warn(ctx->dev, "prog failed\n");

	otp_lock(dev);
	return ret;
}

static int aspeed_otp_ecc_en(struct aspeed_otp *ctx)
{
	struct device *dev = ctx->dev;
	int ret = 0;

	/* Check ecc is already enabled */
	if (ctx->gbl_ecc_en == 1)
		return 0;

	otp_unlock(dev);

	/* enable cfg ecc */
	ret = otp_prog_data(ctx, OTPSTRAP14_ADDR, 0x1);
	if (ret) {
		dev_warn(dev, "%s: prog failed\n", __func__);
		goto end;
	}

	ctx->gbl_ecc_en = 1;
end:
	otp_lock(dev);

	return ret;
}

static long aspeed_otp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_otp *ctx = container_of(c, struct aspeed_otp, miscdev);
	void __user *argp = (void __user *)arg;
	struct otp_revid revid;
	struct otp_read rdata;
	struct otp_prog pdata;
	int ret = 0;

	switch (cmd) {
	case ASPEED_OTP_READ_DATA:
		if (copy_from_user(&rdata, argp, sizeof(struct otp_read)))
			return -EFAULT;

		ret = aspeed_otp_read(ctx, rdata.offset, ctx->data, rdata.len);
		if (ret)
			return -EFAULT;

		if (copy_to_user(rdata.data, ctx->data, rdata.len * 2))
			return -EFAULT;

		break;

	case ASPEED_OTP_PROG_DATA:
		if (copy_from_user(&pdata, argp, sizeof(struct otp_prog)))
			return -EFAULT;

		ret = aspeed_otp_write(ctx, pdata.w_offset, pdata.data, pdata.len);
		break;

	case ASPEED_OTP_GET_ECC:
		if (copy_to_user(argp, &ctx->gbl_ecc_en, sizeof(ctx->gbl_ecc_en)))
			return -EFAULT;
		break;

	case ASPEED_OTP_SET_ECC:
		ret = aspeed_otp_ecc_en(ctx);
		break;

	case ASPEED_OTP_GET_REVID:
		revid.revid0 = ctx->chip_revid0;
		revid.revid1 = ctx->chip_revid1;
		if (copy_to_user(argp, &revid, sizeof(struct otp_revid)))
			return -EFAULT;
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

	otp_unlock(dev);

	/* Check cfg_ecc_en */
	writel(0, ctx->base + OTP_ECC_EN);
	writel(OTPSTRAP14_ADDR, ctx->base + OTP_ADDR);
	writel(OTP_CMD_READ, ctx->base + OTP_CMD);
	ret = wait_complete(dev);
	if (ret)
		return OTP_READ_FAIL;

	val = readl(ctx->base + OTP_RDATA);
	if (val & 0x1)
		ctx->gbl_ecc_en = 0x1;
	else
		ctx->gbl_ecc_en = 0x0;

	otp_lock(dev);

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
	{ .compatible = "aspeed,ast2700-otp" },
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

	priv->data = kmalloc(OTP_MEMORY_SIZE, GFP_KERNEL);
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

	dev_info(dev, "Aspeed OTP driver successfully registered\n");

	return 0;
}

static int aspeed_otp_remove(struct platform_device *pdev)
{
	struct aspeed_otp *ctx = dev_get_drvdata(&pdev->dev);

	kfree(ctx->data);
	misc_deregister(&ctx->miscdev);

	return 0;
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
