// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2026 ASPEED Technology Inc.
 *
 * Caliptra-SS MCI (Manageability Controller Interface) driver: mailbox
 * page-select/lock/execute transport, a direct (non-mailbox) status
 * register read, and a userspace-facing ioctl passthrough for both. See
 * the header comment on the ioctl structs below for how command-specific
 * request/response layout and semantics are kept out of this driver
 * entirely.
 *
 * Unlike the legacy always-mapped Caliptra mailbox, the MCI mailbox's CSR
 * block and its data SRAM are not directly addressable: both are aliased
 * onto the same 64KB local window, and an SCU1 register has to be written
 * with a 64KB-aligned page target to select which one is currently visible
 * in that window.
 *
 * Mailbox transport ported from u-boot's drivers/misc/aspeed_cptra_mci_mbox.c.
 * u-boot is single-threaded, so it serializes solely via the mailbox's own
 * HW LOCK register; Linux additionally needs a mutex around every use of
 * the shared page-select window, since the HW LOCK does not cover
 * aspeed_cptra_mci_reg_read() and unrelated callers can otherwise race on
 * which page is currently mapped. The command-completion poll loop is also
 * bounded here (readl_poll_timeout()) instead of looping forever, since an
 * unbounded busy-wait is not acceptable inside the kernel.
 *
 * Also registers /dev/aspeed-cptra-mci, a misc device exposing
 * CPTRA_MCI_IOC_MBOX_EXECUTE and CPTRA_MCI_IOC_REG_READ (defined below --
 * no uapi header, see the comment on those structs) as thin ioctl
 * passthroughs onto aspeed_cptra_mci_mbox_execute() and
 * aspeed_cptra_mci_reg_read() respectively, so a userspace test tool can
 * exercise every command this driver knows about without ever mapping
 * /dev/mem or reimplementing this file's page-select/lock/execute protocol
 * itself.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/aspeed/aspeed-cptra-mci.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/util_macros.h>

/*
 * /dev/aspeed-cptra-mci ioctl: no uapi header, same as this SoC's legacy
 * aspeed-mbox.c/ASPEED_MBOX_IOCTL_* -- the userspace caller (aspeed_cptra_
 * mci.c) just defines an identical copy of this struct and ioctl number
 * locally rather than sharing a header across the kernel/userspace repo
 * boundary.
 */
struct cptra_mci_ioc_mbox_execute {
	u32 cmd;
	u32 req_len;
	u32 resp_buf_len;
	u32 resp_len;
	u64 req_ptr;
	u64 resp_ptr;
};

/*
 * Direct (non-mailbox-protocol) status register read -- see the comment on
 * aspeed_cptra_mci_reg_read(). page/offset are the CPTRA_MCI_REG_* /
 * CPTRA_MCI_SOC_IFC_* values the userspace caller already has to know
 * regardless (they pick which register), so this ioctl is just as thin a
 * passthrough as CPTRA_MCI_IOC_MBOX_EXECUTE.
 */
struct cptra_mci_ioc_reg_read {
	u32 page;
	u32 offset;
	u32 value;
};

#define __CPTRA_MCI_IOC_MAGIC		'M'
#define CPTRA_MCI_IOC_MBOX_EXECUTE	_IOWR(__CPTRA_MCI_IOC_MAGIC, 0x01, \
					     struct cptra_mci_ioc_mbox_execute)
#define CPTRA_MCI_IOC_REG_READ		_IOWR(__CPTRA_MCI_IOC_MAGIC, 0x02, \
					     struct cptra_mci_ioc_reg_read)

/*
 * SCU1 register offset used to remap the MCI mailbox's 64KB local window,
 * and the page targets written to it. Both the offset and the page-target
 * encoding are specific to how this SoC's ARM-side bus master reaches the
 * MCI block, and differ from the encoding used by other bus masters (e.g.
 * the on-chip MCU's own view of the same hardware uses a different SCU1
 * offset and writes the full, unshifted target address) -- do not reuse
 * these values for a different bus master without re-deriving them.
 */
#define SCU1_CPTRA_SS_AXI_WIN		0x3c4

/*
 * Page targets for SCU1_CPTRA_SS_AXI_WIN, encoded as bits [31:16] of the
 * target address (i.e. the target's 64KB-aligned page number). CSR_PAGE was
 * confirmed on real hardware; SRAM_PAGE is inferred by the same encoding
 * and not yet independently confirmed.
 */
#define CPTRA_MCI_MBOX_SRAM_PAGE	0x2140	/* mailbox data (0x21400000 >> 16) */
#define CPTRA_MCI_MBOX_CSR_PAGE		0x2160	/* mcu_mbox0_csr (0x21600000 >> 16) */

struct aspeed_cptra_mci {
	struct device *dev;
	void __iomem *regs;
	resource_size_t regs_size;
	struct regmap *scu1;
	struct miscdevice miscdev;
	/*
	 * Held by every /dev/aspeed-cptra-mci open fd (see chardev_open()/
	 * chardev_release()) plus one reference from probe(). remove() drops
	 * the probe reference and waits on `released` so devm doesn't free
	 * this struct (or unmap regs) while an already-open fd can still
	 * reach it via file->private_data, bypassing the cptra_mci_lock
	 * lookup entirely.
	 */
	struct kref refcount;
	struct completion released;
};

/*
 * Singleton: only one Caliptra-SS MCI instance exists per SoC. This mutex
 * both guards cptra_mci itself and doubles as the page-select window lock
 * (which page is mapped, plus the read/write that follows, must stay
 * atomic with respect to any other caller reusing the same window):
 * folding the lookup and the critical section into one lock means a caller
 * either completes its access before remove() can null out cptra_mci, or
 * sees NULL and never touches priv at all -- there is no gap between
 * "looked up a non-NULL priv" and "started using it" for remove() to free
 * priv out from under.
 */
static DEFINE_MUTEX(cptra_mci_lock);
static struct aspeed_cptra_mci *cptra_mci;

static struct aspeed_cptra_mci *aspeed_cptra_mci_lock(void)
{
	mutex_lock(&cptra_mci_lock);
	if (!cptra_mci) {
		mutex_unlock(&cptra_mci_lock);
		return NULL;
	}

	return cptra_mci;
}

static void aspeed_cptra_mci_unlock(void)
{
	mutex_unlock(&cptra_mci_lock);
}

static void aspeed_cptra_mci_release(struct kref *kref)
{
	struct aspeed_cptra_mci *priv = container_of(kref, struct aspeed_cptra_mci, refcount);

	complete(&priv->released);
}

static int cptra_mci_mbox_select_page(struct aspeed_cptra_mci *priv, u32 page)
{
	u32 val;
	int ret;

	ret = regmap_write(priv->scu1, SCU1_CPTRA_SS_AXI_WIN, page);
	if (ret) {
		dev_dbg(priv->dev, "failed to write SCU1 page-select (page 0x%x): %d\n", page, ret);
		return ret;
	}

	ret = regmap_read(priv->scu1, SCU1_CPTRA_SS_AXI_WIN, &val);
	if (ret) {
		dev_dbg(priv->dev, "failed to read back SCU1 page-select: %d\n", ret);
		return ret;
	}

	if (val != page) {
		dev_dbg(priv->dev, "failed to select page 0x%x (read back 0x%x)\n", page, val);
		return -EIO;
	}

	return 0;
}

static void cptra_mci_mbox_sram_write(struct aspeed_cptra_mci *priv,
				      const void *data, u32 len, u32 offset)
{
	const u8 *p8 = data;
	u32 word;
	u32 i;

	for (i = 0; (i + sizeof(u32)) <= len; i += sizeof(u32)) {
		memcpy(&word, p8 + i, sizeof(word));
		writel(word, priv->regs + offset + i);
	}

	if (i < len) {
		word = 0;
		memcpy(&word, p8 + i, len - i);
		writel(word, priv->regs + offset + i);
	}
}

static void cptra_mci_mbox_sram_read(struct aspeed_cptra_mci *priv, void *data, u32 len)
{
	u8 *p8 = data;
	u32 word;
	u32 i;

	for (i = 0; (i + sizeof(u32)) <= len; i += sizeof(u32)) {
		word = readl(priv->regs + i);
		memcpy(p8 + i, &word, sizeof(word));
	}

	if (i < len) {
		word = readl(priv->regs + i);
		memcpy(p8 + i, &word, len - i);
	}
}

/*
 * LOCK reads 0 (free) and atomically latches to locked as a side effect of
 * the read. A second read confirming 1 is required to know the lock
 * actually latched, rather than trusting the first read.
 */
static int cptra_mci_mbox_lock(struct aspeed_cptra_mci *priv)
{
	int ret;

	ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		return ret;

	if (readl(priv->regs + CPTRA_MCI_MBOX_LOCK))
		return -EBUSY;

	if (!readl(priv->regs + CPTRA_MCI_MBOX_LOCK)) {
		/*
		 * The first read above already latched the lock as a side
		 * effect, regardless of what this confirmation read reports.
		 * Release it here so a glitched confirmation read doesn't
		 * wedge the mailbox for every later caller.
		 */
		ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_CSR_PAGE);
		if (ret)
			dev_warn(priv->dev,
				 "failed to select CSR page while releasing lock: %d (mailbox may stay wedged)\n",
				 ret);
		writel(0x0, priv->regs + CPTRA_MCI_MBOX_EXECUTE);
		return -EIO;
	}

	return 0;
}

static void cptra_mci_mbox_unlock(struct aspeed_cptra_mci *priv)
{
	int ret;

	ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		dev_warn(priv->dev,
			 "failed to select CSR page while unlocking: %d (mailbox may stay wedged)\n",
			 ret);
	writel(0x0, priv->regs + CPTRA_MCI_MBOX_EXECUTE);
}

static int cptra_mci_mbox_wait_lock(struct aspeed_cptra_mci *priv)
{
	unsigned long deadline = jiffies + usecs_to_jiffies(CPTRA_MCI_MBOX_LOCK_TIMEOUT_US);
	int ret;

	while ((ret = cptra_mci_mbox_lock(priv)) == -EBUSY) {
		if (time_after(jiffies, deadline)) {
			dev_dbg(priv->dev, "timed out waiting for lock\n");
			return -ETIMEDOUT;
		}
		usleep_range(50, 100);
	}

	return ret;
}

struct cptra_mci_mbox_iov {
	const void *base;
	u32 len;
};

/*
 * Shared implementation. req is scattered across up to two buffers (a small
 * fixed-size header struct plus a separate, possibly large, payload buffer)
 * so callers never need to memcpy a large payload into one contiguous
 * request struct just to hand it to this function.
 *
 * Caller must already hold cptra_mci_lock and have a valid priv (either via
 * aspeed_cptra_mci_lock() for the exported API, or via its own reference for
 * the ioctl path -- see cptra_mci_ioctl_mbox_execute()).
 */
static int cptra_mci_mbox_execute_iov_locked(struct aspeed_cptra_mci *priv, u32 cmd,
					     const struct cptra_mci_mbox_iov *iov, int iovcnt,
					     void *resp, u32 resp_buf_len, u32 *resp_len)
{
	u32 status, sts, dlen, req_len, off;
	int ret, i;

	/*
	 * Accumulate with an overflow check on every step rather than summing
	 * first and comparing after: iov[i].len is caller-controlled (directly
	 * from userspace for the ioctl path), and two large enough lengths can
	 * wrap a u32 sum back under CPTRA_MCI_MBOX_SRAM_SIZE, defeating the
	 * bound check while cptra_mci_mbox_sram_write() below still writes the
	 * full, huge, original length past the end of the mapped window.
	 */
	req_len = 0;
	for (i = 0; i < iovcnt; i++) {
		if (iov[i].len > CPTRA_MCI_MBOX_SRAM_SIZE - req_len)
			return -EINVAL;
		req_len += iov[i].len;
	}

	ret = cptra_mci_mbox_wait_lock(priv);
	if (ret)
		return ret;

	writel(cmd, priv->regs + CPTRA_MCI_MBOX_CMD);
	writel(req_len, priv->regs + CPTRA_MCI_MBOX_DLEN);

	ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_SRAM_PAGE);
	if (ret)
		goto unlock;

	off = 0;
	for (i = 0; i < iovcnt; i++) {
		cptra_mci_mbox_sram_write(priv, iov[i].base, iov[i].len, off);
		off += iov[i].len;
	}

	ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_CSR_PAGE);
	if (ret)
		goto unlock;

	writel(0x1, priv->regs + CPTRA_MCI_MBOX_EXECUTE);

	ret = readl_poll_timeout(priv->regs + CPTRA_MCI_MBOX_CMD_STATUS, status,
				 FIELD_GET(CPTRA_MCI_MBOX_CMD_STATUS_PS, status) !=
				 CPTRA_MCI_MBSTS_CMD_BUSY,
				 10, CPTRA_MCI_MBOX_EXEC_TIMEOUT_US);
	if (ret) {
		dev_dbg(priv->dev, "cmd 0x%x timed out\n", cmd);
		goto unlock;
	}

	sts = FIELD_GET(CPTRA_MCI_MBOX_CMD_STATUS_PS, status);
	if (sts == CPTRA_MCI_MBSTS_CMD_FAILURE) {
		dev_dbg(priv->dev, "cmd 0x%x failed\n", cmd);
		ret = -EIO;
		goto unlock;
	}

	dlen = readl(priv->regs + CPTRA_MCI_MBOX_DLEN);
	if (dlen > CPTRA_MCI_MBOX_SRAM_SIZE) {
		dev_dbg(priv->dev, "invalid dlen 0x%x\n", dlen);
		ret = -EIO;
		goto unlock;
	}
	if (dlen > resp_buf_len) {
		dev_dbg(priv->dev, "response 0x%x exceeds buffer 0x%x\n", dlen, resp_buf_len);
		ret = -ENOSPC;
		goto unlock;
	}

	ret = cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_SRAM_PAGE);
	if (ret)
		goto unlock;

	cptra_mci_mbox_sram_read(priv, resp, dlen);

	if (resp_len)
		*resp_len = dlen;

	dev_dbg(priv->dev, "cmd 0x%x completed, dlen=%u\n", cmd, dlen);

	ret = 0;

unlock:
	cptra_mci_mbox_unlock(priv);

	return ret;
}

/* Does the cptra_mci lookup + locking; see cptra_mci_mbox_execute_iov_locked(). */
static int cptra_mci_mbox_execute_iov(u32 cmd, const struct cptra_mci_mbox_iov *iov,
				      int iovcnt, void *resp, u32 resp_buf_len,
				      u32 *resp_len)
{
	struct aspeed_cptra_mci *priv;
	int ret;

	priv = aspeed_cptra_mci_lock();
	if (!priv)
		return -EPROBE_DEFER;

	ret = cptra_mci_mbox_execute_iov_locked(priv, cmd, iov, iovcnt, resp, resp_buf_len,
						resp_len);

	aspeed_cptra_mci_unlock();

	return ret;
}

int aspeed_cptra_mci_mbox_execute(u32 cmd, const void *req, u32 req_len,
				  void *resp, u32 resp_buf_len, u32 *resp_len)
{
	struct cptra_mci_mbox_iov iov = {
		.base = req,
		.len = req_len,
	};

	return cptra_mci_mbox_execute_iov(cmd, &iov, 1, resp, resp_buf_len, resp_len);
}
EXPORT_SYMBOL_GPL(aspeed_cptra_mci_mbox_execute);

int aspeed_cptra_mci_mbox_execute_sg(u32 cmd, const void *hdr, u32 hdr_len,
				     const void *data, u32 data_len,
				     void *resp, u32 resp_buf_len, u32 *resp_len)
{
	struct cptra_mci_mbox_iov iov[2] = {
		{ .base = hdr, .len = hdr_len },
		{ .base = data, .len = data_len },
	};

	return cptra_mci_mbox_execute_iov(cmd, iov, 2, resp, resp_buf_len, resp_len);
}
EXPORT_SYMBOL_GPL(aspeed_cptra_mci_mbox_execute_sg);

/*
 * page/offset come straight from userspace on the ioctl path (see
 * cptra_mci_ioctl_reg_read()), and page is written directly into the SCU1
 * window-select register: an unchecked page lets a caller remap the window
 * onto any 64KB-aligned physical page this bus master can reach, not just
 * the Caliptra MCI blocks this driver knows about. Restrict it to the two
 * page targets aspeed-cptra-mci.h actually documents.
 */
static bool cptra_mci_reg_read_page_allowed(u32 page)
{
	return page == CPTRA_MCI_REG_PAGE || page == CPTRA_MCI_SOC_IFC_PAGE;
}

/*
 * Direct register read outside the mailbox command/lock/execute protocol --
 * see the comment on the CPTRA_MCI_REG_* / CPTRA_MCI_SOC_IFC_* definitions in
 * aspeed-cptra-mci.h. Not covered by the mailbox HW LOCK (that semaphore
 * only arbitrates the mcu_mbox0_csr block, a different page); cptra_mci_lock
 * is what keeps this from racing a concurrent mailbox transaction over the
 * shared page-select window.
 *
 * Caller must already hold cptra_mci_lock and have a valid priv -- see the
 * comment on cptra_mci_mbox_execute_iov_locked().
 */
static int cptra_mci_reg_read_locked(struct aspeed_cptra_mci *priv, u32 page, u32 offset,
				     u32 *value)
{
	int ret;

	if (!cptra_mci_reg_read_page_allowed(page) || offset + sizeof(*value) > priv->regs_size)
		return -EINVAL;

	ret = cptra_mci_mbox_select_page(priv, page);
	if (ret)
		return ret;

	*value = readl(priv->regs + offset);

	cptra_mci_mbox_select_page(priv, CPTRA_MCI_MBOX_CSR_PAGE);

	return 0;
}

int aspeed_cptra_mci_reg_read(u32 page, u32 offset, u32 *value)
{
	struct aspeed_cptra_mci *priv;
	int ret;

	priv = aspeed_cptra_mci_lock();
	if (!priv)
		return -EPROBE_DEFER;

	ret = cptra_mci_reg_read_locked(priv, page, offset, value);

	aspeed_cptra_mci_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(aspeed_cptra_mci_reg_read);

/*
 * /dev/aspeed-cptra-mci: userspace-facing ioctl passthrough. Every request/
 * response struct layout, checksum computation and command-specific
 * interpretation lives entirely in the userspace caller -- this driver only
 * forwards an opaque (cmd, req buffer) to aspeed_cptra_mci_mbox_execute()
 * and copies the raw response back, the same call any other in-kernel
 * client of this file would make.
 */
static long cptra_mci_ioctl_mbox_execute(struct aspeed_cptra_mci *priv, unsigned long arg)
{
	struct cptra_mci_ioc_mbox_execute uarg;
	struct cptra_mci_mbox_iov iov;
	void *req_buf, *resp_buf;
	u32 resp_len;
	int ret;

	if (copy_from_user(&uarg, (void __user *)arg, sizeof(uarg))) {
		dev_dbg(priv->dev, "ioctl: failed to copy request struct from user\n");
		return -EFAULT;
	}

	dev_dbg(priv->dev, "ioctl: cmd=0x%08x req_len=%u resp_buf_len=%u\n",
		uarg.cmd, uarg.req_len, uarg.resp_buf_len);

	if (uarg.req_len > CPTRA_MCI_MBOX_SRAM_SIZE ||
	    uarg.resp_buf_len > CPTRA_MCI_MBOX_SRAM_SIZE) {
		dev_dbg(priv->dev, "ioctl: req_len/resp_buf_len exceeds SRAM size 0x%x\n",
			CPTRA_MCI_MBOX_SRAM_SIZE);
		return -EINVAL;
	}

	req_buf = memdup_user(u64_to_user_ptr(uarg.req_ptr), uarg.req_len);
	if (IS_ERR(req_buf)) {
		dev_dbg(priv->dev, "ioctl: failed to copy request payload from user: %ld\n",
			PTR_ERR(req_buf));
		return PTR_ERR(req_buf);
	}

	resp_buf = kzalloc(uarg.resp_buf_len, GFP_KERNEL);
	if (!resp_buf) {
		kfree(req_buf);
		return -ENOMEM;
	}

	iov.base = req_buf;
	iov.len = uarg.req_len;

	/*
	 * Use priv (kept alive by this fd's own reference, see
	 * cptra_mci_chardev_open()) directly instead of the exported
	 * aspeed_cptra_mci_mbox_execute(): that helper does its own
	 * cptra_mci lookup, which starts failing with -EPROBE_DEFER the
	 * moment remove() clears the singleton -- even though this fd's
	 * priv/regs are still valid and safe to use until it is closed.
	 */
	mutex_lock(&cptra_mci_lock);
	ret = cptra_mci_mbox_execute_iov_locked(priv, uarg.cmd, &iov, 1, resp_buf,
						uarg.resp_buf_len, &resp_len);
	mutex_unlock(&cptra_mci_lock);
	kfree(req_buf);
	if (ret) {
		dev_dbg(priv->dev, "ioctl: cmd 0x%08x execute failed: %d\n", uarg.cmd, ret);
		goto out;
	}

	if (copy_to_user(u64_to_user_ptr(uarg.resp_ptr), resp_buf, resp_len)) {
		dev_dbg(priv->dev, "ioctl: failed to copy response payload to user\n");
		ret = -EFAULT;
		goto out;
	}

	uarg.resp_len = resp_len;
	if (copy_to_user((void __user *)arg, &uarg, sizeof(uarg))) {
		dev_dbg(priv->dev, "ioctl: failed to copy response struct to user\n");
		ret = -EFAULT;
		goto out;
	}

	dev_dbg(priv->dev, "ioctl: cmd 0x%08x completed, resp_len=%u\n", uarg.cmd, resp_len);

out:
	kfree(resp_buf);

	return ret;
}

static long cptra_mci_ioctl_reg_read(struct aspeed_cptra_mci *priv, unsigned long arg)
{
	struct cptra_mci_ioc_reg_read uarg;
	int ret;

	if (copy_from_user(&uarg, (void __user *)arg, sizeof(uarg))) {
		dev_dbg(priv->dev, "ioctl: failed to copy reg_read request from user\n");
		return -EFAULT;
	}

	/* See the comment in cptra_mci_ioctl_mbox_execute() on using priv directly. */
	mutex_lock(&cptra_mci_lock);
	ret = cptra_mci_reg_read_locked(priv, uarg.page, uarg.offset, &uarg.value);
	mutex_unlock(&cptra_mci_lock);
	if (ret) {
		dev_dbg(priv->dev, "ioctl: reg_read page=0x%x offset=0x%x failed: %d\n",
			uarg.page, uarg.offset, ret);
		return ret;
	}

	if (copy_to_user((void __user *)arg, &uarg, sizeof(uarg))) {
		dev_dbg(priv->dev, "ioctl: failed to copy reg_read response to user\n");
		return -EFAULT;
	}

	return 0;
}

static long cptra_mci_chardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct aspeed_cptra_mci *priv = file->private_data;

	switch (cmd) {
	case CPTRA_MCI_IOC_MBOX_EXECUTE:
		return cptra_mci_ioctl_mbox_execute(priv, arg);
	case CPTRA_MCI_IOC_REG_READ:
		return cptra_mci_ioctl_reg_read(priv, arg);
	default:
		return -ENOTTY;
	}
}

/*
 * Takes its own reference on priv (kref, not cptra_mci_lock) so an fd
 * opened before remove() starts keeps working -- and keeps priv/regs
 * alive -- for as long as it stays open, regardless of when cptra_mci
 * itself gets nulled out. See the struct aspeed_cptra_mci comment.
 */
static int cptra_mci_chardev_open(struct inode *inode, struct file *file)
{
	struct aspeed_cptra_mci *priv;

	priv = aspeed_cptra_mci_lock();
	if (!priv)
		return -ENODEV;

	kref_get(&priv->refcount);
	aspeed_cptra_mci_unlock();

	file->private_data = priv;

	return 0;
}

static int cptra_mci_chardev_release(struct inode *inode, struct file *file)
{
	struct aspeed_cptra_mci *priv = file->private_data;

	kref_put(&priv->refcount, aspeed_cptra_mci_release);

	return 0;
}

static const struct file_operations cptra_mci_chardev_fops = {
	.owner		= THIS_MODULE,
	.open		= cptra_mci_chardev_open,
	.release	= cptra_mci_chardev_release,
	.unlocked_ioctl	= cptra_mci_chardev_ioctl,
};

static int aspeed_cptra_mci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_cptra_mci *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);
	priv->regs_size = resource_size(res);

	priv->scu1 = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,scu1");
	if (IS_ERR(priv->scu1)) {
		dev_err(dev, "cannot get SCU1 regmap\n");
		return PTR_ERR(priv->scu1);
	}

	priv->dev = dev;
	kref_init(&priv->refcount);
	init_completion(&priv->released);
	platform_set_drvdata(pdev, priv);

	mutex_lock(&cptra_mci_lock);
	cptra_mci = priv;
	mutex_unlock(&cptra_mci_lock);

	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = "aspeed-cptra-mci";
	priv->miscdev.fops = &cptra_mci_chardev_fops;
	priv->miscdev.parent = dev;

	ret = misc_register(&priv->miscdev);
	if (ret) {
		mutex_lock(&cptra_mci_lock);
		cptra_mci = NULL;
		mutex_unlock(&cptra_mci_lock);
		return dev_err_probe(dev, ret, "failed to register /dev/%s\n", priv->miscdev.name);
	}

	dev_info(dev, "registered /dev/%s\n", priv->miscdev.name);

	return 0;
}

static void aspeed_cptra_mci_remove(struct platform_device *pdev)
{
	struct aspeed_cptra_mci *priv = platform_get_drvdata(pdev);

	/* Block new lookups (exported API) and new opens (/dev fd). */
	mutex_lock(&cptra_mci_lock);
	cptra_mci = NULL;
	mutex_unlock(&cptra_mci_lock);
	misc_deregister(&priv->miscdev);

	/*
	 * Drop the probe reference and wait for it to actually hit zero:
	 * any fd opened before the misc_deregister() above holds its own
	 * reference via file->private_data and keeps priv (and priv->regs)
	 * alive -- and usable -- until it is closed, so this blocks until
	 * every such fd is closed. Only then is it safe to return and let
	 * devm free priv/unmap regs.
	 */
	kref_put(&priv->refcount, aspeed_cptra_mci_release);
	wait_for_completion(&priv->released);
}

static const struct of_device_id aspeed_cptra_mci_of_matches[] = {
	{ .compatible = "aspeed,ast2705-cptra-mci" },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_cptra_mci_of_matches);

static struct platform_driver aspeed_cptra_mci_driver = {
	.probe	= aspeed_cptra_mci_probe,
	.remove	= aspeed_cptra_mci_remove,
	.driver	= {
		.name		= "aspeed-cptra-mci",
		.of_match_table	= aspeed_cptra_mci_of_matches,
	},
};
module_platform_driver(aspeed_cptra_mci_driver);

MODULE_AUTHOR("Jheng Shuo Lin <jheng-shuo_lin@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Caliptra-SS MCI driver");
MODULE_LICENSE("GPL");
