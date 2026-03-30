// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Aspeed Technology Inc.
 */
#include "linux/err.h"
#include "linux/fs.h"
#include "linux/printk.h"
#include "linux/spinlock.h"
#include "linux/stddef.h"
#include "linux/wait.h"
#include "linux/kernel.h"
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/count_zeros.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include "ast2700-espi.h"
#include "aspeed-espi-comm.h"
#include "aspeed-espi.h"

static DEFINE_IDA(ast2700_espi_ida);

#define PERIF_MCYC_ALIGN	SZ_64K
#define PERIF_MMBI_ALIGN	SZ_64M

#define OOB_DMA_RPTR_KEY	0x4f4f4253
#define OOB_DMA_DESC_NUM	8
#define OOB_DMA_DESC_CUSTOM	0x4

#define FLASH_EDAF_ALIGN	SZ_16M

/* peripheral channel (CH0) */
static int ast2700_espi_mmbi_b2h_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct aspeed_espi_perif_mmbi *mmbi;
	struct aspeed_espi_perif *perif;
	struct aspeed_espi *espi;
	unsigned long vm_size;
	pgprot_t prot;

	mmbi = container_of(fp->private_data, struct aspeed_espi_perif_mmbi, b2h_mdev);

	perif = mmbi->perif;

	espi = container_of(perif, struct aspeed_espi, perif);
	vm_size = vma->vm_end - vma->vm_start;
	prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vm_size) > (perif->mmbi.inst_size >> 1))
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (mmbi->b2h_addr >> PAGE_SHIFT) + vma->vm_pgoff,
			    vm_size, prot))
		return -EAGAIN;

	return 0;
}

static int ast2700_espi_mmbi_h2b_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct aspeed_espi_perif_mmbi *mmbi;
	struct aspeed_espi_perif *perif;
	struct aspeed_espi *espi;
	unsigned long vm_size;
	pgprot_t prot;

	mmbi = container_of(fp->private_data, struct aspeed_espi_perif_mmbi, h2b_mdev);

	perif = mmbi->perif;

	espi = container_of(perif, struct aspeed_espi, perif);

	vm_size = vma->vm_end - vma->vm_start;
	prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vm_size) > (perif->mmbi.inst_size >> 1))
		return -EINVAL;

	prot = pgprot_noncached(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (mmbi->h2b_addr >> PAGE_SHIFT) + vma->vm_pgoff,
			    vm_size, prot))
		return -EAGAIN;

	return 0;
}

static __poll_t ast2700_espi_mmbi_h2b_poll(struct file *fp, struct poll_table_struct *pt)
{
	struct aspeed_espi_perif_mmbi *mmbi;

	mmbi = container_of(fp->private_data, struct aspeed_espi_perif_mmbi, h2b_mdev);

	poll_wait(fp, &mmbi->wq, pt);

	if (!mmbi->host_rwp_update)
		return 0;

	mmbi->host_rwp_update = false;

	return EPOLLIN;
}

static long ast2700_espi_perif_pc_get_rx(struct file *fp,
					 struct aspeed_espi_perif *perif,
					 struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	unsigned long flags;
	u32 pkt_len;
	u8 *pkt;
	int i, rc;

	espi = container_of(perif, struct aspeed_espi, perif);

	if (fp->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&perif->pc_rx_mtx))
			return -EAGAIN;

		if (!perif->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	} else {
		mutex_lock(&perif->pc_rx_mtx);

		if (!perif->rx_ready) {
			rc = wait_event_interruptible(perif->wq, perif->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	reg = readl(espi->regs + ESPI_CH0_PC_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH0_PC_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH0_PC_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH0_PC_RX_CTRL_LEN, reg);

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_PERIF_MSG:
		pkt_len = sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_MSG_D:
		pkt_len = ((len) ? len : ESPI_MAX_PLD_LEN) +
			  sizeof(struct espi_perif_msg);
		break;
	case ESPI_PERIF_SUC_CMPLT_D_MIDDLE:
	case ESPI_PERIF_SUC_CMPLT_D_FIRST:
	case ESPI_PERIF_SUC_CMPLT_D_LAST:
	case ESPI_PERIF_SUC_CMPLT_D_ONLY:
		pkt_len = ((len) ? len : ESPI_MAX_PLD_LEN) +
			  sizeof(struct espi_perif_cmplt);
		break;
	case ESPI_PERIF_SUC_CMPLT:
	case ESPI_PERIF_UNSUC_CMPLT:
		pkt_len = sizeof(struct espi_perif_cmplt);
		break;
	default:
		rc = -EFAULT;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len < pkt_len) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (perif->dma.enable) {
		memcpy(hdr + 1, perif->dma.pc_rx_virt, pkt_len - sizeof(*hdr));
	} else {
		for (i = sizeof(*hdr); i < pkt_len; ++i)
			reg = readl(espi->regs + ESPI_CH0_PC_RX_DATA) & 0xff;
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&perif->lock, flags);

	writel(ESPI_CH0_PC_RX_CTRL_SERV_PEND, espi->regs + ESPI_CH0_PC_RX_CTRL);
	perif->rx_ready = 0;

	spin_unlock_irqrestore(&perif->lock, flags);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&perif->pc_rx_mtx);

	return rc;
}

static long ast2700_espi_perif_pc_put_tx(struct file *fp,
					 struct aspeed_espi_perif *perif,
					 struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u8 *pkt;
	int i, rc;

	espi = container_of(perif, struct aspeed_espi, perif);

	if (!mutex_trylock(&perif->pc_tx_mtx))
		return -EAGAIN;

	reg = readl(espi->regs + ESPI_CH0_PC_TX_CTRL);
	if (reg & ESPI_CH0_PC_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (perif->dma.enable) {
		memcpy(perif->dma.pc_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	} else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			writel(pkt[i], espi->regs + ESPI_CH0_PC_TX_DATA);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH0_PC_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH0_PC_TX_CTRL_LEN, len)
	      | ESPI_CH0_PC_TX_CTRL_TRIG_PEND;
	writel(reg, espi->regs + ESPI_CH0_PC_TX_CTRL);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_n_out:
	mutex_unlock(&perif->pc_tx_mtx);

	return rc;
}

static long ast2700_espi_perif_np_put_tx(struct file *fp,
					 struct aspeed_espi_perif *perif,
					 struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u8 *pkt;
	int i, rc;

	espi = container_of(perif, struct aspeed_espi, perif);

	if (!mutex_trylock(&perif->np_tx_mtx))
		return -EAGAIN;

	reg = readl(espi->regs + ESPI_CH0_NP_TX_CTRL);
	if (reg & ESPI_CH0_NP_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (perif->dma.enable) {
		memcpy(perif->dma.np_tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	} else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			writel(pkt[i], espi->regs + ESPI_CH0_NP_TX_DATA);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH0_NP_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH0_NP_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH0_NP_TX_CTRL_LEN, len)
	      | ESPI_CH0_NP_TX_CTRL_TRIG_PEND;
	writel(reg, espi->regs + ESPI_CH0_NP_TX_CTRL);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_n_out:
	mutex_unlock(&perif->np_tx_mtx);

	return rc;
}

static long ast2700_espi_perif_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_perif *perif;
	struct aspeed_espi_ioc ioc;

	perif = container_of(fp->private_data, struct aspeed_espi_perif, mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
		return -EFAULT;

	if (ioc.pkt_len > ESPI_MAX_PKT_LEN)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_PERIF_PC_GET_RX:
		return ast2700_espi_perif_pc_get_rx(fp, perif, &ioc);
	case ASPEED_ESPI_PERIF_PC_PUT_TX:
		return ast2700_espi_perif_pc_put_tx(fp, perif, &ioc);
	case ASPEED_ESPI_PERIF_NP_PUT_TX:
		return ast2700_espi_perif_np_put_tx(fp, perif, &ioc);
	default:
		break;
	};

	return -EINVAL;
}

static int ast2700_espi_perif_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct aspeed_espi_perif *perif;
	unsigned long vm_size;
	pgprot_t vm_prot;

	perif = container_of(fp->private_data, struct aspeed_espi_perif, mdev);
	if (!perif->mcyc.enable)
		return -EPERM;

	vm_size = vma->vm_end - vma->vm_start;
	vm_prot = vma->vm_page_prot;

	if (((vma->vm_pgoff << PAGE_SHIFT) + vm_size) > perif->mcyc.size)
		return -EINVAL;

	vm_prot = pgprot_noncached(vm_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (perif->mcyc.taddr >> PAGE_SHIFT) + vma->vm_pgoff,
			    vm_size, vm_prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations ast2700_espi_mmbi_b2h_fops = {
	.owner = THIS_MODULE,
	.mmap = ast2700_espi_mmbi_b2h_mmap,
};

static const struct file_operations ast2700_espi_mmbi_h2b_fops = {
	.owner = THIS_MODULE,
	.mmap = ast2700_espi_mmbi_h2b_mmap,
	.poll = ast2700_espi_mmbi_h2b_poll,
};

static const struct file_operations ast2700_espi_perif_fops = {
	.owner = THIS_MODULE,
	.mmap = ast2700_espi_perif_mmap,
	.unlocked_ioctl = ast2700_espi_perif_ioctl,
};

static irqreturn_t ast2700_espi_perif_mmbi_isr(int irq, void *arg)
{
	struct aspeed_espi_perif_mmbi *mmbi;
	struct aspeed_espi_perif *perif;
	struct aspeed_espi *espi;
	u32 sts, tmp;
	u32 *p;
	int i;

	espi = (struct aspeed_espi *)arg;

	perif = &espi->perif;

	sts = readl(espi->regs + ESPI_MMBI_INT_STS);
	if (!sts)
		return IRQ_NONE;

	for (i = 0, tmp = sts; i < perif->mmbi.inst_num; ++i, tmp >>= 2) {
		if (!(tmp & 0x3))
			continue;

		mmbi = &perif->mmbi.inst[i];

		p = (u32 *)mmbi->h2b_virt;
		p[0] = readl(espi->regs + ESPI_MMBI_HOST_RWP(i));
		p[1] = readl(espi->regs + ESPI_MMBI_HOST_RWP(i) + 4);

		mmbi->host_rwp_update = true;

		wake_up_interruptible(&mmbi->wq);
	}

	writel(sts, espi->regs + ESPI_MMBI_INT_STS);

	return IRQ_HANDLED;
}

static void ast2700_espi_perif_isr(struct aspeed_espi *espi)
{
	struct aspeed_espi_perif *perif;
	unsigned long flags;
	u32 sts;

	perif = &espi->perif;

	sts = readl(espi->regs + ESPI_CH0_INT_STS);

	if (sts & ESPI_CH0_INT_STS_PC_RX_CMPLT) {
		writel(ESPI_CH0_INT_STS_PC_RX_CMPLT, espi->regs + ESPI_CH0_INT_STS);

		spin_lock_irqsave(&perif->lock, flags);
		perif->rx_ready = true;
		spin_unlock_irqrestore(&perif->lock, flags);

		wake_up_interruptible(&perif->wq);
	}
}

static void ast2700_espi_perif_sw_reset(struct aspeed_espi *espi)
{
	struct device *dev;
	u32 reg;

	dev = espi->dev;

	reg = readl(espi->regs + ESPI_CH0_CTRL);
	reg &= ~(ESPI_CH0_CTRL_NP_TX_RST
		 | ESPI_CH0_CTRL_NP_RX_RST
		 | ESPI_CH0_CTRL_PC_TX_RST
		 | ESPI_CH0_CTRL_PC_RX_RST
		 | ESPI_CH0_CTRL_NP_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_RX_DMA_EN
		 | ESPI_CH0_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH0_CTRL);

	udelay(1);

	reg |= (ESPI_CH0_CTRL_NP_TX_RST
		| ESPI_CH0_CTRL_NP_RX_RST
		| ESPI_CH0_CTRL_PC_TX_RST
		| ESPI_CH0_CTRL_PC_RX_RST);
	writel(reg, espi->regs + ESPI_CH0_CTRL);
}

static void ast2700_espi_perif_reset(struct aspeed_espi *espi)
{
	struct aspeed_espi_perif *perif;
	struct device *dev;
	u64 mask;
	u32 reg;

	dev = espi->dev;

	perif = &espi->perif;

	writel(0x0, espi->regs + ESPI_CH0_INT_EN);
	writel(0xffffffff, espi->regs + ESPI_CH0_INT_STS);

	writel(0x0, espi->regs + ESPI_MMBI_INT_EN);
	writel(0xffffffff, espi->regs + ESPI_MMBI_INT_STS);

	reg = readl(espi->regs + ESPI_CH0_CTRL);
	reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS_WDT | ESPI_CH0_CTRL_MCYC_WR_DIS_WDT);
	writel(reg, espi->regs + ESPI_CH0_CTRL);

	reg = readl(espi->regs + ESPI_CH0_MCYC0_MASKL);
	reg &= ~ESPI_CH0_MCYC0_MASKL_EN;
	writel(reg, espi->regs + ESPI_CH0_MCYC0_MASKL);

	reg = readl(espi->regs + ESPI_CH0_MCYC1_MASKL);
	reg &= ~ESPI_CH0_MCYC1_MASKL_EN;
	writel(reg, espi->regs + ESPI_CH0_MCYC1_MASKL);

	reg = readl(espi->regs + ESPI_CH0_CTRL);
	reg |= (ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
	reg &= ~(ESPI_CH0_CTRL_NP_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_RX_DMA_EN
		 | ESPI_CH0_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH0_CTRL);

	if (perif->mmbi.enable) {
		reg = readl(espi->regs + ESPI_MMBI_CTRL);
		reg &= ~ESPI_MMBI_CTRL_EN;
		writel(reg, espi->regs + ESPI_MMBI_CTRL);

		mask = ~(perif->mmbi.size - 1);
		writel(upper_32_bits(mask), espi->regs + ESPI_CH0_MCYC0_MASKH);
		writel(mask & 0xffffffff, espi->regs + ESPI_CH0_MCYC0_MASKL);
		writel(upper_32_bits(perif->mmbi.saddr), espi->regs + ESPI_CH0_MCYC0_SADDRH);
		writel((perif->mmbi.saddr & 0xffffffff), espi->regs + ESPI_CH0_MCYC0_SADDRL);
		writel(upper_32_bits(perif->mmbi.taddr), espi->regs + ESPI_CH0_MCYC0_TADDRH);
		writel((perif->mmbi.taddr & 0xffffffff), espi->regs + ESPI_CH0_MCYC0_TADDRL);

		writel((0x1 << (perif->mmbi.inst_num * 2)) - 1, espi->regs + ESPI_MMBI_INT_EN);

		reg = FIELD_PREP(ESPI_MMBI_CTRL_INST_NUM, count_trailing_zeros(perif->mmbi.inst_num))
		    | ESPI_MMBI_CTRL_EN;
		writel(reg, espi->regs + ESPI_MMBI_CTRL);

		reg = readl(espi->regs + ESPI_CH0_MCYC0_MASKL) | ESPI_CH0_MCYC0_MASKL_EN;
		writel(reg, espi->regs + ESPI_CH0_MCYC0_MASKL);

		reg = readl(espi->regs + ESPI_CH0_CTRL);
		reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
		writel(reg, espi->regs + ESPI_CH0_CTRL);
	}

	if (perif->mcyc.enable) {
		mask = ~(perif->mcyc.size - 1);
		writel(upper_32_bits(mask), espi->regs + ESPI_CH0_MCYC1_MASKH);
		writel(mask & 0xffffffff, espi->regs + ESPI_CH0_MCYC1_MASKL);
		writel(upper_32_bits(perif->mcyc.saddr), espi->regs + ESPI_CH0_MCYC1_SADDRH);
		writel((perif->mcyc.saddr & 0xffffffff), espi->regs + ESPI_CH0_MCYC1_SADDRL);
		writel(upper_32_bits(perif->mcyc.taddr), espi->regs + ESPI_CH0_MCYC1_TADDRH);
		writel((perif->mcyc.taddr & 0xffffffff), espi->regs + ESPI_CH0_MCYC1_TADDRL);

		reg = readl(espi->regs + ESPI_CH0_MCYC1_MASKL) | ESPI_CH0_MCYC1_MASKL_EN;
		writel(reg, espi->regs + ESPI_CH0_MCYC1_MASKL);

		reg = readl(espi->regs + ESPI_CH0_CTRL);
		reg &= ~(ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
		writel(reg, espi->regs + ESPI_CH0_CTRL);
	}

	if (perif->dma.enable) {
		writel(upper_32_bits(perif->dma.np_tx_addr), espi->regs + ESPI_CH0_NP_TX_DMAH);
		writel((perif->dma.np_tx_addr & 0xffffffff), espi->regs + ESPI_CH0_NP_TX_DMAL);
		writel(upper_32_bits(perif->dma.pc_tx_addr), espi->regs + ESPI_CH0_PC_TX_DMAH);
		writel((perif->dma.pc_tx_addr & 0xffffffff), espi->regs + ESPI_CH0_PC_TX_DMAL);
		writel(upper_32_bits(perif->dma.pc_rx_addr), espi->regs + ESPI_CH0_PC_RX_DMAH);
		writel((perif->dma.pc_rx_addr & 0xffffffff), espi->regs + ESPI_CH0_PC_RX_DMAL);

		reg = readl(espi->regs + ESPI_CH0_CTRL)
		      | ESPI_CH0_CTRL_NP_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_TX_DMA_EN
		      | ESPI_CH0_CTRL_PC_RX_DMA_EN;
		writel(reg, espi->regs + ESPI_CH0_CTRL);
	}
	if (perif->rtc_enable) {
		reg = readl(espi->regs + ESPI_CAP_GEN)
		      | ESPI_CAP_GEN_RTC_SUP;
		writel(reg, espi->regs + ESPI_CAP_GEN);
	}

	writel(ESPI_CH0_INT_EN_PC_RX_CMPLT, espi->regs + ESPI_CH0_INT_EN);

	reg = readl(espi->regs + ESPI_CH0_CTRL) | ESPI_CH0_CTRL_SW_RDY;
	writel(reg, espi->regs + ESPI_CH0_CTRL);
}

int ast2700_espi_perif_probe(struct aspeed_espi *espi)
{
	struct aspeed_espi_perif_mmbi *mmbi;
	struct aspeed_espi_perif *perif;
	struct platform_device *pdev;
	struct device_node *np;
	struct resource res;
	struct device *dev;
	int i, rc;
	u64 temp;

	dev = espi->dev;

	perif = &espi->perif;

	init_waitqueue_head(&perif->wq);

	spin_lock_init(&perif->lock);

	mutex_init(&perif->np_tx_mtx);
	mutex_init(&perif->pc_tx_mtx);
	mutex_init(&perif->pc_rx_mtx);

	perif->mmbi.enable = of_property_read_bool(dev->of_node, "perif-mmbi-enable");
	if (perif->mmbi.enable) {
		pdev = container_of(dev, struct platform_device, dev);

		perif->mmbi.irq = platform_get_irq(pdev, 1);
		if (perif->mmbi.irq < 0) {
			dev_err(dev, "cannot get MMBI IRQ number\n");
			return -ENODEV;
		}

		rc = of_property_read_u64(dev->of_node, "perif-mmbi-src-addr", &temp);
		if (rc || !IS_ALIGNED(temp, PERIF_MMBI_ALIGN)) {
			dev_err(dev, "cannot get 64MB-aligned MMBI host address\n");
			return -ENODEV;
		}
		perif->mmbi.saddr = temp;
		rc = of_property_read_u32(dev->of_node, "perif-mmbi-instance-num", &perif->mmbi.inst_num);
		if (rc ||
		    perif->mmbi.inst_num == 0 ||
		    perif->mmbi.inst_num > PERIF_MMBI_INST_NUM ||
		    (perif->mmbi.inst_num & (perif->mmbi.inst_num - 1))) {
			dev_err(dev, "cannot get valid MMBI instance number, expect 1/2/4/8\n");
			return -EINVAL;
		}

		np = of_parse_phandle(dev->of_node, "perif-mmbi-tgt-memory", 0);
		if (!np || of_address_to_resource(np, 0, &res)) {
			dev_err(dev, "cannot get MMBI memory region\n");
			return -ENODEV;
		}

		of_node_put(np);

		perif->mmbi.taddr = res.start;
		perif->mmbi.size = resource_size(&res);
		perif->mmbi.inst_size = perif->mmbi.size / perif->mmbi.inst_num;
		if (!IS_ALIGNED(perif->mmbi.taddr, PERIF_MMBI_ALIGN) ||
		    !IS_ALIGNED(perif->mmbi.size, PERIF_MMBI_ALIGN)) {
			dev_err(dev, "cannot get 64MB-aligned MMBI address/size\n");
			return -EINVAL;
		}

		perif->mmbi.virt = devm_ioremap_resource(dev, &res);
		if (!perif->mmbi.virt) {
			dev_err(dev, "cannot map MMBI memory region\n");
			return -ENOMEM;
		}

		memset_io(perif->mmbi.virt, 0, perif->mmbi.size);
		perif->mmbi.mmbi_isr = ast2700_espi_perif_mmbi_isr;

		for (i = 0; i < perif->mmbi.inst_num; ++i) {
			mmbi = &perif->mmbi.inst[i];

			init_waitqueue_head(&mmbi->wq);

			mmbi->perif = perif;
			mmbi->host_rwp_update = false;

			mmbi->b2h_virt = perif->mmbi.virt + ((perif->mmbi.inst_size >> 1) * i);
			mmbi->b2h_addr = perif->mmbi.taddr + ((perif->mmbi.inst_size >> 1) * i);
			mmbi->b2h_mdev.parent = dev;
			mmbi->b2h_mdev.minor = MISC_DYNAMIC_MINOR;
			mmbi->b2h_mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-mmbi%d-b2h%d",
							     DEVICE_NAME, espi->dev_id, i);
			mmbi->b2h_mdev.fops = &ast2700_espi_mmbi_b2h_fops;
			rc = misc_register(&mmbi->b2h_mdev);
			if (rc) {
				dev_err(dev, "cannot register device %s\n", mmbi->b2h_mdev.name);
				return rc;
			}

			mmbi->h2b_virt = perif->mmbi.virt + ((perif->mmbi.inst_size >> 1) * (i + perif->mmbi.inst_num));
			mmbi->h2b_addr = perif->mmbi.taddr + ((perif->mmbi.inst_size >> 1) * (i + perif->mmbi.inst_num));
			mmbi->h2b_mdev.parent = dev;
			mmbi->h2b_mdev.minor = MISC_DYNAMIC_MINOR;
			mmbi->h2b_mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-mmbi%d-h2b%d",
							     DEVICE_NAME, espi->dev_id, i);
			mmbi->h2b_mdev.fops = &ast2700_espi_mmbi_h2b_fops;
			rc = misc_register(&mmbi->h2b_mdev);
			if (rc) {
				dev_err(dev, "cannot register device %s\n", mmbi->h2b_mdev.name);
				return rc;
			}
		}
	}

	perif->mcyc.enable = of_property_read_bool(dev->of_node, "perif-mcyc-enable");
	if (perif->mcyc.enable) {
		rc = of_property_read_u64(dev->of_node, "perif-mcyc-src-addr", &temp);
		if (rc || !IS_ALIGNED(temp, PERIF_MCYC_ALIGN)) {
			dev_err(dev, "cannot get 64KB-aligned memory cycle host address\n");
			return -ENODEV;
		}
		perif->mcyc.saddr = temp;

		rc = of_property_read_u64(dev->of_node, "perif-mcyc-size", &temp);
		if (rc || !IS_ALIGNED(temp, PERIF_MCYC_ALIGN)) {
			dev_err(dev, "cannot get 64KB-aligned memory cycle size\n");
			return -EINVAL;
		}
		perif->mcyc.size = temp;

		np = of_parse_phandle(dev->of_node, "memory-region", 0);
		if (np) {
			of_reserved_mem_device_init(dev);
			rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
			if (rc) {
				dev_err(dev, "Failed to mask DMA.\n");
				return -ENODEV;
			}
		}

		perif->mcyc.virt = dmam_alloc_coherent(dev, perif->mcyc.size,
						       &perif->mcyc.taddr, GFP_KERNEL);
		if (!perif->mcyc.virt) {
			dev_err(dev, "cannot allocate memory cycle\n");
			return -ENOMEM;
		}
	}

	perif->dma.enable = of_property_read_bool(dev->of_node, "perif-dma-mode");
	if (perif->dma.enable) {
		perif->dma.pc_tx_virt = dmam_alloc_coherent(dev, PAGE_SIZE,
							    &perif->dma.pc_tx_addr, GFP_KERNEL);
		if (!perif->dma.pc_tx_virt) {
			dev_err(dev, "cannot allocate posted TX DMA buffer\n");
			return -ENOMEM;
		}

		perif->dma.pc_rx_virt = dmam_alloc_coherent(dev, PAGE_SIZE,
							    &perif->dma.pc_rx_addr, GFP_KERNEL);
		if (!perif->dma.pc_rx_virt) {
			dev_err(dev, "cannot allocate posted RX DMA buffer\n");
			return -ENOMEM;
		}

		perif->dma.np_tx_virt = dmam_alloc_coherent(dev, PAGE_SIZE,
							    &perif->dma.np_tx_addr, GFP_KERNEL);
		if (!perif->dma.np_tx_virt) {
			dev_err(dev, "cannot allocate non-posted TX DMA buffer\n");
			return -ENOMEM;
		}
	}
	perif->rtc_enable = of_property_read_bool(dev->of_node, "perif-rtc-enable");

	perif->mdev.parent = dev;
	perif->mdev.minor = MISC_DYNAMIC_MINOR;
	perif->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-peripheral%d", DEVICE_NAME, espi->dev_id);
	perif->mdev.fops = &ast2700_espi_perif_fops;
	rc = misc_register(&perif->mdev);
	if (rc) {
		dev_err(dev, "cannot register device %s\n", perif->mdev.name);
		return rc;
	}

	ast2700_espi_perif_reset(espi);

	return 0;
}

int ast2700_espi_perif_remove(struct aspeed_espi *espi)
{
	struct aspeed_espi_perif_mmbi *mmbi;
	struct aspeed_espi_perif *perif;
	struct device *dev;
	u32 reg;
	int i;

	dev = espi->dev;

	perif = &espi->perif;

	writel(0x0, espi->regs + ESPI_CH0_INT_EN);
	writel(0x0, espi->regs + ESPI_MMBI_INT_EN);

	reg = readl(espi->regs + ESPI_CH0_MCYC0_MASKL);
	reg &= ~ESPI_CH0_MCYC0_MASKL_EN;
	writel(reg, espi->regs + ESPI_CH0_MCYC0_MASKL);

	reg = readl(espi->regs + ESPI_CH0_MCYC1_MASKL);
	reg &= ~ESPI_CH0_MCYC1_MASKL_EN;
	writel(reg, espi->regs + ESPI_CH0_MCYC1_MASKL);

	reg = readl(espi->regs + ESPI_CH0_CTRL);
	reg |= (ESPI_CH0_CTRL_MCYC_RD_DIS | ESPI_CH0_CTRL_MCYC_WR_DIS);
	reg &= ~(ESPI_CH0_CTRL_NP_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_TX_DMA_EN
		 | ESPI_CH0_CTRL_PC_RX_DMA_EN
		 | ESPI_CH0_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH0_CTRL);

	if (perif->mmbi.enable) {
		reg = readl(espi->regs + ESPI_MMBI_CTRL);
		reg &= ~ESPI_MMBI_CTRL_EN;
		writel(reg, espi->regs + ESPI_MMBI_CTRL);

		for (i = 0; i < perif->mmbi.inst_num; ++i) {
			mmbi = &perif->mmbi.inst[i];
			misc_deregister(&mmbi->b2h_mdev);
			misc_deregister(&mmbi->h2b_mdev);
		}

		devm_iounmap(dev, perif->mmbi.virt);
	}

	if (perif->mcyc.enable)
		dmam_free_coherent(dev, perif->mcyc.size, perif->mcyc.virt,
				   perif->mcyc.taddr);

	if (perif->dma.enable) {
		dmam_free_coherent(dev, PAGE_SIZE, perif->dma.np_tx_virt,
				   perif->dma.np_tx_addr);
		dmam_free_coherent(dev, PAGE_SIZE, perif->dma.pc_tx_virt,
				   perif->dma.pc_tx_addr);
		dmam_free_coherent(dev, PAGE_SIZE, perif->dma.pc_rx_virt,
				   perif->dma.pc_rx_addr);
	}

	mutex_destroy(&perif->np_tx_mtx);
	mutex_destroy(&perif->pc_tx_mtx);
	mutex_destroy(&perif->pc_rx_mtx);

	misc_deregister(&perif->mdev);

	return 0;
}

/* virtual wire channel (CH1) */
static long ast2700_espi_vw_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_vw *vw;
	struct aspeed_espi *espi;
	u32 gpio0, gpio1;
	u8 pch_generic;
	u32 reg;
	u32 hw_mode;

	vw = container_of(fp->private_data, struct aspeed_espi_vw, mdev);
	espi = container_of(vw, struct aspeed_espi, vw);
	gpio0 = vw->gpio.val0;
	gpio1 = vw->gpio.val1;
	hw_mode = vw->gpio.hw_mode;
	pch_generic = vw->platform.pch_generic;

	if (hw_mode) {
		dev_err(espi->dev, "HW mode: vGPIO reflect on physical GPIO. Get state from GPIO driver.\n");
		return -EFAULT;
	}

	switch (cmd) {
	case ASPEED_ESPI_VW_GET_GPIO_VAL:
		if (put_user(gpio0, (u32 __user *)arg)) {
			dev_err(espi->dev, "failed to get vGPIO value0\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Get vGPIO value0: 0x%x\n", gpio0);
		break;

	case ASPEED_ESPI_VW_PUT_GPIO_VAL:
		if (get_user(gpio0, (u32 __user *)arg)) {
			dev_err(espi->dev, "failed to put vGPIO value0\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Put vGPIO value0: 0x%x\n", gpio0);
		writel(gpio0, espi->regs + ESPI_CH1_GPIO_VAL0);
		break;
#ifdef CONFIG_ARM64
	case ASPEED_ESPI_VW_GET_GPIO_VAL1:
		if (put_user(gpio1, (u32 __user *)arg)) {
			dev_err(espi->dev, "failed to get vGPIO value1\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Get vGPIO value1: 0x%x\n", gpio1);
		break;

	case ASPEED_ESPI_VW_PUT_GPIO_VAL1:
		if (get_user(gpio1, (u32 __user *)arg)) {
			dev_err(espi->dev, "failed to put vGPIO value1\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Put vGPIO value1: 0x%x\n", gpio1);
		writel(gpio1, espi->regs + ESPI_CH1_GPIO_VAL1);
		break;
#endif
	case ASPEED_ESPI_VW_GET_PCH_GENERIC:
		if (put_user(pch_generic, (u8 __user *)arg)) {
			dev_err(espi->dev, "failed to get PCH generic event status\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Get PCH generic event status: 0x%x\n", pch_generic);
		break;
	case ASPEED_ESPI_VW_PUT_PCH_GENERIC:
		if (get_user(pch_generic, (u8 __user *)arg)) {
			dev_err(espi->dev, "failed to put PCH generic event status\n");
			return -EFAULT;
		}

		dev_info(espi->dev, "Put PCH generic event status: 0x%x\n", pch_generic);
		reg = readl(espi->regs + ESPI_CH1_EVT1);
		reg &= ~ESPI_CH1_EVT1_BMC_GENE;
		reg |= FIELD_PREP(ESPI_CH1_EVT1_BMC_GENE, pch_generic);
		writel(reg, espi->regs + ESPI_CH1_EVT1);
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static const struct file_operations ast2700_espi_vw_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ast2700_espi_vw_ioctl,
};

static inline struct aspeed_espi_vw *to_ast2700_espi_pltrst(struct file *filp)
{
	return container_of(filp->private_data, struct aspeed_espi_vw,
			    pltrst_mdev);
}

static int ast2700_espi_vw_pltrst_open(struct inode *inode, struct file *filp)
{
	struct aspeed_espi_vw *priv = to_ast2700_espi_pltrst(filp);

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
		return -EACCES;
	priv->pltrst.pltrst_avail = true ; /*Setting true returns first data after file open*/

	return 0;
}

static ssize_t ast2700_espi_vw_pltrst_read(struct file *filp, char __user *buf,
					   size_t count, loff_t *offset)
{
	struct aspeed_espi_vw *vw = to_ast2700_espi_pltrst(filp);
	DECLARE_WAITQUEUE(wait, current);
	char data, old_sample;
	int ret = 0;

	spin_lock_irq(&vw->pltrst.pltrst_lock);

	if (filp->f_flags & O_NONBLOCK) {
		if (!vw->pltrst.pltrst_avail) {
			ret = -EAGAIN;
			goto out_unlock;
		}
		data = vw->pltrst.pltrst_status;
		vw->pltrst.pltrst_avail = false;
	} else {
		add_wait_queue(&vw->pltrst.pltrst_wq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		old_sample = vw->pltrst.pltrst_status;

		do {
			if (old_sample != vw->pltrst.pltrst_status) {
				data = vw->pltrst.pltrst_status;
				vw->pltrst.pltrst_avail = false;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
			} else {
				spin_unlock_irq(&vw->pltrst.pltrst_lock);
				schedule();
				spin_lock_irq(&vw->pltrst.pltrst_lock);
			}
		} while (!ret);

		remove_wait_queue(&vw->pltrst.pltrst_wq, &wait);
		set_current_state(TASK_RUNNING);
	}
out_unlock:
	spin_unlock_irq(&vw->pltrst.pltrst_lock);
	if (ret)
		return ret;

	ret = put_user(data, buf);
	if (!ret)
		ret = sizeof(data);

	return ret;
}

static unsigned int ast2700_espi_vw_pltrst_poll(struct file *file,
						poll_table *wait)
{
	struct aspeed_espi_vw *vw = to_ast2700_espi_pltrst(file);
	unsigned int mask = 0;

	poll_wait(file, &vw->pltrst.pltrst_wq, wait);
	if (vw->pltrst.pltrst_avail)
		mask |= POLLIN;
	return mask;
}

static const struct file_operations ast2700_espi_vw_pltrst_fops = {
	.owner = THIS_MODULE,
	.open = ast2700_espi_vw_pltrst_open,
	.read = ast2700_espi_vw_pltrst_read,
	.poll = ast2700_espi_vw_pltrst_poll,
};

static void ast2700_espi_vw_isr(struct aspeed_espi *espi)
{
	struct aspeed_espi_vw *vw;
	u32 sts_evt;
	u32 sts;
	u32 evt;
	unsigned long flags;

	vw = &espi->vw;

	sts = readl(espi->regs + ESPI_CH1_INT_STS);

	if (sts & ESPI_CH1_INT_STS_GPIO) {
		vw->gpio.val0 = readl(espi->regs + ESPI_CH1_GPIO_VAL0);
		vw->gpio.val1 = readl(espi->regs + ESPI_CH1_GPIO_VAL1);
		/* Workaround for AST2700 VW INT_STS */
		writel(ESPI_CH1_INT_STS_GPIO_CLR, espi->regs + ESPI_CH1_INT_STS);
	}

	if (sts & ESPI_CH1_INT_STS_EVT1) {
		sts_evt = readl(espi->regs + ESPI_CH1_EVT1_INT_STS);
		if (sts_evt & ESPI_CH1_EVT1_INT_STS_PCH_GENE) {
			vw->platform.pch_generic = FIELD_GET(ESPI_CH1_EVT1_PCH_GENE,
							     readl(espi->regs + ESPI_CH1_EVT1));

			writel(ESPI_CH1_EVT1_INT_STS_PCH_GENE,
			       espi->regs + ESPI_CH1_EVT1_INT_STS);
		}

		writel(ESPI_CH1_INT_STS_EVT1_CLR, espi->regs + ESPI_CH1_INT_STS);
	}

	if (sts & ESPI_CH1_INT_STS_EVT0) {
		sts_evt = readl(espi->regs + ESPI_CH1_EVT0_INT_STS);
		evt = readl(espi->regs + ESPI_CH1_EVT0);
		if (sts_evt & ESPI_CH1_EVT0_INT_STS_PLTRSTN || vw->pltrst.pltrst_status == 'U') {
			spin_lock_irqsave(&vw->pltrst.pltrst_lock, flags);
			vw->pltrst.pltrst_status = (evt & ESPI_CH1_EVT0_PLTRSTN) ? '1' : '0';
			vw->pltrst.pltrst_avail = true;
			spin_unlock_irqrestore(&vw->pltrst.pltrst_lock, flags);

			writel(ESPI_CH1_EVT0_INT_STS_PLTRSTN,
			       espi->regs + ESPI_CH1_EVT0_INT_STS);

			wake_up_interruptible(&vw->pltrst.pltrst_wq);
		}
		writel(ESPI_CH1_INT_STS_EVT0, espi->regs + ESPI_CH1_INT_STS);
	}
}

static void ast2700_espi_vw_reset(struct aspeed_espi *espi)
{
	u32 reg;
	struct aspeed_espi_vw *vw = &espi->vw;

	writel(0x0, espi->regs + ESPI_CH1_INT_EN);
	writel(0xffffffff, espi->regs + ESPI_CH1_INT_STS);

	writel(vw->gpio.grp, espi->regs + ESPI_CH1_GPIO_GRP);
	writel(vw->gpio.dir0, espi->regs + ESPI_CH1_GPIO_DIR0);
	writel(vw->gpio.dir1, espi->regs + ESPI_CH1_GPIO_DIR1);

	vw->gpio.val0 = readl(espi->regs + ESPI_CH1_GPIO_VAL0);
	vw->gpio.val1 = readl(espi->regs + ESPI_CH1_GPIO_VAL1);
	vw->platform.pch_generic = FIELD_GET(ESPI_CH1_EVT1_PCH_GENE,
					     readl(espi->regs + ESPI_CH1_EVT1));

	writel(ESPI_CH1_EVT1_INT_T2_PCH_GENE,
	       espi->regs + ESPI_CH1_EVT1_INT_T2);
	writel(ESPI_CH1_EVT1_INT_EN_PCH_GENE, espi->regs + ESPI_CH1_EVT1_INT_EN);

	if (vw->pltrst.enabled) {
		spin_lock(&vw->pltrst.pltrst_lock);
		vw->pltrst.pltrst_status = 'U'; /* Unknown */
		vw->pltrst.pltrst_avail = true;
		spin_unlock(&vw->pltrst.pltrst_lock);

		writel(ESPI_CH1_EVT0_INT_T2_PLTRSTN,
		       espi->regs + ESPI_CH1_EVT0_INT_T2);
		writel(ESPI_CH1_EVT0_INT_EN_PLTRSTN, espi->regs + ESPI_CH1_EVT0_INT_EN);
		writel(ESPI_CH1_INT_EN_GPIO | ESPI_CH1_INT_EN_SYS_EVT1 | ESPI_CH1_INT_EN_SYS_EVT0,
		       espi->regs + ESPI_CH1_INT_EN);
	} else {
		writel(ESPI_CH1_INT_EN_GPIO | ESPI_CH1_INT_EN_SYS_EVT1, espi->regs + ESPI_CH1_INT_EN);
	}

	reg = readl(espi->regs + ESPI_CH1_CTRL)
	      | ((vw->gpio.hw_mode) ? ESPI_CH1_CTRL_GPIO_HW : 0)
	      | ESPI_CH1_CTRL_SW_RDY;
	writel(reg, espi->regs + ESPI_CH1_CTRL);
}

int ast2700_espi_vw_probe(struct aspeed_espi *espi)
{
	int rc;
	struct device *dev = espi->dev;
	struct aspeed_espi_vw *vw = &espi->vw;

	vw->gpio.hw_mode = of_property_read_bool(dev->of_node, "vw-gpio-hw-mode");
	of_property_read_u32(dev->of_node, "vw-gpio-group", &vw->gpio.grp);
	of_property_read_u32_index(dev->of_node, "vw-gpio-direction", 0, &vw->gpio.dir0);
	of_property_read_u32_index(dev->of_node, "vw-gpio-direction", 1, &vw->gpio.dir1);

	vw->pltrst.enabled = of_property_read_bool(dev->of_node, "vw-pltrst-monitor");
	if (vw->pltrst.enabled) {
		spin_lock_init(&vw->pltrst.pltrst_lock);
		init_waitqueue_head(&vw->pltrst.pltrst_wq);
		vw->pltrst.pltrst_status = 'U'; /* Unknown */
		vw->pltrst.pltrst_avail = false;
		vw->pltrst_mdev.parent = dev;
		vw->pltrst_mdev.minor = MISC_DYNAMIC_MINOR;
		vw->pltrst_mdev.name =
			devm_kasprintf(dev, GFP_KERNEL, "%s-pltrstn%d",
				       DEVICE_NAME, espi->dev_id);
		vw->pltrst_mdev.fops = &ast2700_espi_vw_pltrst_fops;
		rc = misc_register(&vw->pltrst_mdev);
		if (rc) {
			dev_err(dev, "cannot register device %s\n", vw->pltrst_mdev.name);
			return rc;
		}
	}

	vw->mdev.parent = dev;
	vw->mdev.minor = MISC_DYNAMIC_MINOR;
	vw->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-vw%d", DEVICE_NAME, espi->dev_id);
	vw->mdev.fops = &ast2700_espi_vw_fops;
	rc = misc_register(&vw->mdev);
	if (rc) {
		dev_err(dev, "cannot register device %s\n", vw->mdev.name);
		return rc;
	}

	ast2700_espi_vw_reset(espi);

	return 0;
}

int ast2700_espi_vw_remove(struct aspeed_espi *espi)
{
	struct aspeed_espi_vw *vw;

	vw = &espi->vw;

	writel(0x0, espi->regs + ESPI_CH1_INT_EN);

	misc_deregister(&vw->mdev);

	return 0;
}

/* out-of-band channel (CH2) */
static long ast2700_espi_oob_dma_get_rx(struct file *fp,
					struct aspeed_espi_oob *oob,
					struct aspeed_espi_ioc *ioc)
{
	struct ast2700_espi_oob_dma_rx_desc *d;
	struct ast2700_espi_oob_dma_rx_desc *rx_descs;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u32 wptr, pkt_len;
	unsigned long flags;
	u8 *pkt;
	int rc;

	espi = container_of(oob, struct aspeed_espi, oob);

	wptr = FIELD_PREP(ESPI_CH2_RX_DESC_WPTR_WP, readl(espi->regs + ESPI_CH2_RX_DESC_WPTR));

	rx_descs = (struct ast2700_espi_oob_dma_rx_desc *)oob->dma.rxd_virt;
	d = &rx_descs[wptr];

	if (!d->dirty)
		return -EFAULT;

	pkt_len = ((d->len) ? d->len : ESPI_MAX_PLD_LEN) + sizeof(struct espi_comm_hdr);

	if (ioc->pkt_len < pkt_len)
		return -EINVAL;

	pkt = vmalloc(pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = d->cyc;
	hdr->tag = d->tag;
	hdr->len_h = d->len >> 8;
	hdr->len_l = d->len & 0xff;
	memcpy(hdr + 1, oob->dma.rx_virt + (PAGE_SIZE * wptr), pkt_len - sizeof(*hdr));

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&oob->lock, flags);

	/* make current descriptor available again */
	d->dirty = 0;

	wptr = ((wptr + 1) % OOB_DMA_DESC_NUM);
	writel(wptr | ESPI_CH2_RX_DESC_WPTR_VALID, espi->regs + ESPI_CH2_RX_DESC_WPTR);

	/* set ready flag base on the next RX descriptor */
	oob->rx_ready = rx_descs[wptr].dirty;

	spin_unlock_irqrestore(&oob->lock, flags);

	rc = 0;

free_n_out:
	vfree(pkt);

	return rc;
}

static long ast2700_espi_oob_get_rx(struct file *fp,
				    struct aspeed_espi_oob *oob,
				    struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	unsigned long flags;
	u32 pkt_len;
	u8 *pkt;
	int i, rc;

	espi = container_of(oob, struct aspeed_espi, oob);

	if (fp->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&oob->rx_mtx))
			return -EAGAIN;

		if (!oob->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	} else {
		mutex_lock(&oob->rx_mtx);

		if (!oob->rx_ready) {
			rc = wait_event_interruptible(oob->wq, oob->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}

	if (oob->dma.enable) {
		rc = ast2700_espi_oob_dma_get_rx(fp, oob, ioc);
		goto unlock_mtx_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	reg = readl(espi->regs + ESPI_CH2_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH2_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH2_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH2_RX_CTRL_LEN, reg);

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	pkt_len = ((len) ? len : ESPI_MAX_PLD_LEN) + sizeof(struct espi_comm_hdr);

	if (ioc->pkt_len < pkt_len) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	for (i = sizeof(*hdr); i < pkt_len; ++i) {
		reg = readl(espi->regs + ESPI_CH2_RX_DATA);
		pkt[i] = reg & 0xff;
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&oob->lock, flags);

	writel(ESPI_CH2_RX_CTRL_SERV_PEND, espi->regs + ESPI_CH2_RX_CTRL);
	oob->rx_ready = 0;

	spin_unlock_irqrestore(&oob->lock, flags);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&oob->rx_mtx);

	return rc;
}

static long ast2700_espi_oob_dma_put_tx(struct file *fp,
					struct aspeed_espi_oob *oob,
					struct aspeed_espi_ioc *ioc)
{
	struct ast2700_espi_oob_dma_tx_desc *d;
	struct ast2700_espi_oob_dma_tx_desc *tx_descs;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u32 rptr, wptr;
	u8 *pkt;
	int rc;

	espi = container_of(oob, struct aspeed_espi, oob);

	pkt = vzalloc(ioc->pkt_len);
	if (!pkt)
		return -ENOMEM;

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/* kick HW to update descriptor read/write pointer */
	writel(ESPI_CH2_TX_DESC_RPTR_UPT, espi->regs + ESPI_CH2_TX_DESC_RPTR);

	rptr = readl(espi->regs + ESPI_CH2_TX_DESC_RPTR);
	wptr = readl(espi->regs + ESPI_CH2_TX_DESC_WPTR);

	if (((wptr + 1) % OOB_DMA_DESC_NUM) == rptr) {
		rc = -EBUSY;
		goto free_n_out;
	}

	tx_descs = (struct ast2700_espi_oob_dma_tx_desc *)oob->dma.txd_virt;
	d = &tx_descs[wptr];
	d->cyc = hdr->cyc;
	d->tag = hdr->tag;
	d->len = (hdr->len_h << 8) | (hdr->len_l & 0xff);
	d->msg_type = OOB_DMA_DESC_CUSTOM;

	memcpy(oob->dma.tx_virt + (PAGE_SIZE * wptr), hdr + 1,  ioc->pkt_len - sizeof(*hdr));

	dma_wmb();

	wptr = (wptr + 1) % OOB_DMA_DESC_NUM;
	writel(wptr | ESPI_CH2_TX_DESC_WPTR_VALID, espi->regs + ESPI_CH2_TX_DESC_WPTR);

	rc = 0;

free_n_out:
	vfree(pkt);

	return rc;
}

static long ast2700_espi_oob_put_tx(struct file *fp,
				    struct aspeed_espi_oob *oob,
				    struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u8 *pkt;
	int i, rc;

	espi = container_of(oob, struct aspeed_espi, oob);

	if (!mutex_trylock(&oob->tx_mtx))
		return -EAGAIN;

	if (oob->dma.enable) {
		rc = ast2700_espi_oob_dma_put_tx(fp, oob, ioc);
		goto unlock_mtx_n_out;
	}

	reg = readl(espi->regs + ESPI_CH2_TX_CTRL);
	if (reg & ESPI_CH2_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len > ESPI_MAX_PKT_LEN) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
		writel(pkt[i], espi->regs + ESPI_CH2_TX_DATA);

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH2_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH2_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH2_TX_CTRL_LEN, len)
	      | ESPI_CH2_TX_CTRL_TRIG_PEND;
	writel(reg, espi->regs + ESPI_CH2_TX_CTRL);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&oob->tx_mtx);

	return rc;
}

static long ast2700_espi_oob_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_oob *oob;
	struct aspeed_espi_ioc ioc;

	oob = container_of(fp->private_data, struct aspeed_espi_oob, mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
		return -EFAULT;

	if (ioc.pkt_len > ESPI_MAX_PKT_LEN)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_OOB_GET_RX:
		return ast2700_espi_oob_get_rx(fp, oob, &ioc);
	case ASPEED_ESPI_OOB_PUT_TX:
		return ast2700_espi_oob_put_tx(fp, oob, &ioc);
	};

	return -EINVAL;
}

static const struct file_operations ast2700_espi_oob_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ast2700_espi_oob_ioctl,
};

static void ast2700_espi_oob_isr(struct aspeed_espi *espi)
{
	struct aspeed_espi_oob *oob;
	unsigned long flags;
	u32 sts;

	oob = &espi->oob;

	sts = readl(espi->regs + ESPI_CH2_INT_STS);

	if (sts & ESPI_CH2_INT_STS_RX_CMPLT) {
		writel(ESPI_CH2_INT_STS_RX_CMPLT, espi->regs + ESPI_CH2_INT_STS);

		spin_lock_irqsave(&oob->lock, flags);
		oob->rx_ready = true;
		spin_unlock_irqrestore(&oob->lock, flags);

		wake_up_interruptible(&oob->wq);
	}
}

static void ast2700_espi_oob_reset(struct aspeed_espi *espi)
{
	struct aspeed_espi_oob *oob;
	struct ast2700_espi_oob_dma_tx_desc *tx_desc;
	struct ast2700_espi_oob_dma_rx_desc *rx_desc;
	dma_addr_t tx_addr, rx_addr;
	u32 reg;
	int i;

	oob = &espi->oob;

	writel(0x0, espi->regs + ESPI_CH2_INT_EN);
	writel(0xffffffff, espi->regs + ESPI_CH2_INT_STS);

	reg = readl(espi->regs + ESPI_CH2_CTRL);
	reg &= ~(ESPI_CH2_CTRL_TX_RST
		 | ESPI_CH2_CTRL_RX_RST
		 | ESPI_CH2_CTRL_TX_DMA_EN
		 | ESPI_CH2_CTRL_RX_DMA_EN
		 | ESPI_CH2_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH2_CTRL);

	udelay(1);

	reg |= (ESPI_CH2_CTRL_TX_RST | ESPI_CH2_CTRL_RX_RST);
	writel(reg, espi->regs + ESPI_CH2_CTRL);

	if (oob->dma.enable) {
		tx_addr = oob->dma.tx_addr;
		rx_addr = oob->dma.rx_addr;

		tx_desc = (struct ast2700_espi_oob_dma_tx_desc *)oob->dma.txd_virt;
		rx_desc = (struct ast2700_espi_oob_dma_rx_desc *)oob->dma.rxd_virt;

		for (i = 0; i < OOB_DMA_DESC_NUM; ++i) {
			tx_desc[i].data_addrh = upper_32_bits(tx_addr);
			tx_desc[i].data_addrl = tx_addr & 0xffffffff;
			tx_addr += PAGE_SIZE;

			rx_desc[i].data_addrh = upper_32_bits(rx_addr);
			rx_desc[i].data_addrl = rx_addr & 0xffffffff;
			rx_desc[i].dirty = 0;
			rx_addr += PAGE_SIZE;
		}

		writel(upper_32_bits(oob->dma.txd_addr), espi->regs + ESPI_CH2_TX_DMAH);
		writel(oob->dma.txd_addr & 0xffffffff, espi->regs + ESPI_CH2_TX_DMAL);
		writel(OOB_DMA_RPTR_KEY, espi->regs + ESPI_CH2_TX_DESC_RPTR);
		writel(0x0, espi->regs + ESPI_CH2_TX_DESC_WPTR);
		writel(OOB_DMA_DESC_NUM, espi->regs + ESPI_CH2_TX_DESC_EPTR);

		writel(upper_32_bits(oob->dma.rxd_addr), espi->regs + ESPI_CH2_RX_DMAH);
		writel(oob->dma.rxd_addr & 0xffffffff, espi->regs + ESPI_CH2_RX_DMAL);
		writel(OOB_DMA_RPTR_KEY, espi->regs + ESPI_CH2_RX_DESC_RPTR);
		writel(0x0, espi->regs + ESPI_CH2_RX_DESC_WPTR);
		writel(OOB_DMA_DESC_NUM, espi->regs + ESPI_CH2_RX_DESC_EPTR);

		reg = readl(espi->regs + ESPI_CH2_CTRL)
		      | ESPI_CH2_CTRL_TX_DMA_EN
		      | ESPI_CH2_CTRL_RX_DMA_EN;
		writel(reg, espi->regs + ESPI_CH2_CTRL);

		/* activate RX DMA to make OOB_FREE */
		reg = readl(espi->regs + ESPI_CH2_RX_DESC_WPTR) | ESPI_CH2_RX_DESC_WPTR_VALID;
		writel(reg, espi->regs + ESPI_CH2_RX_DESC_WPTR);
	}

	writel(ESPI_CH2_INT_EN_RX_CMPLT, espi->regs + ESPI_CH2_INT_EN);

	reg = readl(espi->regs + ESPI_CH2_CTRL) | ESPI_CH2_CTRL_SW_RDY;
	writel(reg, espi->regs + ESPI_CH2_CTRL);
}

int ast2700_espi_oob_probe(struct aspeed_espi *espi)
{
	struct aspeed_espi_oob *oob;
	struct device *dev;
	int rc;

	dev = espi->dev;

	oob = &espi->oob;

	init_waitqueue_head(&oob->wq);

	spin_lock_init(&oob->lock);

	mutex_init(&oob->tx_mtx);
	mutex_init(&oob->rx_mtx);

	oob->dma.enable = of_property_read_bool(dev->of_node, "oob-dma-mode");
	if (oob->dma.enable) {
		oob->dma.txd_virt = dmam_alloc_coherent(dev,
							sizeof(struct ast2700_espi_oob_dma_tx_desc) *
							OOB_DMA_DESC_NUM,
					    &oob->dma.txd_addr, GFP_KERNEL);
		if (!oob->dma.txd_virt) {
			dev_err(dev, "cannot allocate DMA TX descriptor\n");
			return -ENOMEM;
		}
		oob->dma.tx_virt = dmam_alloc_coherent(dev, PAGE_SIZE * OOB_DMA_DESC_NUM, &oob->dma.tx_addr, GFP_KERNEL);
		if (!oob->dma.tx_virt) {
			dev_err(dev, "cannot allocate DMA TX buffer\n");
			return -ENOMEM;
		}

		oob->dma.rxd_virt = dmam_alloc_coherent(dev,
							sizeof(struct ast2700_espi_oob_dma_rx_desc) *
							OOB_DMA_DESC_NUM,
						&oob->dma.rxd_addr, GFP_KERNEL);
		if (!oob->dma.rxd_virt) {
			dev_err(dev, "cannot allocate DMA RX descriptor\n");
			return -ENOMEM;
		}

		oob->dma.rx_virt = dmam_alloc_coherent(dev, PAGE_SIZE * OOB_DMA_DESC_NUM, &oob->dma.rx_addr, GFP_KERNEL);
		if (!oob->dma.rx_virt) {
			dev_err(dev, "cannot allocate DMA TX buffer\n");
			return -ENOMEM;
		}
	}

	oob->mdev.parent = dev;
	oob->mdev.minor = MISC_DYNAMIC_MINOR;
	oob->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-oob%d", DEVICE_NAME, espi->dev_id);
	oob->mdev.fops = &ast2700_espi_oob_fops;
	rc = misc_register(&oob->mdev);
	if (rc) {
		dev_err(dev, "cannot register device %s\n", oob->mdev.name);
		return rc;
	}

	ast2700_espi_oob_reset(espi);

	return 0;
}

int ast2700_espi_oob_remove(struct aspeed_espi *espi)
{
	struct aspeed_espi_oob *oob;
	struct device *dev;
	u32 reg;

	dev = espi->dev;

	oob = &espi->oob;

	writel(0x0, espi->regs + ESPI_CH2_INT_EN);

	reg = readl(espi->regs + ESPI_CH2_CTRL);
	reg &= ~(ESPI_CH2_CTRL_TX_DMA_EN
		 | ESPI_CH2_CTRL_RX_DMA_EN
		 | ESPI_CH2_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH2_CTRL);

	if (oob->dma.enable) {
		dmam_free_coherent(dev, sizeof(struct ast2700_espi_oob_dma_tx_desc) * OOB_DMA_DESC_NUM,
				   oob->dma.txd_virt, oob->dma.txd_addr);
		dmam_free_coherent(dev, PAGE_SIZE * OOB_DMA_DESC_NUM,
				   oob->dma.tx_virt, oob->dma.tx_addr);
		dmam_free_coherent(dev, sizeof(struct ast2700_espi_oob_dma_rx_desc) * OOB_DMA_DESC_NUM,
				   oob->dma.rxd_virt, oob->dma.rxd_addr);
		dmam_free_coherent(dev, PAGE_SIZE * OOB_DMA_DESC_NUM,
				   oob->dma.rx_virt, oob->dma.rx_addr);
	}

	mutex_destroy(&oob->tx_mtx);
	mutex_destroy(&oob->rx_mtx);

	misc_deregister(&oob->mdev);

	return 0;
}

/* flash channel (CH3) */
static long ast2700_espi_flash_get_rx(struct file *fp,
				      struct aspeed_espi_flash *flash,
				      struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	unsigned long flags;
	u32 pkt_len;
	u8 *pkt;
	int i, rc;

	rc = 0;

	espi = container_of(flash, struct aspeed_espi, flash);

	if (fp->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&flash->rx_mtx))
			return -EAGAIN;

		if (!flash->rx_ready) {
			rc = -ENODATA;
			goto unlock_mtx_n_out;
		}
	} else {
		mutex_lock(&flash->rx_mtx);

		if (!flash->rx_ready) {
			rc = wait_event_interruptible(flash->wq, flash->rx_ready);
			if (rc == -ERESTARTSYS) {
				rc = -EINTR;
				goto unlock_mtx_n_out;
			}
		}
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	reg = readl(espi->regs + ESPI_CH3_RX_CTRL);
	cyc = FIELD_GET(ESPI_CH3_RX_CTRL_CYC, reg);
	tag = FIELD_GET(ESPI_CH3_RX_CTRL_TAG, reg);
	len = FIELD_GET(ESPI_CH3_RX_CTRL_LEN, reg);

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (cyc) {
	case ESPI_FLASH_WRITE:
		pkt_len = ((len) ? len : ESPI_MAX_PLD_LEN) +
			  sizeof(struct espi_flash_rwe);
		break;
	case ESPI_FLASH_READ:
	case ESPI_FLASH_ERASE:
		pkt_len = sizeof(struct espi_flash_rwe);
		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		pkt_len = ((len) ? len : ESPI_MAX_PLD_LEN) +
			  sizeof(struct espi_flash_cmplt);
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
		pkt_len = sizeof(struct espi_flash_cmplt);
		break;
	default:
		rc = -EFAULT;
		goto unlock_mtx_n_out;
	}

	if (ioc->pkt_len < pkt_len) {
		rc = -EINVAL;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;
	hdr->cyc = cyc;
	hdr->tag = tag;
	hdr->len_h = len >> 8;
	hdr->len_l = len & 0xff;

	if (flash->dma.enable) {
		memcpy(hdr + 1, flash->dma.rx_virt, pkt_len - sizeof(*hdr));
	} else {
		for (i = sizeof(*hdr); i < pkt_len; ++i)
			pkt[i] = readl(espi->regs + ESPI_CH3_RX_DATA) & 0xff;
	}

	if (copy_to_user((void __user *)ioc->pkt, pkt, pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	spin_lock_irqsave(&flash->lock, flags);

	writel(ESPI_CH3_RX_CTRL_SERV_PEND, espi->regs + ESPI_CH3_RX_CTRL);
	flash->rx_ready = 0;

	spin_unlock_irqrestore(&flash->lock, flags);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&flash->rx_mtx);

	return rc;
}

static long ast2700_espi_flash_put_tx(struct file *fp,
				      struct aspeed_espi_flash *flash,
				      struct aspeed_espi_ioc *ioc)
{
	u32 reg, cyc, tag, len;
	struct aspeed_espi *espi;
	struct espi_comm_hdr *hdr;
	u8 *pkt;
	int i, rc;

	espi = container_of(flash, struct aspeed_espi, flash);

	if (!mutex_trylock(&flash->tx_mtx))
		return -EAGAIN;

	reg = readl(espi->regs + ESPI_CH3_TX_CTRL);
	if (reg & ESPI_CH3_TX_CTRL_TRIG_PEND) {
		rc = -EBUSY;
		goto unlock_mtx_n_out;
	}

	pkt = vmalloc(ioc->pkt_len);
	if (!pkt) {
		rc = -ENOMEM;
		goto unlock_mtx_n_out;
	}

	hdr = (struct espi_comm_hdr *)pkt;

	if (copy_from_user(pkt, (void __user *)ioc->pkt, ioc->pkt_len)) {
		rc = -EFAULT;
		goto free_n_out;
	}

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	if (flash->dma.enable) {
		memcpy(flash->dma.tx_virt, hdr + 1, ioc->pkt_len - sizeof(*hdr));
		dma_wmb();
	} else {
		for (i = sizeof(*hdr); i < ioc->pkt_len; ++i)
			writel(pkt[i], espi->regs + ESPI_CH3_TX_DATA);
	}

	cyc = hdr->cyc;
	tag = hdr->tag;
	len = (hdr->len_h << 8) | (hdr->len_l & 0xff);

	reg = FIELD_PREP(ESPI_CH3_TX_CTRL_CYC, cyc)
	      | FIELD_PREP(ESPI_CH3_TX_CTRL_TAG, tag)
	      | FIELD_PREP(ESPI_CH3_TX_CTRL_LEN, len)
	      | ESPI_CH3_TX_CTRL_TRIG_PEND;
	writel(reg, espi->regs + ESPI_CH3_TX_CTRL);

	rc = 0;

free_n_out:
	vfree(pkt);

unlock_mtx_n_out:
	mutex_unlock(&flash->tx_mtx);

	return rc;
}

static long ast2700_espi_flash_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_espi_flash *flash;
	struct aspeed_espi_ioc ioc;

	flash = container_of(fp->private_data, struct aspeed_espi_flash, mdev);

	if (copy_from_user(&ioc, (void __user *)arg, sizeof(ioc)))
		return -EFAULT;

	if (ioc.pkt_len > ESPI_MAX_PKT_LEN)
		return -EINVAL;

	switch (cmd) {
	case ASPEED_ESPI_FLASH_GET_RX:
		return ast2700_espi_flash_get_rx(fp, flash, &ioc);
	case ASPEED_ESPI_FLASH_PUT_TX:
		return ast2700_espi_flash_put_tx(fp, flash, &ioc);
	};

	return -EINVAL;
}

static const struct file_operations ast2700_espi_flash_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ast2700_espi_flash_ioctl,
};

static void ast2700_espi_flash_isr(struct aspeed_espi *espi)
{
	struct aspeed_espi_flash *flash;
	unsigned long flags;
	u32 sts;

	flash = &espi->flash;

	sts = readl(espi->regs + ESPI_CH3_INT_STS);

	if (sts & ESPI_CH3_INT_STS_RX_CMPLT) {
		writel(ESPI_CH3_INT_STS_RX_CMPLT, espi->regs + ESPI_CH3_INT_STS);

		spin_lock_irqsave(&flash->lock, flags);
		flash->rx_ready = true;
		spin_unlock_irqrestore(&flash->lock, flags);

		wake_up_interruptible(&flash->wq);
	}
}

static void ast2700_espi_flash_reset(struct aspeed_espi *espi)
{
	u32 reg;
	u64 mask;
	struct aspeed_espi_flash *flash = &espi->flash;

	writel(0x0, espi->regs + ESPI_CH3_INT_EN);
	writel(0xffffffff, espi->regs + ESPI_CH3_INT_STS);

	reg = readl(espi->regs + ESPI_CH3_CTRL);
	reg &= ~(ESPI_CH3_CTRL_TX_RST
		 | ESPI_CH3_CTRL_RX_RST
		 | ESPI_CH3_CTRL_TX_DMA_EN
		 | ESPI_CH3_CTRL_RX_DMA_EN
		 | ESPI_CH3_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH3_CTRL);

	udelay(1);

	reg |= (ESPI_CH3_CTRL_TX_RST | ESPI_CH3_CTRL_RX_RST);
	writel(reg, espi->regs + ESPI_CH3_CTRL);

	if (flash->edaf.mode == EDAF_MODE_MIX) {
		mask = ~(flash->edaf.size - 1);
		writel(upper_32_bits(mask), espi->regs + ESPI_CH3_EDAF_MASKH);
		writel(mask & 0xffffffff, espi->regs + ESPI_CH3_EDAF_MASKL);
		writel(upper_32_bits(flash->edaf.taddr), espi->regs + ESPI_CH3_EDAF_TADDRH);
		writel(flash->edaf.taddr & 0xffffffff, espi->regs + ESPI_CH3_EDAF_TADDRL);
	}

	reg = readl(espi->regs + ESPI_CH3_CTRL) & ~ESPI_CH3_CTRL_EDAF_MODE;
	reg |= FIELD_PREP(ESPI_CH3_CTRL_EDAF_MODE, flash->edaf.mode);
	writel(reg, espi->regs + ESPI_CH3_CTRL);

	if (flash->dma.enable) {
		writel(upper_32_bits(flash->dma.tx_addr), espi->regs + ESPI_CH3_TX_DMAH);
		writel(flash->dma.tx_addr & 0xffffffff, espi->regs + ESPI_CH3_TX_DMAL);
		writel(upper_32_bits(flash->dma.rx_addr), espi->regs + ESPI_CH3_RX_DMAH);
		writel(flash->dma.rx_addr & 0xffffffff, espi->regs + ESPI_CH3_RX_DMAL);

		reg = readl(espi->regs + ESPI_CH3_CTRL)
		      | ESPI_CH3_CTRL_TX_DMA_EN
		      | ESPI_CH3_CTRL_RX_DMA_EN;
		writel(reg, espi->regs + ESPI_CH3_CTRL);
	}

	writel(ESPI_CH3_INT_EN_RX_CMPLT, espi->regs + ESPI_CH3_INT_EN);

	reg = readl(espi->regs + ESPI_CH3_CTRL) | ESPI_CH3_CTRL_SW_RDY;
	writel(reg, espi->regs + ESPI_CH3_CTRL);
}

int ast2700_espi_flash_probe(struct aspeed_espi *espi)
{
	struct aspeed_espi_flash *flash;
	struct device_node *np;
	struct resource res;
	struct device *dev;
	void *virt;
	int rc;

	dev = espi->dev;

	flash = &espi->flash;

	init_waitqueue_head(&flash->wq);

	spin_lock_init(&flash->lock);

	mutex_init(&flash->tx_mtx);
	mutex_init(&flash->rx_mtx);

	flash->edaf.mode = EDAF_MODE_HW;

	of_property_read_u32(dev->of_node, "flash-edaf-mode", &flash->edaf.mode);
	dev_info(dev, "eDAF mode: 0x%x\n", flash->edaf.mode);
	if (flash->edaf.mode == EDAF_MODE_MIX) {
		np = of_parse_phandle(dev->of_node, "flash-edaf-tgt-addr", 0);
		if (!np || of_address_to_resource(np, 0, &res)) {
			dev_err(dev, "cannot get eDAF memory region\n");
			return -ENODEV;
		}

		of_node_put(np);

		flash->edaf.taddr = res.start;
		flash->edaf.size = resource_size(&res);

		virt = devm_ioremap_resource(dev, &res);
		if (!virt) {
			dev_err(dev, "cannot map eDAF memory region\n");
			return -ENOMEM;
		}
	}

	flash->dma.enable = of_property_read_bool(dev->of_node, "flash-dma-mode");
	if (flash->dma.enable) {
		flash->dma.tx_virt = dmam_alloc_coherent(dev, PAGE_SIZE, &flash->dma.tx_addr, GFP_KERNEL);
		if (!flash->dma.tx_virt) {
			dev_err(dev, "cannot allocate DMA TX buffer\n");
			return -ENOMEM;
		}

		flash->dma.rx_virt = dmam_alloc_coherent(dev, PAGE_SIZE, &flash->dma.rx_addr, GFP_KERNEL);
		if (!flash->dma.rx_virt) {
			dev_err(dev, "cannot allocate DMA RX buffer\n");
			return -ENOMEM;
		}
	}

	flash->mdev.parent = dev;
	flash->mdev.minor = MISC_DYNAMIC_MINOR;
	flash->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s-flash%d", DEVICE_NAME, espi->dev_id);
	flash->mdev.fops = &ast2700_espi_flash_fops;
	rc = misc_register(&flash->mdev);
	if (rc) {
		dev_err(dev, "cannot register device %s\n", flash->mdev.name);
		return rc;
	}

	ast2700_espi_flash_reset(espi);

	return 0;
}

int ast2700_espi_flash_remove(struct aspeed_espi *espi)
{
	struct aspeed_espi_flash *flash;
	struct device *dev;
	u32 reg;

	dev = espi->dev;

	flash = &espi->flash;

	writel(0x0, espi->regs + ESPI_CH3_INT_EN);

	reg = readl(espi->regs + ESPI_CH3_CTRL);
	reg &= ~(ESPI_CH3_CTRL_TX_DMA_EN
		 | ESPI_CH3_CTRL_RX_DMA_EN
		 | ESPI_CH3_CTRL_SW_RDY);
	writel(reg, espi->regs + ESPI_CH3_CTRL);

	if (flash->dma.enable) {
		dmam_free_coherent(dev, PAGE_SIZE, flash->dma.tx_virt, flash->dma.tx_addr);
		dmam_free_coherent(dev, PAGE_SIZE, flash->dma.rx_virt, flash->dma.rx_addr);
	}

	mutex_destroy(&flash->tx_mtx);
	mutex_destroy(&flash->rx_mtx);

	misc_deregister(&flash->mdev);

	return 0;
}

/* global control */
irqreturn_t ast2700_espi_isr(int irq, void *arg)
{
	u32 sts;
	struct aspeed_espi *espi = (struct aspeed_espi *)arg;

	sts = readl(espi->regs + ESPI_INT_STS);
	if (!sts)
		return IRQ_NONE;

	if (sts & ESPI_INT_STS_CH0)
		ast2700_espi_perif_isr(espi);

	if (sts & ESPI_INT_STS_CH1)
		ast2700_espi_vw_isr(espi);

	if (sts & ESPI_INT_STS_CH2)
		ast2700_espi_oob_isr(espi);

	if (sts & ESPI_INT_STS_CH3)
		ast2700_espi_flash_isr(espi);

	if (sts & ESPI_INT_STS_RST_DEASSERT) {
		ast2700_espi_perif_sw_reset(espi);
		ast2700_espi_perif_reset(espi);
		ast2700_espi_vw_reset(espi);
		ast2700_espi_oob_reset(espi);
		ast2700_espi_flash_reset(espi);
		writel(ESPI_INT_STS_RST_DEASSERT, espi->regs + ESPI_INT_STS);
	}

	return IRQ_HANDLED;
}

void ast2700_espi_pre_init(struct aspeed_espi *espi)
{
	struct device *dev;
	struct regmap *scu1;
	u32 reg;
	int rc;

	dev = espi->dev;

	scu1 = syscon_regmap_lookup_by_phandle(dev->of_node, "aspeed,syscon");
	if (IS_ERR(scu1)) {
		dev_err(dev, "failed to find SCU1 regmap, error %ld\n", PTR_ERR(scu1));
		return;
	}
	rc = regmap_update_bits(scu1, SCU1_DDR,
				SCU1_DDR_DIS_ESPI0_AHB | SCU1_DDR_DIS_ESPI1_AHB,
				0);
	if (rc) {
		dev_err(dev, "failed to update SCU1 regmap, error %d\n", rc);
		return;
	}

	espi->dev_id = ida_alloc(&ast2700_espi_ida, GFP_KERNEL);
	if (espi->dev_id < 0) {
		dev_err(dev, "cannote allocate device ID, error %d\n", espi->dev_id);
		return;
	}

	reg = readl(espi->regs + ESPI_INT_EN);
	reg &= ~ESPI_INT_EN_RST_DEASSERT;
	writel(reg, espi->regs + ESPI_INT_EN);
}

void ast2700_espi_post_init(struct aspeed_espi *espi)
{
	u32 reg;

	reg = readl(espi->regs + ESPI_INT_EN);
	reg |= ESPI_INT_EN_RST_DEASSERT;
	writel(reg, espi->regs + ESPI_INT_EN);
}

void ast2700_espi_deinit(struct aspeed_espi *espi)
{
	struct device *dev;
	u32 reg;

	dev = espi->dev;

	reg = readl(espi->regs + ESPI_INT_EN);
	reg &= ~ESPI_INT_EN_RST_DEASSERT;
	writel(reg, espi->regs + ESPI_INT_EN);
}
