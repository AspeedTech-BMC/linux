// SPDX-License-Identifier: GPL-2.0
/* ASPEED CAN device driver
 *
 * Copyright (C) 2026 ASPEED Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

#define CAN_AC_SEG		0x0004 /* nominal (arbitration) phase timing */
#define  AC_SEG1_MASK		GENMASK(8, 0)
#define  AC_SEG2_MASK		GENMASK(22, 16)
#define  AC_SJW_MASK		GENMASK(30, 24)
#define CAN_FD_SEG		0x0008 /* data phase timing */
#define  FD_SEG1_MASK		GENMASK(7, 0)
#define  FD_SEG2_MASK		GENMASK(22, 16)
#define  FD_SJW_MASK		GENMASK(30, 24)
#define CAN_BITTIME		0x0010 /* prescaler, TDC, retransmit limit */
#define  PRESC_MASK		GENMASK(4, 0)
#define  FD_SSPOFF_MASK		GENMASK(15, 8)
#define  REALIM_MASK		GENMASK(26, 24)
#define  RETLIM_MASK		GENMASK(30, 28)
#define  RELIM_DEFAULT		0x7
#define CAN_INTF		0x0014 /* interrupt flags */
#define  INTF_EIF		BIT(1) /* general error */
#define  INTF_TSIF		BIT(2) /* STB TX done */
#define  INTF_RAFIF		BIT(4) /* RX buf almost full */
#define  INTF_RFIF		BIT(5) /* RX buf full */
#define  INTF_ROIF		BIT(6) /* RX buf overflow */
#define  INTF_RIF		BIT(7) /* frame received */
#define  INTF_BEIF		BIT(8) /* bus error */
#define  INTF_ALIF		BIT(9) /* arbitration loss */
#define  INTF_EPIF		BIT(10) /* error passive transition */
#define  INTF_EPASS		BIT(30) /* error passive state */
#define  INTF_EWARN		BIT(31) /* error warning limit reached */
#define  INTF_ERR_MASK		(INTF_EIF | INTF_ROIF | INTF_BEIF | \
				 INTF_ALIF | INTF_EPIF)
#define  INTF_RX_MASK		(INTF_RAFIF | INTF_RFIF | INTF_RIF)
#define CAN_INTE		0x0018 /* interrupt enable */
#define INTE_ALL		(INTF_ERR_MASK | INTF_TSIF | INTF_RX_MASK)
#define CAN_CFG_STAT		0x0028 /* cfg_stat, tcmd, tctrl, rctrl */
#define  CFGSTAT_BUSOFF		BIT(0) /* bus off */
#define  CFGSTAT_LBMI		BIT(5) /* loop back mode internal */
#define  CFGSTAT_LBME		BIT(6) /* loop back mode external */
#define  CFGSTAT_RESET		BIT(7) /* reset mode */
#define  TCMD_TSALL		BIT(9) /* transmit STB all frames */
#define  TCMD_LOM		BIT(14) /* listen-only mode */
#define  TCMD_TBSEL		BIT(15) /* TX buf select: 0=PTB, 1=STB */
#define  TCTRL_TSFF		BIT(18) /* STB full flag */
#define  TCTRL_TSMODE		BIT(21) /* STB mode (0=FIFO, 1=priority) */
#define  TCTRL_TSNEXT		BIT(22) /* STB next: advance to next slot */
#define  TCTRL_FD_ISO		BIT(23) /* FD ISO mode (ISO 11898-1:2015) */
#define  RCTRL_RSTAT_MASK	GENMASK(25, 24) /* receive status */
#define  RCTRL_RREL		BIT(28) /* receive release */
#define  RCTRL_SACK		BIT(31) /* self-ACK in loopback mode */
#define CAN_LIMIT		0x002c /* limit, ealcap, recnt, tecnt */
#define  LIMIT_EWL_MASK		GENMASK(3, 0) /* error warning limit */
#define  LIMIT_AFWL_MASK	GENMASK(7, 4) /* almost full warning level */
#define  EALCAP_KOER_MASK	GENMASK(15, 13) /* kind of error */
#define  RECNT_MASK		GENMASK(23, 16) /* receive error counter */
#define  TECNT_MASK		GENMASK(31, 24) /* transmit error counter */
#define  LIMIT_AFWL_DEFAULT	0x2
#define  LIMIT_EWL_DEFAULT	0xb
#define CAN_ACFCTRL		0x0044 /* acceptance filter control */
#define  ACFADR_MASK		GENMASK(3, 0) /* selects accessed filter */
#define  ACF_EN_MASK		GENMASK(31, 16) /* per-filter enable bits */
#define CAN_ACFC		0x0048 /* acceptance filter code */
#define CAN_ACFM		0x0058 /* acceptance filter mask */
#define CAN_RBUF		0x0070 /* receive buffer base */
#define CAN_TBUF		0x0890 /* transmit buffer base */
#define CAN_MODE_CFG		0x1100 /* can ctrl mode config */
#define  MODE_CFG_FD_EN		BIT(0) /* enable CAN FD mode */

/* LLC frame buffer layout (TBUF/RBUF slot) */
#define BUF_ID			0x00
#define  BUF_ID_EFF_MASK	GENMASK(28, 0)
#define  BUF_ID_SFF_MASK	GENMASK(28, 18)
#define BUF_CTL			0x04
#define  BUF_DLC_MASK		GENMASK(10, 0)
#define  BUF_IDE		BIT(16)
#define  BUF_FDF		BIT(17)
#define  BUF_BRS		BIT(18)
#define  BUF_RMF		BIT(20)
#define  BUF_ESI		BIT(22)
#define BUF_TYPE		0x08
#define  BUF_HANDLE_SHIFT	24
#define  BUF_HANDLE_MAX		0x100
#define BUF_DATA		0x10

/* KOER (Kind Of ERror) field values */
#define KOER_BIT		1
#define KOER_FORM		2
#define KOER_STUFF		3
#define KOER_ACK		4
#define KOER_CRC		5

#define STB_SLOTS		3
#define ECHO_SKB_SLOTS		STB_SLOTS
#define ASPEED_CAN_RX_SLOTS	3
#define ASPEED_CAN_NAPI_WEIGHT	ASPEED_CAN_RX_SLOTS
#define STB_INVALID_HANDLE	0xffffffffu

struct aspeed_can_stb_slot {
	struct list_head list;
	u32 skb_idx;
	u32 handle;
};

struct aspeed_can_priv {
	struct can_priv can;
	void __iomem *reg_base;
	spinlock_t tx_lock; /* Protects STB queue and echo skb state. */
	struct list_head stb_head;
	struct aspeed_can_stb_slot stb_slots[STB_SLOTS];
	struct napi_struct napi;
	struct clk *clk;
	struct reset_control *reset;
	u32 frame_handle;
};

static const struct can_bittiming_const aspeed_can_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,
	.tseg1_max = 512,
	.tseg2_min = 1,
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

static const struct can_bittiming_const aspeed_canfd_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,
	.tseg1_max = 254,
	.tseg2_min = 1,
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

static const struct can_tdc_const aspeed_canfd_tdc_const = {
	/* Manual TDCV is unsupported. */
	.tdcv_min = 0,
	.tdcv_max = 0,
	.tdco_min = 0,
	.tdco_max = 255,
	/* Filter window not supported */
	.tdcf_min = 0,
	.tdcf_max = 0,
};

static u32 aspeed_can_read(struct aspeed_can_priv *priv, u32 reg)
{
	return readl(priv->reg_base + reg);
}

static void aspeed_can_write(struct aspeed_can_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->reg_base + reg);
}

static void aspeed_can_set_bits(struct aspeed_can_priv *priv, u32 reg, u32 mask)
{
	writel(readl(priv->reg_base + reg) | mask, priv->reg_base + reg);
}

static void aspeed_can_clr_bits(struct aspeed_can_priv *priv, u32 reg, u32 mask)
{
	writel(readl(priv->reg_base + reg) & ~mask, priv->reg_base + reg);
}

static void aspeed_can_clr_irq(struct aspeed_can_priv *priv, u32 bits)
{
	writel(bits, priv->reg_base + CAN_INTF);
}

static void aspeed_can_stb_init(struct aspeed_can_priv *priv)
{
	int i;

	INIT_LIST_HEAD(&priv->stb_head);
	for (i = 0; i < STB_SLOTS; i++) {
		INIT_LIST_HEAD(&priv->stb_slots[i].list);
		priv->stb_slots[i].skb_idx = i;
		priv->stb_slots[i].handle = STB_INVALID_HANDLE;
	}
}

static struct aspeed_can_stb_slot *
aspeed_can_stb_alloc(struct aspeed_can_priv *priv)
{
	int i;

	for (i = 0; i < STB_SLOTS; i++) {
		if (list_empty(&priv->stb_slots[i].list))
			return &priv->stb_slots[i];
	}
	return NULL;
}

static struct aspeed_can_stb_slot *
aspeed_can_stb_find(struct aspeed_can_priv *priv, u32 handle)
{
	struct aspeed_can_stb_slot *slot;

	list_for_each_entry(slot, &priv->stb_head, list) {
		if (slot->handle == handle)
			return slot;
	}
	return NULL;
}

static int aspeed_can_set_reset_mode(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	unsigned long flags;
	u32 val;
	int ret;

	aspeed_can_set_bits(priv, CAN_CFG_STAT, CFGSTAT_RESET);

	ret = readx_poll_timeout(readl, priv->reg_base + CAN_CFG_STAT,
				 val, val & CFGSTAT_RESET, 500, USEC_PER_SEC);
	if (ret) {
		netdev_warn(ndev, "timed out entering reset mode\n");
		return ret;
	}

	spin_lock_irqsave(&priv->tx_lock, flags);
	aspeed_can_stb_init(priv);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	return 0;
}

static int aspeed_can_exit_reset_mode(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 val;
	int ret;

	aspeed_can_clr_bits(priv, CAN_CFG_STAT, CFGSTAT_RESET);

	ret = readx_poll_timeout(readl, priv->reg_base + CAN_CFG_STAT,
				 val, !(val & CFGSTAT_RESET),
				 500, USEC_PER_SEC);
	if (ret)
		netdev_warn(ndev, "timed out exiting reset mode\n");

	return ret;
}

static int aspeed_can_do_set_bittiming(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u32 ac_seg, val;

	if (!(aspeed_can_read(priv, CAN_CFG_STAT) & CFGSTAT_RESET))
		return 0;

	ac_seg = FIELD_PREP(AC_SEG1_MASK, bt->prop_seg + bt->phase_seg1 - 1) |
		 FIELD_PREP(AC_SEG2_MASK, bt->phase_seg2 - 1) |
		 FIELD_PREP(AC_SJW_MASK, bt->sjw - 1);
	aspeed_can_write(priv, CAN_AC_SEG, ac_seg);

	/* Prescaler is shared with the data phase. */
	val = aspeed_can_read(priv, CAN_BITTIME);
	val &= ~PRESC_MASK;
	val |= FIELD_PREP(PRESC_MASK, bt->brp - 1);
	aspeed_can_write(priv, CAN_BITTIME, val);

	return 0;
}

static int aspeed_can_do_set_data_bittiming(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.fd.data_bittiming;
	u32 fd_seg, val;

	if (!(aspeed_can_read(priv, CAN_CFG_STAT) & CFGSTAT_RESET))
		return 0;

	/*
	 * The hardware has a single prescaler shared by both the nominal and
	 * data phase, so both must be configured with the same BRP value.
	 */
	if (bt->brp != dbt->brp) {
		netdev_alert(ndev,
			     "nominal (%u) and data (%u) prescaler must match\n",
			     bt->brp, dbt->brp);
		return -EINVAL;
	}

	fd_seg = FIELD_PREP(FD_SEG1_MASK,
			    dbt->prop_seg + dbt->phase_seg1 - 1) |
		 FIELD_PREP(FD_SEG2_MASK, dbt->phase_seg2 - 1) |
		 FIELD_PREP(FD_SJW_MASK, dbt->sjw - 1);
	aspeed_can_write(priv, CAN_FD_SEG, fd_seg);

	val = aspeed_can_read(priv, CAN_BITTIME);
	val &= ~FD_SSPOFF_MASK;
	if (can_fd_tdc_is_enabled(&priv->can))
		val |= FIELD_PREP(FD_SSPOFF_MASK,
				  priv->can.fd.tdc.tdco / dbt->brp);
	aspeed_can_write(priv, CAN_BITTIME, val);

	return 0;
}

static void aspeed_can_configure_filters(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 ctrl;

	/* Accept every frame ID in hardware. */
	aspeed_can_clr_bits(priv, CAN_ACFCTRL, ACF_EN_MASK);

	ctrl = aspeed_can_read(priv, CAN_ACFCTRL);
	ctrl &= ~ACFADR_MASK;
	aspeed_can_write(priv, CAN_ACFCTRL, ctrl);

	aspeed_can_write(priv, CAN_ACFC + BUF_ID, 0);
	aspeed_can_write(priv, CAN_ACFC + BUF_CTL, 0);
	aspeed_can_write(priv, CAN_ACFC + BUF_TYPE, 0);
	aspeed_can_write(priv, CAN_ACFM + BUF_ID, 0xffffffff);
	aspeed_can_write(priv, CAN_ACFM + BUF_CTL, 0xffffffff);
	aspeed_can_write(priv, CAN_ACFM + BUF_TYPE, 0xffffffff);

	aspeed_can_set_bits(priv, CAN_ACFCTRL, FIELD_PREP(ACF_EN_MASK, BIT(0)));
}

static int aspeed_can_chip_start(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 cfg, val;
	int ret;

	ret = aspeed_can_set_reset_mode(ndev);
	if (ret)
		return ret;

	ret = aspeed_can_do_set_bittiming(ndev);
	if (ret)
		return ret;

	aspeed_can_clr_bits(priv, CAN_MODE_CFG, MODE_CFG_FD_EN);
	aspeed_can_clr_bits(priv, CAN_CFG_STAT, TCTRL_FD_ISO);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		ret = aspeed_can_do_set_data_bittiming(ndev);
		if (ret)
			return ret;

		aspeed_can_set_bits(priv, CAN_MODE_CFG, MODE_CFG_FD_EN);
		/* Set explicitly to guarantee FD ISO mode. */
		aspeed_can_set_bits(priv, CAN_CFG_STAT, TCTRL_FD_ISO);
	}

	aspeed_can_configure_filters(ndev);

	ret = aspeed_can_exit_reset_mode(ndev);
	if (ret)
		return ret;

	val = aspeed_can_read(priv, CAN_BITTIME);
	val &= ~(RETLIM_MASK | REALIM_MASK);
	if (!(priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT))
		val |= FIELD_PREP(RETLIM_MASK, RELIM_DEFAULT) |
		       FIELD_PREP(REALIM_MASK, RELIM_DEFAULT);
	aspeed_can_write(priv, CAN_BITTIME, val);

	val = aspeed_can_read(priv, CAN_LIMIT);
	val &= ~(LIMIT_EWL_MASK | LIMIT_AFWL_MASK);
	aspeed_can_write(priv, CAN_LIMIT, val |
			 FIELD_PREP(LIMIT_AFWL_MASK, LIMIT_AFWL_DEFAULT) |
			 FIELD_PREP(LIMIT_EWL_MASK, LIMIT_EWL_DEFAULT));

	aspeed_can_set_bits(priv, CAN_CFG_STAT, TCMD_TBSEL);
	aspeed_can_clr_bits(priv, CAN_CFG_STAT, TCTRL_TSMODE);

	cfg = aspeed_can_read(priv, CAN_CFG_STAT);
	cfg &= ~(CFGSTAT_LBME | CFGSTAT_LBMI | TCMD_LOM | RCTRL_SACK);
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		cfg |= TCMD_LOM;
	else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		cfg |= RCTRL_SACK | CFGSTAT_LBME;
	aspeed_can_write(priv, CAN_CFG_STAT, cfg);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	priv->frame_handle = 0;

	aspeed_can_write(priv, CAN_INTE, INTE_ALL);

	return 0;
}

static int aspeed_can_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	unsigned long flags;
	int ret, i;

	switch (mode) {
	case CAN_MODE_START:
		spin_lock_irqsave(&priv->tx_lock, flags);
		for (i = 0; i < STB_SLOTS; i++)
			can_free_echo_skb(ndev, i, NULL);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		ret = aspeed_can_chip_start(ndev);
		if (ret) {
			netdev_err(ndev, "chip_start failed: %d\n", ret);
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void aspeed_can_write_frame(struct net_device *ndev,
				   struct sk_buff *skb)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	u32 id, ctl, type;
	unsigned int i;

	if (cf->can_id & CAN_EFF_FLAG)
		id = FIELD_PREP(BUF_ID_EFF_MASK, cf->can_id & CAN_EFF_MASK);
	else
		id = FIELD_PREP(BUF_ID_SFF_MASK, cf->can_id & CAN_SFF_MASK);

	if (can_is_canfd_skb(skb))
		ctl = can_fd_len2dlc(cf->len);
	else
		ctl = can_get_cc_dlc((struct can_frame *)cf,
				     priv->can.ctrlmode);

	if (cf->can_id & CAN_EFF_FLAG)
		ctl |= BUF_IDE;
	if (can_is_canfd_skb(skb)) {
		ctl |= BUF_FDF;
		if (cf->flags & CANFD_BRS)
			ctl |= BUF_BRS;
	} else {
		if (cf->can_id & CAN_RTR_FLAG)
			ctl |= BUF_RMF;
	}

	type = (u32)priv->frame_handle << BUF_HANDLE_SHIFT;

	aspeed_can_write(priv, CAN_TBUF + BUF_ID, id);
	aspeed_can_write(priv, CAN_TBUF + BUF_CTL, ctl);
	aspeed_can_write(priv, CAN_TBUF + BUF_TYPE, type);

	if (cf->can_id & CAN_RTR_FLAG)
		return;

	for (i = 0; i < round_up(cf->len, 4); i += 4)
		aspeed_can_write(priv, CAN_TBUF + BUF_DATA + i,
				 *(u32 *)(cf->data + i));
}

static int aspeed_can_do_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct aspeed_can_stb_slot *slot;
	u32 cfg;
	int ret;

	cfg = aspeed_can_read(priv, CAN_CFG_STAT);
	if (cfg & TCTRL_TSFF) {
		netdev_err(ndev, "hardware STB full\n");
		return -ENOSPC;
	}

	if (priv->frame_handle >= BUF_HANDLE_MAX)
		priv->frame_handle = 0;

	if (aspeed_can_stb_find(priv, priv->frame_handle)) {
		netdev_err(ndev, "handle %u already in STB\n",
			   priv->frame_handle);
		return -ENOSPC;
	}

	slot = aspeed_can_stb_alloc(priv);
	if (!slot) {
		netdev_err(ndev, "no free STB slot\n");
		return -ENOSPC;
	}

	aspeed_can_write_frame(ndev, skb);
	ret = can_put_echo_skb(skb, ndev, slot->skb_idx, 0);
	if (ret) {
		ndev->stats.tx_dropped++;
		return 0;
	}

	aspeed_can_set_bits(priv, CAN_CFG_STAT, TCTRL_TSNEXT);
	/*
	 * TSNEXT is self-clearing; HW updates the STB write pointer only
	 * after it clears, so wait for it before issuing the TX command.
	 */
	if (readl_poll_timeout_atomic(priv->reg_base + CAN_CFG_STAT, cfg,
				      !(cfg & TCTRL_TSNEXT), 0, 10)) {
		netdev_err(ndev, "timeout waiting for TSNEXT\n");
		can_free_echo_skb(ndev, slot->skb_idx, NULL);
		ndev->stats.tx_dropped++;
		ndev->stats.tx_errors++;
		netif_stop_queue(ndev);
		return 0;
	}

	slot->handle = priv->frame_handle;
	list_add_tail(&slot->list, &priv->stb_head);

	if (!aspeed_can_stb_alloc(priv))
		netif_stop_queue(ndev);

	if (!(cfg & TCMD_TSALL))
		aspeed_can_set_bits(priv, CAN_CFG_STAT, TCMD_TSALL);

	priv->frame_handle++;
	return 0;
}

static netdev_tx_t aspeed_can_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	unsigned long flags;
	int ret;

	if (can_dev_dropped_skb(ndev, skb))
		return NETDEV_TX_OK;

	spin_lock_irqsave(&priv->tx_lock, flags);
	ret = aspeed_can_do_xmit(skb, ndev);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (ret) {
		netif_stop_queue(ndev);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
	}

	return NETDEV_TX_OK;
}

static int aspeed_can_rx(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 id_reg, ctl_reg;
	u8 dlc;
	unsigned int i;

	if (!(aspeed_can_read(priv, CAN_CFG_STAT) & RCTRL_RSTAT_MASK))
		return 0;

	ctl_reg = aspeed_can_read(priv, CAN_RBUF + BUF_CTL);
	dlc = FIELD_GET(BUF_DLC_MASK, ctl_reg) & CAN_MAX_RAW_DLC;

	if (ctl_reg & BUF_FDF)
		skb = alloc_canfd_skb(ndev, &cf);
	else
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);

	if (!skb) {
		stats->rx_dropped++;
		aspeed_can_set_bits(priv, CAN_CFG_STAT, RCTRL_RREL);
		return 1;
	}

	id_reg = aspeed_can_read(priv, CAN_RBUF + BUF_ID);

	if (ctl_reg & BUF_IDE)
		cf->can_id = FIELD_GET(BUF_ID_EFF_MASK, id_reg) | CAN_EFF_FLAG;
	else
		cf->can_id = FIELD_GET(BUF_ID_SFF_MASK, id_reg);

	if (ctl_reg & BUF_RMF)
		cf->can_id |= CAN_RTR_FLAG;

	if (ctl_reg & BUF_FDF) {
		cf->len = can_fd_dlc2len(dlc);
		if (ctl_reg & BUF_BRS)
			cf->flags |= CANFD_BRS;
		if (ctl_reg & BUF_ESI)
			cf->flags |= CANFD_ESI;
	} else {
		can_frame_set_cc_len((struct can_frame *)cf, dlc,
				     priv->can.ctrlmode);
	}

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i += 4)
			*(u32 *)(cf->data + i) =
				aspeed_can_read(priv, CAN_RBUF + BUF_DATA + i);
		stats->rx_bytes += cf->len;
	}

	stats->rx_packets++;

	aspeed_can_set_bits(priv, CAN_CFG_STAT, RCTRL_RREL);
	netif_receive_skb(skb);

	return 1;
}

static enum can_state aspeed_can_get_state(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 cfg  = aspeed_can_read(priv, CAN_CFG_STAT);
	u32 intf = aspeed_can_read(priv, CAN_INTF);

	if (cfg & CFGSTAT_BUSOFF)
		return CAN_STATE_BUS_OFF;
	if (intf & INTF_EPASS)
		return CAN_STATE_ERROR_PASSIVE;
	if (intf & INTF_EWARN)
		return CAN_STATE_ERROR_WARNING;

	return CAN_STATE_ERROR_ACTIVE;
}

static void aspeed_can_set_error_state(struct net_device *ndev,
				       enum can_state new_state,
				       struct can_frame *cf)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 cnt = aspeed_can_read(priv, CAN_LIMIT);
	u32 txerr = FIELD_GET(TECNT_MASK, cnt);
	u32 rxerr = FIELD_GET(RECNT_MASK, cnt);
	enum can_state tx_state = txerr >= rxerr ? new_state : 0;
	enum can_state rx_state = txerr <= rxerr ? new_state : 0;

	if (WARN_ON(new_state > CAN_STATE_ERROR_PASSIVE))
		return;

	can_change_state(ndev, cf, tx_state, rx_state);

	if (cf) {
		cf->can_id |= CAN_ERR_CNT;
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}
}

static void aspeed_can_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame cf = {};
	u32 cnt, cfg, koer;

	cnt  = aspeed_can_read(priv, CAN_LIMIT);
	cfg  = aspeed_can_read(priv, CAN_CFG_STAT);
	koer = FIELD_GET(EALCAP_KOER_MASK, cnt);

	if (cfg & CFGSTAT_BUSOFF) {
		u32 txerr = FIELD_GET(TECNT_MASK, cnt);
		u32 rxerr = FIELD_GET(RECNT_MASK, cnt);

		can_change_state(ndev, &cf,
				 CAN_STATE_BUS_OFF, CAN_STATE_BUS_OFF);
		cf.can_id |= CAN_ERR_CNT;
		cf.data[6] = txerr;
		cf.data[7] = rxerr;
		aspeed_can_set_bits(priv, CAN_CFG_STAT, CFGSTAT_RESET);
		can_bus_off(ndev);
	} else {
		enum can_state new_state = aspeed_can_get_state(ndev);

		if (new_state != priv->can.state)
			aspeed_can_set_error_state(ndev, new_state, &cf);
	}

	if (isr & INTF_ALIF) {
		priv->can.can_stats.arbitration_lost++;
		cf.can_id |= CAN_ERR_LOSTARB;
		cf.data[0] = CAN_ERR_LOSTARB_UNSPEC;
	}

	if (isr & INTF_ROIF) {
		stats->rx_over_errors++;
		stats->rx_errors++;
		cf.can_id |= CAN_ERR_CRTL;
		cf.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
	}

	if (isr & INTF_BEIF) {
		bool berr = !!(priv->can.ctrlmode &
			       CAN_CTRLMODE_BERR_REPORTING);

		priv->can.can_stats.bus_error++;

		if (berr)
			cf.can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		switch (koer) {
		case KOER_BIT:
			stats->tx_errors++;
			if (berr)
				cf.data[2] = CAN_ERR_PROT_BIT;
			break;
		case KOER_FORM:
			stats->rx_errors++;
			if (berr)
				cf.data[2] = CAN_ERR_PROT_FORM;
			break;
		case KOER_STUFF:
			stats->rx_errors++;
			if (berr)
				cf.data[2] = CAN_ERR_PROT_STUFF;
			break;
		case KOER_ACK:
			stats->tx_errors++;
			if (berr) {
				cf.can_id |= CAN_ERR_ACK;
				cf.data[3] = CAN_ERR_PROT_LOC_ACK;
			}
			break;
		case KOER_CRC:
			stats->rx_errors++;
			if (berr)
				cf.data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
			break;
		default:
			break;
		}
	}

	if (cf.can_id) {
		struct can_frame *skb_cf;
		struct sk_buff *skb = alloc_can_err_skb(ndev, &skb_cf);

		if (skb) {
			skb_cf->can_id |= cf.can_id;
			memcpy(skb_cf->data, cf.data, CAN_ERR_DLC);
			netif_rx(skb);
		}
	}
}

static int aspeed_can_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	int work_done = 0;

	while (work_done < quota &&
	       aspeed_can_read(priv, CAN_CFG_STAT) & RCTRL_RSTAT_MASK)
		work_done += aspeed_can_rx(ndev);

	if (work_done < quota && napi_complete_done(napi, work_done))
		aspeed_can_set_bits(priv, CAN_INTE, INTF_RX_MASK);

	return work_done;
}

static void aspeed_can_tx_interrupt(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct aspeed_can_stb_slot *slot, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);

	aspeed_can_clr_irq(priv, INTF_TSIF);

	if (list_empty(&priv->stb_head)) {
		netdev_warn(ndev, "STB TX interrupt with no pending frame\n");
		netif_wake_queue(ndev);
		goto done;
	}

	/* TSIF fires only when the STB is empty; echo all pending frames. */
	list_for_each_entry_safe(slot, tmp, &priv->stb_head, list) {
		stats->tx_bytes += can_get_echo_skb(ndev, slot->skb_idx, NULL);
		stats->tx_packets++;
		list_del_init(&slot->list);
	}

	netif_wake_queue(ndev);

done:
	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

static irqreturn_t aspeed_can_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 isr;

	isr = aspeed_can_read(priv, CAN_INTF);
	if (!isr)
		return IRQ_NONE;

	if (isr & INTF_TSIF)
		aspeed_can_tx_interrupt(ndev);

	if (isr & INTF_RX_MASK) {
		if (isr & INTF_RAFIF)
			netdev_dbg(ndev, "RX buffer almost full\n");
		if (isr & INTF_RFIF)
			netdev_dbg(ndev, "RX buffer full\n");
		aspeed_can_clr_irq(priv, isr & INTF_RX_MASK);
		aspeed_can_clr_bits(priv, CAN_INTE, INTF_RX_MASK);
		napi_schedule(&priv->napi);
	}

	if (isr & INTF_ERR_MASK) {
		aspeed_can_clr_irq(priv, isr & INTF_ERR_MASK);
		aspeed_can_err_interrupt(ndev, isr);
	}

	return IRQ_HANDLED;
}

static void aspeed_can_chip_stop(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	int ret;

	aspeed_can_write(priv, CAN_INTE, 0);
	ret = aspeed_can_set_reset_mode(ndev);
	if (ret)
		netdev_err(ndev, "chip_stop: set_reset_mode failed\n");

	priv->can.state = CAN_STATE_STOPPED;
}

static int aspeed_can_open(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	int ret;

	ret = open_candev(ndev);
	if (ret)
		return ret;

	ret = request_irq(ndev->irq, aspeed_can_interrupt,
			  0, ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "failed to request IRQ: %d\n", ret);
		goto err_candev;
	}

	napi_enable(&priv->napi);

	ret = aspeed_can_chip_start(ndev);
	if (ret) {
		netdev_err(ndev, "chip_start failed: %d\n", ret);
		goto err_napi;
	}

	netif_start_queue(ndev);

	return 0;

err_napi:
	napi_disable(&priv->napi);
	free_irq(ndev->irq, ndev);
err_candev:
	close_candev(ndev);
	return ret;
}

static int aspeed_can_close(struct net_device *ndev)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	int i;

	netif_stop_queue(ndev);
	aspeed_can_write(priv, CAN_INTE, 0);
	napi_disable(&priv->napi);
	aspeed_can_chip_stop(ndev);
	free_irq(ndev->irq, ndev);

	for (i = 0; i < STB_SLOTS; i++)
		can_free_echo_skb(ndev, i, NULL);

	close_candev(ndev);

	return 0;
}

static int aspeed_can_get_berr_counter(const struct net_device *ndev,
				       struct can_berr_counter *bec)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	u32 cnt = aspeed_can_read(priv, CAN_LIMIT);

	bec->rxerr = FIELD_GET(RECNT_MASK, cnt);
	bec->txerr = FIELD_GET(TECNT_MASK, cnt);

	return 0;
}

static void aspeed_can_tx_timeout(struct net_device *ndev,
				  unsigned int txqueue)
{
	struct aspeed_can_priv *priv = netdev_priv(ndev);
	unsigned long flags;
	int i;

	netdev_warn(ndev, "TX timeout\n");

	aspeed_can_write(priv, CAN_INTE, 0);

	spin_lock_irqsave(&priv->tx_lock, flags);
	for (i = 0; i < STB_SLOTS; i++)
		can_free_echo_skb(ndev, i, NULL);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (!aspeed_can_chip_start(ndev))
		netif_wake_queue(ndev);
	else
		netdev_err(ndev, "failed to recover from TX timeout\n");
}

static const struct net_device_ops aspeed_can_netdev_ops = {
	.ndo_open	= aspeed_can_open,
	.ndo_stop	= aspeed_can_close,
	.ndo_start_xmit	= aspeed_can_start_xmit,
	.ndo_tx_timeout	= aspeed_can_tx_timeout,
};

static const struct ethtool_ops aspeed_can_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static const struct of_device_id aspeed_can_of_match[] = {
	{ .compatible = "aspeed,ast2700-canfd" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, aspeed_can_of_match);

static int aspeed_can_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct aspeed_can_priv *priv;
	u32 can_clk;
	int ret;

	ndev = alloc_candev(sizeof(*priv), ECHO_SKB_SLOTS);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);

	priv->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg_base)) {
		ret = PTR_ERR(priv->reg_base);
		goto err_free;
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(priv->clk),
				    "missing clock\n");
		goto err_free;
	}

	can_clk = clk_get_rate(priv->clk);
	if (!can_clk) {
		ret = dev_err_probe(&pdev->dev, -EINVAL,
				    "invalid clock rate\n");
		goto err_free;
	}

	priv->reset = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		ret = PTR_ERR(priv->reset);
		goto err_free;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to enable clock\n");
		goto err_free;
	}

	ret = reset_control_deassert(priv->reset);
	if (ret)
		goto err_clk;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto err_reset;
	ndev->irq = ret;

	priv->can.clock.freq = can_clk;
	priv->can.bittiming_const = &aspeed_can_bittiming_const;
	priv->can.fd.data_bittiming_const = &aspeed_canfd_bittiming_const;
	priv->can.fd.tdc_const = &aspeed_canfd_tdc_const;
	priv->can.do_set_bittiming = aspeed_can_do_set_bittiming;
	priv->can.fd.do_set_data_bittiming = aspeed_can_do_set_data_bittiming;
	priv->can.do_set_mode = aspeed_can_do_set_mode;
	priv->can.do_get_berr_counter = aspeed_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				       CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_BERR_REPORTING |
				       CAN_CTRLMODE_FD |
				       CAN_CTRLMODE_TDC_AUTO |
				       CAN_CTRLMODE_CC_LEN8_DLC |
				       CAN_CTRLMODE_ONE_SHOT;

	spin_lock_init(&priv->tx_lock);

	aspeed_can_stb_init(priv);

	ndev->flags |= IFF_ECHO;
	ndev->netdev_ops = &aspeed_can_netdev_ops;
	ndev->ethtool_ops = &aspeed_can_ethtool_ops;
	ndev->watchdog_timeo = msecs_to_jiffies(1000);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	netif_napi_add_weight(ndev, &priv->napi, aspeed_can_rx_poll,
			      ASPEED_CAN_NAPI_WEIGHT);

	ret = register_candev(ndev);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "register_candev failed\n");
		goto err_reset;
	}

	netdev_dbg(ndev, "reg_base=%p irq=%d clk=%u Hz\n",
		   priv->reg_base, ndev->irq, priv->can.clock.freq);

	return 0;

err_reset:
	reset_control_assert(priv->reset);
err_clk:
	clk_disable_unprepare(priv->clk);
err_free:
	free_candev(ndev);
	return ret;
}

static void aspeed_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct aspeed_can_priv *priv = netdev_priv(ndev);

	unregister_candev(ndev);
	netif_napi_del(&priv->napi);
	reset_control_assert(priv->reset);
	clk_disable_unprepare(priv->clk);
	free_candev(ndev);
}

static struct platform_driver aspeed_can_driver = {
	.probe = aspeed_can_probe,
	.remove = aspeed_can_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_can_of_match,
	},
};

module_platform_driver(aspeed_can_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chin-Ting Kuo <chin-ting_kuo@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED CAN controller driver");
