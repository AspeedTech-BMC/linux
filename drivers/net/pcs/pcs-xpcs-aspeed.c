// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Aspeed Technology Inc. */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/pcs/pcs-xpcs.h>

#include "pcs-xpcs.h"

/* VR_XS_PMA_MMD - multi-protocol 12G/16G/25G PMA */
#define ASPEED_PMA_RX_LSTS		0x8020
#define ASPEED_PMA_RX_VALID_0		BIT(12)
#define ASPEED_PMA_TX_GENCTRL1		0x8031
#define ASPEED_PMA_VBOOST_EN_0		BIT(4)
#define ASPEED_PMA_TX_GENCTRL2		0x8032
#define ASPEED_PMA_TX0_WIDTH_MASK	GENMASK(9, 8)
#define ASPEED_PMA_WIDTH_10_BIT		0x1
#define ASPEED_PMA_TX_RATE_CTRL		0x8034
#define ASPEED_PMA_TX0_RATE_MASK	GENMASK(2, 0)
#define ASPEED_PMA_BAUD_8		0x3
#define ASPEED_PMA_TX_EQ_CTRL0		0x8036
#define ASPEED_PMA_TX_EQ_MAIN_MASK	GENMASK(13, 8)
#define ASPEED_PMA_TX_EQ_CTRL1		0x8037
#define ASPEED_PMA_TX_EQ_POST_MASK	GENMASK(5, 0)
#define ASPEED_PMA_RX_GENCTRL2		0x8052
#define ASPEED_PMA_RX0_WIDTH_MASK	GENMASK(9, 8)
#define ASPEED_PMA_RX_RATE_CTRL		0x8054
#define ASPEED_PMA_RX0_RATE_MASK	GENMASK(1, 0)
#define ASPEED_PMA_RX_EQ_CTRL0		0x8058
#define ASPEED_PMA_CTLE_BOOST_0_MASK	GENMASK(4, 0)
#define ASPEED_PMA_CTLE_POLE_0_MASK	GENMASK(6, 5)
#define ASPEED_PMA_RX_EQ_CTRL4		0x805c
#define ASPEED_PMA_CONT_ADAPT_0	BIT(0)
#define ASPEED_PMA_RX_AD_REQ		BIT(12)
#define ASPEED_PMA_RX_EQ_CTRL5		0x805d
#define ASPEED_PMA_RX_GENCTRL4		0x8068
#define ASPEED_PMA_RX_DFE_BYP_0	BIT(8)
#define ASPEED_PMA_RX_MISC_CTRL0	0x8069
#define ASPEED_PMA_RX0_MISC_MASK	GENMASK(7, 0)
#define ASPEED_PMA_RX_IQ_CTRL0		0x806b
#define ASPEED_PMA_RX0_DELTA_IQ_MASK	GENMASK(11, 8)
#define ASPEED_PMA_MPLLA_CTRL0		0x8071
#define ASPEED_PMA_MPLLA_MULTIPLIER_MASK	GENMASK(7, 0)
#define ASPEED_PMA_MPLLA_CTRL2		0x8073
#define ASPEED_PMA_MPLLA_DIV10_CLK_EN	BIT(9)
#define ASPEED_PMA_MPLLA_DIV16P5_CLK_EN	BIT(10)
#define ASPEED_PMA_MPLLA_CTRL3		0x8077
#define ASPEED_PMA_VCO_CAL_LD0		0x8092
#define ASPEED_PMA_VCO_LD_VAL_0_MASK	GENMASK(12, 0)
#define ASPEED_PMA_VCO_CAL_REF0	0x8096
#define ASPEED_PMA_VCO_REF_LD_0_MASK	GENMASK(6, 0)
#define ASPEED_PMA_MISC_STS		0x8098
#define ASPEED_PMA_RX_ADPT_ACK		BIT(12)
#define ASPEED_PMA_SRAM		0x809b
#define ASPEED_PMA_INIT_DN		BIT(0)
#define ASPEED_PMA_EXT_LD_DN		BIT(1)

/* VR_XS_PCS_MMD */
#define ASPEED_PCS_DEBUG_CTRL		0x8005
#define ASPEED_PCS_SUPRESS_LOS_DET	BIT(4)
#define ASPEED_PCS_RX_DT_EN_CTL	BIT(6)

/* SR_XS_PCS_MMD */
#define ASPEED_SR_XS_PCS_CTRL2		0x0007
#define ASPEED_PCS_TYPE_SEL_MASK	GENMASK(3, 0)
#define ASPEED_SEL_10GBASE_X		0x1

static int aspeed_xpcs_vr_reset(struct dw_xpcs *xpcs)
{
	int ret, val;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_SRAM,
			  ASPEED_PMA_EXT_LD_DN, 0);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PCS, DW_VENDOR | DW_VR_XS_PCS_DIG_CTRL1,
			  DW_VR_RST, DW_VR_RST);
	if (ret < 0)
		return ret;

	ret = read_poll_timeout(xpcs_read, val, val & ASPEED_PMA_INIT_DN,
				1000, 100000, true,
				xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_SRAM);
	if (val < 0)
		ret = val;
	if (ret < 0) {
		dev_err(&xpcs->mdiodev->dev, "%s: PMA init timeout\n", __func__);
		return ret;
	}

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_SRAM,
			  ASPEED_PMA_EXT_LD_DN, ASPEED_PMA_EXT_LD_DN);
	if (ret < 0)
		return ret;

	ret = read_poll_timeout(xpcs_read, val, !(val & DW_VR_RST),
				1000, 100000, true,
				xpcs, MDIO_MMD_PCS, DW_VENDOR | DW_VR_XS_PCS_DIG_CTRL1);
	if (val < 0)
		ret = val;
	if (ret < 0) {
		dev_err(&xpcs->mdiodev->dev, "%s: VR reset timeout\n", __func__);
		return ret;
	}

	return 0;
}

static int aspeed_xpcs_wait_reset_done(struct dw_xpcs *xpcs)
{
	int ret, val;

	ret = read_poll_timeout(xpcs_read, val, !(val & BMCR_RESET),
				1000, 100000, true,
				xpcs, MDIO_MMD_PCS, MII_BMCR);
	if (val < 0)
		ret = val;
	if (ret < 0)
		dev_err(&xpcs->mdiodev->dev, "%s: reset done timeout\n", __func__);

	return ret;
}

static int aspeed_xpcs_wait_rx_valid(struct dw_xpcs *xpcs)
{
	int ret, val;

	ret = read_poll_timeout(xpcs_read, val, val & ASPEED_PMA_RX_VALID_0,
				1000, 100000, true,
				xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_LSTS);
	if (val < 0)
		ret = val;
	if (ret < 0)
		dev_err(&xpcs->mdiodev->dev, "%s: RX invalid\n", __func__);

	return ret;
}

static int aspeed_xpcs_pma_config_common(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_write(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_MPLLA_CTRL3, 0xa03e);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL0,
			  ASPEED_PMA_CTLE_BOOST_0_MASK | ASPEED_PMA_CTLE_POLE_0_MASK,
			  FIELD_PREP(ASPEED_PMA_CTLE_BOOST_0_MASK, 0xa) |
			  FIELD_PREP(ASPEED_PMA_CTLE_POLE_0_MASK, 0x1));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_MISC_CTRL0,
			  ASPEED_PMA_RX0_MISC_MASK,
			  FIELD_PREP(ASPEED_PMA_RX0_MISC_MASK, 0x2));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_IQ_CTRL0,
			  ASPEED_PMA_RX0_DELTA_IQ_MASK,
			  FIELD_PREP(ASPEED_PMA_RX0_DELTA_IQ_MASK, 0x3));
	if (ret < 0)
		return ret;

	return 0;
}

static int aspeed_xpcs_pma_finish_common(struct dw_xpcs *xpcs)
{
	int ret, val;

	ret = aspeed_xpcs_vr_reset(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PCS, ASPEED_PCS_DEBUG_CTRL,
			  ASPEED_PCS_SUPRESS_LOS_DET | ASPEED_PCS_RX_DT_EN_CTL,
			  ASPEED_PCS_SUPRESS_LOS_DET | ASPEED_PCS_RX_DT_EN_CTL);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_wait_reset_done(xpcs);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_wait_rx_valid(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL4,
			  ASPEED_PMA_RX_AD_REQ, ASPEED_PMA_RX_AD_REQ);
	if (ret < 0)
		return ret;

	ret = read_poll_timeout(xpcs_read, val, val & ASPEED_PMA_RX_ADPT_ACK,
				1000, 500000, true,
				xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_MISC_STS);
	if (val < 0)
		ret = val;
	if (ret < 0) {
		dev_err(&xpcs->mdiodev->dev, "%s: EQ ACK failed\n", __func__);
		return ret;
	}

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL4,
			  ASPEED_PMA_RX_AD_REQ, 0);
	if (ret < 0)
		return ret;

	return 0;
}

int aspeed_xpcs_10gbaser_pma_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = aspeed_xpcs_pma_config_common(xpcs);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_pma_finish_common(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL0,
			  ASPEED_PMA_TX_EQ_MAIN_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_MAIN_MASK, 0x21));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL1,
			  ASPEED_PMA_TX_EQ_POST_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_POST_MASK, 0x1c));
	if (ret < 0)
		return ret;

	return ret;
}

int aspeed_xpcs_usxgmii_pma_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = aspeed_xpcs_pma_config_common(xpcs);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_pma_finish_common(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL0,
			  ASPEED_PMA_TX_EQ_MAIN_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_MAIN_MASK, 0x21));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL1,
			  ASPEED_PMA_TX_EQ_POST_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_POST_MASK, 0x1c));
	if (ret < 0)
		return ret;

	return ret;
}

int aspeed_xpcs_sgmii_pma_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_MPLLA_CTRL0,
			  ASPEED_PMA_MPLLA_MULTIPLIER_MASK,
			  FIELD_PREP(ASPEED_PMA_MPLLA_MULTIPLIER_MASK, 0x20));
	if (ret < 0)
		return ret;

	ret = xpcs_write(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_MPLLA_CTRL3, 0xa03e);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_VCO_CAL_LD0,
			  ASPEED_PMA_VCO_LD_VAL_0_MASK,
			  FIELD_PREP(ASPEED_PMA_VCO_LD_VAL_0_MASK, 0x540));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_VCO_CAL_REF0,
			  ASPEED_PMA_VCO_REF_LD_0_MASK,
			  FIELD_PREP(ASPEED_PMA_VCO_REF_LD_0_MASK, 0x2a));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL4,
			  ASPEED_PMA_CONT_ADAPT_0, 0);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_RATE_CTRL,
			  ASPEED_PMA_TX0_RATE_MASK,
			  FIELD_PREP(ASPEED_PMA_TX0_RATE_MASK, ASPEED_PMA_BAUD_8));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_RATE_CTRL,
			  ASPEED_PMA_RX0_RATE_MASK,
			  FIELD_PREP(ASPEED_PMA_RX0_RATE_MASK, ASPEED_PMA_BAUD_8));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_GENCTRL2,
			  ASPEED_PMA_TX0_WIDTH_MASK,
			  FIELD_PREP(ASPEED_PMA_TX0_WIDTH_MASK, ASPEED_PMA_WIDTH_10_BIT));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_GENCTRL2,
			  ASPEED_PMA_RX0_WIDTH_MASK,
			  FIELD_PREP(ASPEED_PMA_RX0_WIDTH_MASK, ASPEED_PMA_WIDTH_10_BIT));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_MPLLA_CTRL2,
			  ASPEED_PMA_MPLLA_DIV10_CLK_EN | ASPEED_PMA_MPLLA_DIV16P5_CLK_EN,
			  ASPEED_PMA_MPLLA_DIV10_CLK_EN);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_GENCTRL1,
			  ASPEED_PMA_VBOOST_EN_0, 0);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL0,
			  ASPEED_PMA_CTLE_BOOST_0_MASK | ASPEED_PMA_CTLE_POLE_0_MASK,
			  FIELD_PREP(ASPEED_PMA_CTLE_BOOST_0_MASK, 0x6) |
			  FIELD_PREP(ASPEED_PMA_CTLE_POLE_0_MASK, 0));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_MISC_CTRL0,
			  ASPEED_PMA_RX0_MISC_MASK,
			  FIELD_PREP(ASPEED_PMA_RX0_MISC_MASK, 0x6));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_GENCTRL4,
			  ASPEED_PMA_RX_DFE_BYP_0, ASPEED_PMA_RX_DFE_BYP_0);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_IQ_CTRL0,
			  ASPEED_PMA_RX0_DELTA_IQ_MASK, 0);
	if (ret < 0)
		return ret;

	ret = xpcs_write(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_RX_EQ_CTRL5, 0);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PCS, ASPEED_SR_XS_PCS_CTRL2,
			  ASPEED_PCS_TYPE_SEL_MASK,
			  FIELD_PREP(ASPEED_PCS_TYPE_SEL_MASK, ASPEED_SEL_10GBASE_X));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PCS, MII_BMCR, BMCR_SPEED100, 0);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_vr_reset(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PCS, ASPEED_PCS_DEBUG_CTRL,
			  ASPEED_PCS_SUPRESS_LOS_DET | ASPEED_PCS_RX_DT_EN_CTL, 0);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_wait_reset_done(xpcs);
	if (ret < 0)
		return ret;

	ret = aspeed_xpcs_wait_rx_valid(xpcs);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL0,
			  ASPEED_PMA_TX_EQ_MAIN_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_MAIN_MASK, 0x28));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_PMAPMD, ASPEED_PMA_TX_EQ_CTRL1,
			  ASPEED_PMA_TX_EQ_POST_MASK,
			  FIELD_PREP(ASPEED_PMA_TX_EQ_POST_MASK, 0));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_VEND2, DW_VR_MII_AN_CTRL,
			  DW_VR_MII_PCS_MODE_MASK,
			  FIELD_PREP(DW_VR_MII_PCS_MODE_MASK,
				     DW_VR_MII_PCS_MODE_C37_SGMII));
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_VEND2, DW_VR_MII_DIG_CTRL1,
			  DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW,
			  DW_VR_MII_DIG_CTRL1_MAC_AUTO_SW);
	if (ret < 0)
		return ret;

	ret = xpcs_modify(xpcs, MDIO_MMD_VEND2, MII_BMCR,
			  BMCR_ANENABLE, BMCR_ANENABLE);
	if (ret < 0)
		return ret;

	return ret;
}
