// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright ASPEED Technology

#include "linux/dev_printk.h"
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/delay.h>

#include "aspeed-ltpi.h"

#define LTPI_AUTO_CAP_LOW			0x24
#define   LTPI_I2C_IO_FRAME_EN			GENMASK(29, 24)
#define LTPI_AUTO_CAP_HIGH			0x28
#define   LTPI_UART_IO_FRAME_EN			GENMASK(14, 13)

#define LTPI_LINK_CONTROLL			0x80
#define   LTPI_AUTO_CONFIG			BIT(10)

#define LTPI_INTR_STATUS			0x100
#define LTPI_INTR_EN				0x104
#define   LTPI_INTR_EN_OP_LINK_LOST		BIT(4)
#define LTPI_LINK_MANAGE_ST			0x108
#define   LTPI_LINK_PARTNER_FLAG		BIT(24)

#define LTPI_MANUAL_CAP_LOW			0x118
#define LTPI_MANUAL_CAP_HIGH			0x11c

#define LTPI_I2C_TIMING_0			0x134
#define LTPI_I2C_TIMING_1			0x138

#define LTPI_I2C_100K_0			0x3535352f
#define LTPI_I2C_100K_1			0x09353535

#define LTPI_I2C_400K_0			0x06060d06
#define LTPI_I2C_400K_1			0x090d0a06

#define SCU_IO_PINS_TRAP1			0x10
#define SCU_IO_PINS_TRAP1_CLEAR			0x14
#define   SCU_IO_PINS_TRAP_LTPI			GENMASK(2, 0)
#define SCU_IO_OTP_TRAP1			0xa00
#define SCU_IO_OTP_TRAP1_CLEAR			0xa04
#define SCU_IO_OTP_TRAP2			0xa20
#define SCU_IO_OTP_TRAP2_CLEAR			0xa24

/* SCU registers for PLL control */
#define SCU1_HPLL_1				0x300
#define SCU1_HPLL_2				0x304
#define SCU1_APLL_1				0x310
#define SCU1_APLL_2				0x314
#define SCU1_DPLL_1				0x320
#define SCU1_DPLL_2				0x324
#define SCU1_L0PLL_1				0x340
#define SCU1_L0PLL_2				0x344
#define SCU1_L1PLL_1				0x350
#define SCU1_L1PLL_2				0x354

#define PLL_REG1_RESET				BIT(25)
#define PLL_REG1_BYPASS				BIT(24)
#define PLL_REG1_DIS				BIT(23)
#define PLL_REG1_P				GENMASK(22, 19)
#define PLL_REG1_N				GENMASK(18, 13)
#define PLL_REG1_M				GENMASK(12, 0)

#define PLL_REG2_LOCK				BIT(31)
#define PLL_REG2_BWADJ				GENMASK(11, 0)

#define MAX_I2C_IN_LTPI				6
#define MAX_UART_IN_LTPI			2

#define ADVERTISE_TIMEOUT_US			105000 /* 105 ms */

enum chip_version {
	AST2700,
	AST1700,
};

struct ltpi_clk_info {
	s16 freq; /* clock frequency in MHz*/
	s16 clk_sel; /* clock selection */
};

static const struct ltpi_clk_info ltpi_clk_lookup_sdr[13] = {
	{ 25, REG_LTPI_PLL_25M },
	{ 50, REG_LTPI_PLL_LPLL },
	{ 75, REG_LTPI_PLL_LPLL },
	{ 100, REG_LTPI_PLL_LPLL },
	{ 150, REG_LTPI_PLL_LPLL },
	{ 200, REG_LTPI_PLL_LPLL },
	{ 250, REG_LTPI_PLL_LPLL },
	{ 300, REG_LTPI_PLL_LPLL },
	{ 400, REG_LTPI_PLL_LPLL },
	{ 600, REG_LTPI_PLL_LPLL },
	{ -1, -1 },
	{ -1, -1 },
	{ 500, REG_LTPI_PLL_LPLL }
};

static const struct ltpi_clk_info ltpi_clk_lookup_ddr[13] = {
	{ 50, REG_LTPI_PLL_LPLL },
	{ 100, REG_LTPI_PLL_LPLL },
	{ 150, REG_LTPI_PLL_LPLL },
	{ 200, REG_LTPI_PLL_LPLL },
	{ 300, REG_LTPI_PLL_LPLL },
	{ 400, REG_LTPI_PLL_LPLL },
	{ 500, REG_LTPI_PLL_LPLL },
	{ 600, REG_LTPI_PLL_LPLL },
	{ 800, REG_LTPI_PLL_LPLL },
	{ 1200, REG_LTPI_PLL_LPLL },
	{ -1, -1 },
	{ -1, -1 },
	{ 1000, REG_LTPI_PLL_LPLL }
};

#define MHZ(x)			((x) * 1000000)
#define NUM_PLL_PARAM		13
#define REG_N_M_P(n, m, p)	((((n) - 1) << 13) | ((m) - 1) | (((p) - 1) << 19))
#define REG_BWADJ(bwadj)	((bwadj) - 1)

struct pll_param {
	int freq;
	u32 n_m_p;
	u16 bwadj;
};

static const struct pll_param pll_param_lookup[NUM_PLL_PARAM] = {
	{ .freq = MHZ(50), .n_m_p = REG_N_M_P(1, 32, 16), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(75), .n_m_p = REG_N_M_P(1, 48, 16), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(100), .n_m_p = REG_N_M_P(1, 56, 14), .bwadj = REG_BWADJ(28) },
	{ .freq = MHZ(150), .n_m_p = REG_N_M_P(1, 60, 10), .bwadj = REG_BWADJ(30) },
	{ .freq = MHZ(200), .n_m_p = REG_N_M_P(1, 48, 6), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(250), .n_m_p = REG_N_M_P(1, 60, 6), .bwadj = REG_BWADJ(30) },
	{ .freq = MHZ(300), .n_m_p = REG_N_M_P(1, 48, 4), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(400), .n_m_p = REG_N_M_P(1, 32, 2), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(500), .n_m_p = REG_N_M_P(1, 40, 2), .bwadj = REG_BWADJ(20) },
	{ .freq = MHZ(600), .n_m_p = REG_N_M_P(1, 48, 2), .bwadj = REG_BWADJ(24) },
	{ .freq = MHZ(800), .n_m_p = REG_N_M_P(1, 32, 1), .bwadj = REG_BWADJ(16) },
	{ .freq = MHZ(1000), .n_m_p = REG_N_M_P(1, 40, 1), .bwadj = REG_BWADJ(20) },
	{ .freq = MHZ(1200), .n_m_p = REG_N_M_P(1, 48, 1), .bwadj = REG_BWADJ(24) },
};

struct pll_info {
	u32 reg_offset0;
	u32 reg_offset1;
};

static const struct pll_info scu_pll_info[] = {
	[0] = { .reg_offset0 = SCU1_L0PLL_1, .reg_offset1 = SCU1_L0PLL_2 },
	[1] = { .reg_offset0 = SCU1_L1PLL_1, .reg_offset1 = SCU1_L1PLL_2 },
};

struct aspeed_ltpi_priv {
	struct device *dev;
	void __iomem *regs;
	void __iomem *phy_regs;
	void __iomem *top_regs;
	struct clk *ltpi_clk;
	struct clk *ltpi_phyclk;
	struct reset_control *ltpi_rst;
	struct regmap *scu;
	u32 version;
	u32 i2c_tunneling;
	u32 i2c_timing_0;
	u32 i2c_timing_1;
	u32 uart_tunneling;
	int index;

	/* Training parameters */
	u16 phy_speed_cap;
	bool otp_ddr_dis;
	int disable_auto_downshift;
	int crc_format;
	int io_driving;
	int clk_inverse;
	int link_speed_frm_rx_cnt;
	int ad_timeout;
	u64 op_timeout;
	u64 t_link_detect;
};

static int ltpi_get_link_partner(struct aspeed_ltpi_priv *ltpi)
{
	u32 reg = readl(ltpi->regs + LTPI_LINK_MNG_ST);

	return FIELD_GET(REG_LTPI_LINK_PARTNER_FLAG, reg);
}

static void ltpi_dump(struct aspeed_ltpi_priv *ltpi)
{
	int i;

	dev_warn(ltpi->dev, "\nDumping LTPI registers:\n");
	for (i = 0; i < 0x250; i += 0x10) {
		dev_warn(ltpi->dev, "control[%08x]: %08x %08x %08x %08x\n", i,
			 readl(ltpi->regs + i), readl(ltpi->regs + i + 4),
			 readl(ltpi->regs + i + 8), readl(ltpi->regs + i + 12));
	}

	for (i = 0; i < 0x30; i += 0x10) {
		dev_warn(ltpi->dev, "PHY[%08x]: %08x %08x %08x %08x\n", i,
			 readl(ltpi->phy_regs + i),
			 readl(ltpi->phy_regs + i + 4),
			 readl(ltpi->phy_regs + i + 8),
			 readl(ltpi->phy_regs + i + 12));
	}
}

static irqreturn_t aspeed_ltpi_irq_handler(int irq, void *dev_id)
{
	struct aspeed_ltpi_priv *priv = dev_id;
	u32 status = readl(priv->regs + LTPI_INTR_STATUS);

	if (status & LTPI_INTR_EN_OP_LINK_LOST) {
		writel(0, priv->regs + LTPI_INTR_EN);
		writel(status, priv->regs + LTPI_INTR_STATUS);
		if (ltpi_get_link_partner(priv))
			panic("LTPI link lost!\n");
		/* Will not return */
		else
			dev_err(priv->dev, "LTPI link lost!\n");
	}

	writel(status, priv->regs + LTPI_INTR_STATUS);

	return IRQ_HANDLED;
}

static int aspeed_ltpi_init_mux(struct aspeed_ltpi_priv *priv)
{
	u32 reg, i2c_en, uart_en, i;

	reg = readl(priv->regs + LTPI_AUTO_CAP_LOW);

	i2c_en = FIELD_GET(LTPI_I2C_IO_FRAME_EN, reg);
	i2c_en &= priv->i2c_tunneling;

	reg &= ~LTPI_I2C_IO_FRAME_EN;
	reg |= FIELD_PREP(LTPI_I2C_IO_FRAME_EN, i2c_en);
	writel(reg, priv->regs + LTPI_MANUAL_CAP_LOW);

	reg = readl(priv->regs + LTPI_AUTO_CAP_HIGH);

	uart_en = FIELD_GET(LTPI_UART_IO_FRAME_EN, reg);
	uart_en &= priv->uart_tunneling;

	reg &= ~LTPI_UART_IO_FRAME_EN;
	reg |= FIELD_PREP(LTPI_UART_IO_FRAME_EN, uart_en);

	writel(reg, priv->regs + LTPI_MANUAL_CAP_HIGH);

	/* Apply LTPI manual configuration */
	reg = readl(priv->regs + LTPI_LINK_CONTROLL);
	reg &= ~LTPI_AUTO_CONFIG;
	writel(reg, priv->regs + LTPI_LINK_CONTROLL);

	/* Set the AST1700 i2c ac-timing */
	if (priv->version == AST1700) {
		/* Apply i2c timing with i2c tunneling setting */
		for (i = 0; i < MAX_I2C_IN_LTPI; i++) {
			if ((priv->i2c_tunneling >> i) & 0x1) {
				writel(priv->i2c_timing_0,
				       priv->regs + LTPI_I2C_TIMING_0 +
					       (0x8 * i));
				writel(priv->i2c_timing_1,
				       priv->regs + LTPI_I2C_TIMING_1 +
					       (0x8 * i));
			}
		}
	}

	return 0;
}

static int clz16(u16 x)
{
	int n = 0;

	if (x == 0)
		return 16;

	if (x <= 0x00ff) {
		n += 8;
		x <<= 8;
	}
	if (x <= 0x0fff) {
		n += 4;
		x <<= 4;
	}
	if (x <= 0x3fff) {
		n += 2;
		x <<= 2;
	}
	if (x <= 0x7fff)
		n += 1;

	return n;
}

static u16 find_max_speed(u16 cap)
{
	return 15 - clz16(cap & ~LTPI_SP_CAP_DDR);
}

static void ltpi_phy_unlock(struct aspeed_ltpi_priv *ltpi)
{
	writel(LTPI_PROT_KEY_UNLOCK, ltpi->phy_regs + LTPI_PROT_KEY);
}

static void ltpi_enable_rx_bias(struct aspeed_ltpi_priv *ltpi)
{
	u32 val = readl(ltpi->top_regs + LTPI_LVDS_RX_CTRL);

	val |= (REG_LTPI_LVDS_RX1_BIAS_EN | REG_LTPI_LVDS_RX0_BIAS_EN);
	writel(val, ltpi->top_regs + LTPI_LVDS_RX_CTRL);
	udelay(1);
}

static int ltpi_phy_get_mode(struct aspeed_ltpi_priv *ltpi)
{
	u32 reg = readl(ltpi->phy_regs + LTPI_PHY_CTRL);

	return FIELD_GET(REG_LTPI_PHY_MODE, reg);
}

static int ltpi_phy_set_mode(struct aspeed_ltpi_priv *ltpi, int mode)
{
	u32 reg;

	if (mode < 0 || mode > LTPI_PHY_MODE_CDR_HI_SP) {
		dev_err(ltpi->dev, "%s: invalid mode %d\n", __func__, mode);
		return -1;
	}

	reg = readl(ltpi->phy_regs + LTPI_PHY_CTRL);
	reg &= ~REG_LTPI_PHY_MODE;
	reg |= mode;
	writel(reg, ltpi->phy_regs + LTPI_PHY_CTRL);

	return 0;
}

static int ltpi_phy_set_clksel(struct aspeed_ltpi_priv *ltpi, int clksel,
			       bool is_op_clk)
{
	u32 reg;

#define RX_CLK_INVERSE BIT(1)
#define TX_CLK_INVERSE BIT(0)

	reg = readl(ltpi->phy_regs + LTPI_PLL_CTRL);
	reg &= ~(REG_LTPI_PLL_SELECT | REG_LTPI_PLL_SET |
		 REG_LTPI_RX_PHY_CLK_INV | REG_LTPI_TX_PHY_CLK_INV);
	reg |= FIELD_PREP(REG_LTPI_PLL_SELECT, clksel);

	if (ltpi->clk_inverse & RX_CLK_INVERSE)
		reg |= REG_LTPI_RX_PHY_CLK_INV;

	if (ltpi->clk_inverse & TX_CLK_INVERSE)
		reg |= REG_LTPI_TX_PHY_CLK_INV;

	if (is_op_clk)
		reg |= REG_LTPI_PLL_SET;

	writel(reg, ltpi->phy_regs + LTPI_PLL_CTRL);

	return 0;
}

static void ltpi_set_crc_format(struct aspeed_ltpi_priv *ltpi, int crc_fmt)
{
	u32 val = readl(ltpi->regs + LTPI_CRC_OPTION);

	val &= ~(REG_LTPI_SW_CRC_OUT_ML_FIRST | REG_LTPI_SW_CRC_IN_LSB_FIRST);
	if (crc_fmt)
		val |= REG_LTPI_SW_CRC_OUT_ML_FIRST |
		       REG_LTPI_SW_CRC_IN_LSB_FIRST;

	writel(val, ltpi->regs + LTPI_CRC_OPTION);
}

static int ltpi_reset(struct aspeed_ltpi_priv *ltpi)
{
	/* Using reset controller */
	reset_control_assert(ltpi->ltpi_rst);
	udelay(1);
	/* Assuming clk gate is handled by clock framework or enabled */
	/* U-Boot does ungate here */
	if (ltpi->ltpi_clk)
		clk_prepare_enable(ltpi->ltpi_clk);

	reset_control_deassert(ltpi->ltpi_rst);

	ltpi_phy_unlock(ltpi);

	return 0;
}

static u32 ltpi_get_link_mng_state(struct aspeed_ltpi_priv *ltpi)
{
	return FIELD_GET(REG_LTPI_LINK_MNG_ST,
			 readl(ltpi->regs + LTPI_LINK_MNG_ST));
}

static int ltpi_poll_link_mng_state(struct aspeed_ltpi_priv *ltpi,
				    u32 expected, u32 unexpected,
				    int timeout_us)
{
	u32 state;
	int ret;

	ret = readl_poll_timeout(ltpi->regs + LTPI_LINK_MNG_ST, state,
				 (FIELD_GET(REG_LTPI_LINK_MNG_ST, state) == expected) ||
					 (FIELD_GET(REG_LTPI_LINK_MNG_ST, state) == unexpected),
				 100, timeout_us);

	if (FIELD_GET(REG_LTPI_LINK_MNG_ST, state) == unexpected)
		return -ENOLINK;

	if (ret)
		return -ETIMEDOUT;

	return 0;
}

static int ltpi_wait_state_pll_set(struct aspeed_ltpi_priv *ltpi,
				   int timeout_us)
{
	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_WAIT_PLL_SET, -1,
					timeout_us);
}

static int ltpi_wait_state_op(struct aspeed_ltpi_priv *ltpi)
{
	u32 val = readl(ltpi->regs + LTPI_LINK_ST);

	val |= (REG_LTPI_CON_ACC_TO_ERR | REG_LTPI_FRM_CRC_ERR |
		REG_LTPI_LINK_LOST_ERR);
	writel(val, ltpi->regs + LTPI_LINK_ST);

	return ltpi_poll_link_mng_state(ltpi, LTPI_LINK_MNG_ST_OP,
					LTPI_LINK_MNG_ST_DETECT_ALIGN,
					ltpi->ad_timeout);
}

static int ltpi_set_lvds_io_driving(struct aspeed_ltpi_priv *ltpi, int driving)
{
	u32 val;

	val = readl(ltpi->top_regs + LTPI_SW_FORCE_EN);
	val |= REG_LTPI_SW_FORCE_LVDS_TX_DS_EN;
	writel(val, ltpi->top_regs + LTPI_SW_FORCE_EN);

	val = readl(ltpi->top_regs + LTPI_LVDS_TX_CTRL);
	val &= ~(REG_LTPI_LVDS_TX1_DS1 | REG_LTPI_LVDS_TX1_DS0 |
		 REG_LTPI_LVDS_TX0_DS1 | REG_LTPI_LVDS_TX0_DS0);

	/* tx1: clk, tx0: data */
	if (driving & BIT(1))
		val |= (REG_LTPI_LVDS_TX1_DS1 | REG_LTPI_LVDS_TX0_DS1);

	if (driving & BIT(0))
		val |= (REG_LTPI_LVDS_TX1_DS0 | REG_LTPI_LVDS_TX0_DS0);

	writel(val, ltpi->top_regs + LTPI_LVDS_TX_CTRL);

	return 0;
}

static int ltpi_set_local_speed_cap(struct aspeed_ltpi_priv *ltpi,
				    u32 speed_cap)
{
	u32 reg;

	/* only set bits that Aspeed SOC supported */
	speed_cap &= LTPI_SP_CAP_ASPEED_SUPPORTED;
	if (ltpi->otp_ddr_dis)
		speed_cap &= ~LTPI_SP_CAP_DDR;

	reg = readl(ltpi->regs + LTPI_CAP_LOCAL);
	reg &= ~REG_LTPI_SP_CAP_LOCAL;
	reg |= FIELD_PREP(REG_LTPI_SP_CAP_LOCAL, speed_cap);
	writel(reg, ltpi->regs + LTPI_CAP_LOCAL);

	return 0;
}

static void ltpi_do_link_training(struct aspeed_ltpi_priv *ltpi)
{
	u32 val;

	/* Reset the PHY to PHY_MODE_OFF */
	ltpi_reset(ltpi);

	ltpi_enable_rx_bias(ltpi);
	ltpi_set_local_speed_cap(ltpi, ltpi->phy_speed_cap);
	ltpi_set_lvds_io_driving(ltpi, ltpi->io_driving);
	ltpi_set_crc_format(ltpi, ltpi->crc_format);

	/*
	 * Configure the LINK_SPEED frame count to be received before
	 * entering AD. This configuraiton only effects the SCM LTPI.
	 */
	val = readl(ltpi->regs + LTPI_LINK_MANAGE_CTRL0);
	val &= ~REG_LTPI_RX_LINK_SP_FRM_NUM;
	val |= FIELD_PREP(REG_LTPI_RX_LINK_SP_FRM_NUM,
			  ltpi->link_speed_frm_rx_cnt);
	writel(val, ltpi->regs + LTPI_LINK_MANAGE_CTRL0);

	/*
	 * ad_timeout in us = ad_timeout in clock cycles * period of 25MHz
	 * ad_timeout in clock cycles
	 * = ad_timeout in us / period of 25MHz
	 * = (ad_timeout in us * 1000)ns / 40ns = ad_timeout * 25
	 */
	writel(ltpi->ad_timeout * 25, ltpi->regs + LTPI_LINK_MANAGE_CTRL1);

	/* Set the clock source to the base frequency 25MHz */
	ltpi_phy_set_clksel(ltpi, REG_LTPI_PLL_25M, false);

	/* To make the remote side back to the link lost state */
	mdelay(ADVERTISE_TIMEOUT_US / 1000);

	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_SDR);
}

static int scu_get_pll_freq(struct aspeed_ltpi_priv *ltpi, int pll_id)
{
	const struct pll_info *info;
	u32 reg;
	int m, n, p;

	if (pll_id > 1)
		return -1;

	info = &scu_pll_info[pll_id];
	regmap_read(ltpi->scu, info->reg_offset0, &reg);
	m = FIELD_GET(PLL_REG1_M, reg);
	n = FIELD_GET(PLL_REG1_N, reg);
	p = FIELD_GET(PLL_REG1_P, reg);

	return (25000000 * (m + 1) / (n + 1) / (p + 1));
}

static int scu_set_pll_freq(struct aspeed_ltpi_priv *ltpi, int pll_id, int freq)
{
	const struct pll_info *info;
	const struct pll_param *param;
	int curr_freq, i;
	bool match = false;

	if (pll_id > 1)
		return -EINVAL;

	curr_freq = scu_get_pll_freq(ltpi, pll_id);
	if (curr_freq == freq)
		return 0;

	for (i = 0; i < NUM_PLL_PARAM; i++) {
		if (freq == pll_param_lookup[i].freq) {
			match = true;
			break;
		}
	}

	if (!match)
		return -EINVAL;

	param = &pll_param_lookup[i];
	info = &scu_pll_info[pll_id];

	dev_info(ltpi->dev, "Setting PLL frequency to %d MHz\n",
		 freq / 1000000);
	regmap_update_bits(ltpi->scu, info->reg_offset0, PLL_REG1_RESET,
			   PLL_REG1_RESET);
	regmap_update_bits(ltpi->scu, info->reg_offset0,
			   PLL_REG1_P | PLL_REG1_N | PLL_REG1_M, param->n_m_p);
	regmap_update_bits(ltpi->scu, info->reg_offset1, PLL_REG2_BWADJ,
			   param->bwadj);

	udelay(5);
	regmap_update_bits(ltpi->scu, info->reg_offset0, PLL_REG1_RESET, 0);
	udelay(20);

	return 0;
}

static int ltpi_set_operational_clk(struct aspeed_ltpi_priv *ltpi,
				    u16 speed_cap)
{
	const struct ltpi_clk_info *info;
	int target_speed, clksel, phy_mode, pll_id;

	pll_id = ltpi->index; /* 0 or 1 maps to L0PLL or L1PLL in array */

	/* find max attainable speed */
	target_speed = find_max_speed(speed_cap);

	/* set phy mode "OFF" */
	ltpi_phy_set_mode(ltpi, LTPI_PHY_MODE_OFF);

	if (speed_cap & LTPI_SP_CAP_DDR) {
		phy_mode = LTPI_PHY_MODE_DDR;
		info = ltpi_clk_lookup_ddr;
	} else {
		phy_mode = LTPI_PHY_MODE_SDR;
		info = ltpi_clk_lookup_sdr;
	}
	clksel = info[target_speed].clk_sel;
	ltpi_phy_set_clksel(ltpi, clksel, true);
	if (clksel == REG_LTPI_PLL_LPLL)
		scu_set_pll_freq(ltpi, pll_id,
				 info[target_speed].freq * 1000000);

	/* Start TX with the operational frequency */
	ltpi_phy_set_mode(ltpi, phy_mode);

	return target_speed;
}

static int ltpi_scm_init(struct aspeed_ltpi_priv *ltpi)
{
	int ret, target_speed;
	u32 reg, state;

	dev_info(ltpi->dev, "Starting LTPI initialization\n");
	/* Check whether LTPI is initialized */
	state = ltpi_get_link_mng_state(ltpi);
	if (state == LTPI_LINK_MNG_ST_OP) {
		dev_info(ltpi->dev, "LTPI already operational PHY mode: %d\n",
			 ltpi_phy_get_mode(ltpi));
		return 0;
	}

	ltpi->t_link_detect = ktime_get_ns();

	/* LTPI initialization is required, start link training phase */
	do {
		dev_info(ltpi->dev, "Starting LTPI link training\n");
		ltpi_do_link_training(ltpi);
		do {
			ret = ltpi_wait_state_pll_set(ltpi, 20000);
			if (ret == 0)
				break;

			if (ltpi->op_timeout &&
			    (ktime_get_ns() - ltpi->t_link_detect >
			     ltpi->op_timeout)) {
				dev_err(ltpi->dev,
					"Timeout while waiting for PLL set state\n");
				ret = -ETIMEDOUT;
				goto ltpi_scm_exit;
			}
		} while (1);

		/* read intersection of the speed capabilities */
		reg = FIELD_GET(REG_LTPI_SP_INTERSETION,
				readl(ltpi->regs + LTPI_LINK_MNG_ST));
		if (reg == 0) {
			dev_err(ltpi->dev, "No common speed\n");
			ret = -ENOLINK;
			goto ltpi_scm_exit;
		}

		target_speed = ltpi_set_operational_clk(ltpi, reg);

		/* poll link state 0x7 */
		ret = ltpi_wait_state_op(ltpi);
		if (ret == 0) {
			/* Start OEM TX & RX if the link partner is AST1700 */
			if (readl(ltpi->regs + LTPI_LINK_MNG_ST) &
			    REG_LTPI_LINK_PARTNER_FLAG) {
				u32 val = readl(ltpi->regs +
						LTPI_OEM_BUS_SETTING);
				val |= (REG_LTPI_OEM_RX_START_TRIG |
					REG_LTPI_OEM_TX_START_TRIG);
				writel(val, ltpi->regs + LTPI_OEM_BUS_SETTING);
			}
			break;
		}

		if (ltpi->op_timeout &&
		    (ktime_get_ns() - ltpi->t_link_detect > ltpi->op_timeout)) {
			dev_err(ltpi->dev,
				"Timeout while waiting for operational state\n");
			ret = -ETIMEDOUT;
			goto ltpi_scm_exit;
		}

		if (!ltpi->disable_auto_downshift)
			/* clear the bit to specify the current speed doesn't work */
			ltpi->phy_speed_cap &= ~BIT(target_speed);

		/* the lowest speed 25M should always be supported */
		if ((ltpi->phy_speed_cap & LTPI_SP_CAP_25M) == 0)
			ltpi->phy_speed_cap |= LTPI_SP_CAP_25M;

		dev_warn(ltpi->dev,
			 "Failed to enter operational state, restarting link training\n");
		ltpi_dump(ltpi);
	} while (1);

	dev_info(ltpi->dev, "LTPI Link trained successfully\n");
	return 0;

ltpi_scm_exit:
	dev_err(ltpi->dev, "Exiting initialization\n");
	ltpi_dump(ltpi);
	ltpi_reset(ltpi);
	return ret;
}

/* Sysfs attribute show/store functions */
static ssize_t ad_timeout_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->ad_timeout);
}

static ssize_t ad_timeout_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->ad_timeout = val;
	return count;
}

static ssize_t io_driving_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", priv->io_driving);
}

static ssize_t io_driving_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->io_driving = val;
	return count;
}

static ssize_t clk_inverse_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", priv->clk_inverse);
}

static ssize_t clk_inverse_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->clk_inverse = val;
	return count;
}

static ssize_t link_speed_frm_rx_cnt_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->link_speed_frm_rx_cnt);
}

static ssize_t link_speed_frm_rx_cnt_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->link_speed_frm_rx_cnt = val;
	return count;
}

static ssize_t disable_auto_downshift_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->disable_auto_downshift);
}

static ssize_t disable_auto_downshift_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->disable_auto_downshift = val ? 1 : 0;
	return count;
}

static ssize_t crc_format_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->crc_format);
}

static ssize_t crc_format_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->crc_format = val ? 1 : 0;
	return count;
}

static ssize_t phy_speed_cap_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", priv->phy_speed_cap);
}

static ssize_t phy_speed_cap_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	priv->phy_speed_cap = val;
	return count;
}

static ssize_t op_timeout_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%llu\n", priv->op_timeout);
}

static ssize_t op_timeout_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	unsigned long long val;

	if (kstrtoull(buf, 0, &val))
		return -EINVAL;

	priv->op_timeout = val;
	return count;
}

static ssize_t rescan_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	u32 val, reg;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val == 1 && priv->version == AST2700) {
		dev_info(priv->dev, "Triggering LTPI rescan\n");
		if (ltpi_scm_init(priv)) {
			dev_err(priv->dev, "LTPI rescan failed\n");
			return -EIO;
		}
		writel(LTPI_INTR_EN_OP_LINK_LOST,
		       priv->regs + LTPI_INTR_STATUS);
		writel(LTPI_INTR_EN_OP_LINK_LOST, priv->regs + LTPI_INTR_EN);
		aspeed_ltpi_init_mux(priv);
		if (ltpi_get_link_partner(priv)) {
			reg = FIELD_PREP(REG_LTPI_AHB_ADDR_MAP0, 0x5) |
			      FIELD_PREP(REG_LTPI_AHB_ADDR_MAP1, 0xa0);
		} else {
			reg = 0;
			writel(0, priv->regs + LTPI_DATA_CH_CFG0);
		}
		writel(reg, priv->regs + LTPI_AHB_CTRL0);
	}

	return count;
}

static ssize_t link_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aspeed_ltpi_priv *priv = dev_get_drvdata(dev);
	u32 state = ltpi_get_link_mng_state(priv);
	char state_str[128];
	int speed;
	u32 phy_mode, clk_select;
	char modes[9][8] = { "OFF", "SDR", "DDR", "NA",	   "CDR_LO",
			     "NA",  "NA",  "NA",  "CDR_HI" };

	if (state != LTPI_LINK_MNG_ST_OP) {
		sprintf(state_str, "LTPI: not linked\n");
	} else {
		phy_mode = FIELD_GET(REG_LTPI_PHY_MODE, readl(priv->phy_regs + LTPI_PHY_CTRL));
		clk_select = FIELD_GET(REG_LTPI_PLL_SELECT, readl(priv->phy_regs + LTPI_PLL_CTRL));
		if (clk_select == REG_LTPI_PLL_LPLL)
			speed = scu_get_pll_freq(priv, priv->index) / 1000000;
		else
			speed = 25;

		sprintf(state_str,
			"LTPI%d:\n"
			"    link partner    : %s\n"
			"    link mode       : %s\n"
			"    link bandwidth  : %dMbps\n",
			priv->index,
			ltpi_get_link_partner(priv) ? "ast1700" : "fpga",
			&modes[phy_mode][0], speed);
	}

	return sprintf(buf, "%s\n", state_str);
}

/* Device attributes */
static DEVICE_ATTR_RW(ad_timeout);
static DEVICE_ATTR_RW(io_driving);
static DEVICE_ATTR_RW(clk_inverse);
static DEVICE_ATTR_RW(link_speed_frm_rx_cnt);
static DEVICE_ATTR_RW(disable_auto_downshift);
static DEVICE_ATTR_RW(crc_format);
static DEVICE_ATTR_RW(phy_speed_cap);
static DEVICE_ATTR_RW(op_timeout);
static DEVICE_ATTR_WO(rescan);
static DEVICE_ATTR_RO(link_status);

static struct attribute *aspeed_ltpi_attrs[] = {
	&dev_attr_ad_timeout.attr,
	&dev_attr_io_driving.attr,
	&dev_attr_clk_inverse.attr,
	&dev_attr_link_speed_frm_rx_cnt.attr,
	&dev_attr_disable_auto_downshift.attr,
	&dev_attr_crc_format.attr,
	&dev_attr_phy_speed_cap.attr,
	&dev_attr_op_timeout.attr,
	&dev_attr_rescan.attr,
	&dev_attr_link_status.attr,
	NULL,
};

static const struct attribute_group aspeed_ltpi_attr_group = {
	.attrs = aspeed_ltpi_attrs,
};

static int aspeed_ltpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_dev_auxdata *lookup = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	struct aspeed_ltpi_priv *priv;
	int irq, ret;
	struct resource *res;

	match = of_match_device(dev->driver->of_match_table, dev);

	if (match) {
		if (of_property_match_string(np, "compatible",
					     match->compatible) < 0)
			return -ENODEV;
	} else {
		return -ENODEV;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->regs = devm_platform_ioremap_resource_byname(pdev, "base");
	if (IS_ERR(priv->regs))
		priv->regs =
			devm_platform_ioremap_resource(pdev, 0); // Fallback
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->phy_regs = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(priv->phy_regs))
		priv->phy_regs =
			priv->regs + 0x200; // Fallback unsafe if size small

	priv->top_regs = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(priv->top_regs))
		priv->top_regs =
			priv->regs + 0x800; // Fallback unsafe if size small

	/* Identify index based on physical address for PLL control */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && (res->start & 0x1000))
		priv->index = 1;
	else
		priv->index = 0;

	priv->version = (enum chip_version)device_get_match_data(dev);

	priv->ltpi_clk = devm_clk_get(&pdev->dev, "ltpi");
	if (IS_ERR(priv->ltpi_clk)) {
		priv->ltpi_clk = devm_clk_get(&pdev->dev, "ahb");
		if (IS_ERR(priv->ltpi_clk))
			return PTR_ERR(priv->ltpi_clk);

		clk_prepare_enable(priv->ltpi_clk);

		priv->ltpi_phyclk = devm_clk_get(&pdev->dev, "phy");
		if (IS_ERR(priv->ltpi_phyclk))
			return PTR_ERR(priv->ltpi_phyclk);

		clk_prepare_enable(priv->ltpi_phyclk);
	} else {
		priv->ltpi_phyclk = NULL;
	}

	priv->ltpi_rst =
		devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(priv->ltpi_rst))
		return PTR_ERR(priv->ltpi_rst);

	reset_control_deassert(priv->ltpi_rst);

	priv->i2c_tunneling = GENMASK(MAX_I2C_IN_LTPI - 1, 0);
	if (!of_property_read_u32(np, "i2c-tunneling", &ret))
		priv->i2c_tunneling = ret;

	priv->i2c_timing_0 = LTPI_I2C_100K_0;
	priv->i2c_timing_1 = LTPI_I2C_100K_1;
	if (!of_property_read_u32(np, "i2c-tunneling-timing", &ret)) {
		if (ret == 400) {
			priv->i2c_timing_0 = LTPI_I2C_400K_0;
			priv->i2c_timing_1 = LTPI_I2C_400K_1;
		}
	}
	priv->uart_tunneling = GENMASK(MAX_UART_IN_LTPI - 1, 0);
	if (!of_property_read_u32(np, "uart-tunneling", &ret))
		priv->uart_tunneling = ret;

	priv->scu = syscon_regmap_lookup_by_phandle(np, "aspeed,scu");
	if (IS_ERR(priv->scu)) {
		dev_err(&pdev->dev, "failed to get SCU regmap\n");
		return PTR_ERR(priv->scu);
	}
	if (of_get_property(np, "remote-controller", NULL)) {
		u32 reg;

		/* Clear all the pins/otp strap but LTPI related settings for AST1700 */
		regmap_read(priv->scu, SCU_IO_PINS_TRAP1, &reg);
		reg &= ~SCU_IO_PINS_TRAP_LTPI;
		regmap_write(priv->scu, SCU_IO_PINS_TRAP1_CLEAR, reg);

		regmap_read(priv->scu, SCU_IO_OTP_TRAP1, &reg);
		regmap_write(priv->scu, SCU_IO_OTP_TRAP1_CLEAR, reg);

		regmap_read(priv->scu, SCU_IO_OTP_TRAP2, &reg);
		regmap_write(priv->scu, SCU_IO_OTP_TRAP2_CLEAR, reg);
	} else {
		irq = platform_get_irq(pdev, 0);
		ret = devm_request_irq(priv->dev, irq, aspeed_ltpi_irq_handler,
				       0, dev_name(priv->dev), priv);
		if (ret) {
			dev_err(priv->dev, "failed to request irq\n");
			reset_control_assert(priv->ltpi_rst);
			clk_disable_unprepare(priv->ltpi_phyclk);
			clk_disable_unprepare(priv->ltpi_clk);
			return ret;
		}

		writel(LTPI_INTR_EN_OP_LINK_LOST,
		       priv->regs + LTPI_INTR_STATUS);
		writel(LTPI_INTR_EN_OP_LINK_LOST, priv->regs + LTPI_INTR_EN);
	}

	/* Initialize training parameters with defaults */
	priv->ad_timeout = ADVERTISE_TIMEOUT_US;
	priv->io_driving = 0x2;
	priv->clk_inverse = 0x0;
	priv->link_speed_frm_rx_cnt = 0;
	priv->disable_auto_downshift = 0;
	priv->crc_format = 0;
	priv->otp_ddr_dis = false;
	priv->op_timeout = 2000000000; /* 2 seconds timeout by default */
	priv->phy_speed_cap = LTPI_SP_CAP_ASPEED_SUPPORTED;

	aspeed_ltpi_init_mux(priv);

	platform_set_drvdata(pdev, priv);

	/* Create sysfs attributes */
	ret = sysfs_create_group(&dev->kobj, &aspeed_ltpi_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group\n");
		reset_control_assert(priv->ltpi_rst);
		clk_disable_unprepare(priv->ltpi_phyclk);
		clk_disable_unprepare(priv->ltpi_clk);
		return ret;
	}

	if (np)
		of_platform_populate(np, NULL, lookup, priv->dev);

	return 0;
}

static void aspeed_ltpi_remove(struct platform_device *pdev)
{
	struct aspeed_ltpi_priv *priv;

	priv = platform_get_drvdata(pdev);

	/* Remove sysfs attributes */
	sysfs_remove_group(&pdev->dev.kobj, &aspeed_ltpi_attr_group);

	reset_control_assert(priv->ltpi_rst);
	clk_disable_unprepare(priv->ltpi_phyclk);
	clk_disable_unprepare(priv->ltpi_clk);
}

static const struct of_device_id aspeed_ltpi_of_match[] = {
	{
		.compatible = "aspeed-ltpi",
		.data = (const void *)AST2700,
	},
	{
		.compatible = "aspeed-ast1700-ltpi",
		.data = (const void *)AST1700,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aspeed_ltpi_of_match);

static struct platform_driver aspeed_ltpi_driver = {
	.probe = aspeed_ltpi_probe,
	.remove = aspeed_ltpi_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_ltpi_of_match,
	},
};

module_platform_driver(aspeed_ltpi_driver);

MODULE_DESCRIPTION("LVDS Tunneling Protocol and Interface Bus Driver");
MODULE_AUTHOR("Dylan Hung <dylan_hung@aspeedtech.com>");
MODULE_LICENSE("GPL");
