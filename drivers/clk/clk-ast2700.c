// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 ASPEED Technology Inc.
 * Author: Ryan Chen <ryan_chen@aspeedtech.com>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/units.h>

#include <soc/aspeed/reset-aspeed.h>
#include <dt-bindings/clock/aspeed,ast2700-scu.h>

#define SCU_CLK_12MHZ	(12 * HZ_PER_MHZ)
#define SCU_CLK_24MHZ	(24 * HZ_PER_MHZ)
#define SCU_CLK_25MHZ	(25 * HZ_PER_MHZ)
#define SCU_CLK_192MHZ	(192 * HZ_PER_MHZ)

/* SOC0 */
#define SCU0_HWSTRAP1		0x010
#define SCU0_CLK_STOP		0x240
#define SCU0_CLK_SEL1		0x280
#define SCU0_CLK_SEL2		0x284
#define GET_USB_REFCLK_DIV(x)	((GENMASK(23, 20) & (x)) >> 20)
#define UART_DIV13_EN		BIT(30)
#define SCU0_HPLL_PARAM		0x300
#define SCU0_DPLL_PARAM		0x308
#define SCU0_MPLL_PARAM		0x310
#define SCU0_D0CLK_PARAM	0x320
#define SCU0_D1CLK_PARAM	0x330
#define SCU0_CRT0CLK_PARAM	0x340
#define SCU0_CRT1CLK_PARAM	0x350
#define SCU0_MPHYCLK_PARAM	0x360

/* SOC1 */
#define SCU1_REVISION_ID	0x0
#define REVISION_ID		GENMASK(23, 16)
#define SCU1_CLK_STOP		0x240
#define SCU1_CLK_STOP2		0x260
#define SCU1_CLK_SEL1		0x280
#define SCU1_CLK_SEL2		0x284
#define SCU1_CLK_I3C_DIV_MASK	GENMASK(25, 23)
#define SCU1_CLK_I3C_DIV(n)	((n) - 1)
#define UXCLK_MASK		GENMASK(1, 0)
#define HUXCLK_MASK		GENMASK(4, 3)
#define SCU1_HPLL_PARAM		0x300
#define SCU1_APLL_PARAM		0x310
#define SCU1_DPLL_PARAM		0x320
#define SCU1_UXCLK_CTRL		0x330
#define SCU1_HUXCLK_CTRL	0x334
#define SCU1_MAC12_CLK_DLY	0x390
#define SCU1_MAC12_CLK_DLY_100M	0x394
#define SCU1_MAC12_CLK_DLY_10M	0x398

/*
 * MAC Clock Delay settings
 */
#define MAC_CLK_RMII1_50M_RCLK_O_CTRL	BIT(30)
#define   MAC_CLK_RMII1_50M_RCLK_O_DIS	0
#define   MAC_CLK_RMII1_50M_RCLK_O_EN	1
#define MAC_CLK_RMII0_50M_RCLK_O_CTRL	BIT(29)
#define   MAC_CLK_RMII0_5M_RCLK_O_DIS	0
#define   MAC_CLK_RMII0_5M_RCLK_O_EN	1
#define MAC_CLK_RMII_TXD_FALLING_2	BIT(27)
#define MAC_CLK_RMII_TXD_FALLING_1	BIT(26)
#define MAC_CLK_RXCLK_INV_2		BIT(25)
#define MAC_CLK_RXCLK_INV_1		BIT(24)
#define MAC_CLK_1G_INPUT_DELAY_2	GENMASK(23, 18)
#define MAC_CLK_1G_INPUT_DELAY_1	GENMASK(17, 12)
#define MAC_CLK_1G_OUTPUT_DELAY_2	GENMASK(11, 6)
#define MAC_CLK_1G_OUTPUT_DELAY_1	GENMASK(5, 0)

#define MAC_CLK_100M_10M_RESERVED	GENMASK(31, 26)
#define MAC_CLK_100M_10M_RXCLK_INV_2	BIT(25)
#define MAC_CLK_100M_10M_RXCLK_INV_1	BIT(24)
#define MAC_CLK_100M_10M_INPUT_DELAY_2	GENMASK(23, 18)
#define MAC_CLK_100M_10M_INPUT_DELAY_1	GENMASK(17, 12)
#define MAC_CLK_100M_10M_OUTPUT_DELAY_2	GENMASK(11, 6)
#define MAC_CLK_100M_10M_OUTPUT_DELAY_1	GENMASK(5, 0)

#define AST2700_DEF_MAC12_DELAY_1G_A0	0x00CF4D75
#define AST2700_DEF_MAC12_DELAY_1G_A1	0x00C70C73
#define AST2700_DEF_MAC12_DELAY_100M	0x00410410
#define AST2700_DEF_MAC12_DELAY_10M	0x00410410

struct mac_delay_config {
	u32 tx_delay_1000;
	u32 rx_delay_1000;
	u32 tx_delay_100;
	u32 rx_delay_100;
	u32 tx_delay_10;
	u32 rx_delay_10;
};

enum ast2700_clk_type {
	CLK_MUX,
	CLK_PLL,
	CLK_HPLL,
	CLK_GATE,
	CLK_MISC,
	CLK_FIXED,
	DCLK_FIXED,
	CLK_DIVIDER,
	CLK_UART_PLL,
	CLK_FIXED_FACTOR,
	CLK_GATE_ASPEED,
};

struct ast2700_clk_fixed_factor_data {
	const struct clk_parent_data *parent;
	unsigned int mult;
	unsigned int div;
};

struct ast2700_clk_gate_data {
	const struct clk_parent_data *parent;
	u32 flags;
	u32 reg;
	u8 bit;
};

struct ast2700_clk_mux_data {
	const struct clk_parent_data *parents;
	unsigned int num_parents;
	u8 bit_shift;
	u8 bit_width;
	u32 reg;
};

struct ast2700_clk_div_data {
	const struct clk_div_table *div_table;
	const struct clk_parent_data *parent;
	u8 bit_shift;
	u8 bit_width;
	u32 reg;
};

struct ast2700_clk_pll_data {
	const struct clk_parent_data *parent;
	u32 reg;
};

struct ast2700_clk_fixed_rate_data {
	unsigned long fixed_rate;
};

struct ast2700_clk_info {
	const char *name;
	u8 clk_idx;
	u32 reg;
	u32 type;
	union {
		struct ast2700_clk_fixed_factor_data factor;
		struct ast2700_clk_fixed_rate_data rate;
		struct ast2700_clk_gate_data gate;
		struct ast2700_clk_div_data div;
		struct ast2700_clk_pll_data pll;
		struct ast2700_clk_mux_data mux;
	} data;
};

struct ast2700_clk_data {
	struct ast2700_clk_info const *clk_info;
	unsigned int nr_clks;
	const int scu;
};

struct ast2700_clk_ctrl {
	const struct ast2700_clk_data *clk_data;
	struct device *dev;
	void __iomem *base;
	spinlock_t lock; /* clk lock */
};

static const struct clk_div_table ast2700_rgmii_div_table[] = {
	{ 0x0, 4 },
	{ 0x1, 4 },
	{ 0x2, 6 },
	{ 0x3, 8 },
	{ 0x4, 10 },
	{ 0x5, 12 },
	{ 0x6, 14 },
	{ 0x7, 16 },
	{ 0 }
};

static const struct clk_div_table ast2700_rmii_div_table[] = {
	{ 0x0, 8 },
	{ 0x1, 8 },
	{ 0x2, 12 },
	{ 0x3, 16 },
	{ 0x4, 20 },
	{ 0x5, 24 },
	{ 0x6, 28 },
	{ 0x7, 32 },
	{ 0 }
};

static const struct clk_div_table ast2700_clk_div_table[] = {
	{ 0x0, 2 },
	{ 0x1, 2 },
	{ 0x2, 3 },
	{ 0x3, 4 },
	{ 0x4, 5 },
	{ 0x5, 6 },
	{ 0x6, 7 },
	{ 0x7, 8 },
	{ 0 }
};

static const struct clk_div_table ast2700_clk_div_table2[] = {
	{ 0x0, 2 },
	{ 0x1, 4 },
	{ 0x2, 6 },
	{ 0x3, 8 },
	{ 0x4, 10 },
	{ 0x5, 12 },
	{ 0x6, 14 },
	{ 0x7, 16 },
	{ 0 }
};

static const struct clk_div_table ast2700_hclk_div_table[] = {
	{ 0x0, 6 },
	{ 0x1, 5 },
	{ 0x2, 4 },
	{ 0x3, 7 },
	{ 0 }
};

static const struct clk_div_table ast2700_clk_uart_div_table[] = {
	{ 0x0, 1 },
	{ 0x1, 13 },
	{ 0 }
};

static const struct clk_parent_data soc0_clkin[] = {
	{ .fw_name = "soc0-clkin", .name = "soc0-clkin" },
};

static const struct clk_parent_data pspclk[] = {
	{ .fw_name = "pspclk", .name = "pspclk" },
};

static const struct clk_parent_data soc0_mpll_div8[] = {
	{ .fw_name = "soc0-mpll_div8", .name = "soc0-mpll_div8" },
};

static const struct clk_parent_data mphysrc[] = {
	{ .fw_name = "mphysrc", .name = "mphysrc" },
};

static const struct clk_parent_data u2phy_refclksrc[] = {
	{ .fw_name = "u2phy_refclksrc", .name = "u2phy_refclksrc" },
};

static const struct clk_parent_data soc0_hpll[] = {
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
};

static const struct clk_parent_data soc0_mpll[] = {
	{ .fw_name = "soc0-mpll", .name = "soc0-mpll" },
};

static const struct clk_parent_data axi0clk[] = {
	{ .fw_name = "axi0clk", .name = "axi0clk" },
};

static const struct clk_parent_data soc0_ahbmux[] = {
	{ .fw_name = "soc0-ahbmux", .name = "soc0-ahbmux" },
};

static const struct clk_parent_data soc0_ahb[] = {
	{ .fw_name = "soc0-ahb", .name = "soc0-ahb" },
};

static const struct clk_parent_data soc0_uartclk[] = {
	{ .fw_name = "soc0-uartclk", .name = "soc0-uartclk" },
};

static const struct clk_parent_data emmcclk[] = {
	{ .fw_name = "emmcclk", .name = "emmcclk" },
};

static const struct clk_parent_data emmcsrc_mux[] = {
	{ .fw_name = "emmcsrc-mux", .name = "emmcsrc-mux" },
};

static const struct clk_parent_data soc1_clkin[] = {
	{ .fw_name = "soc1-clkin", .name = "soc1-clkin" },
};

static const struct clk_parent_data soc1_hpll[] = {
	{ .fw_name = "soc1-hpll", .name = "soc1-hpll" },
};

static const struct clk_parent_data soc1_apll[] = {
	{ .fw_name = "soc1-apll", .name = "soc1-apll" },
};

static const struct clk_parent_data sdclk[] = {
	{ .fw_name = "sdclk", .name = "sdclk" },
};

static const struct clk_parent_data sdclk_mux[] = {
	{ .fw_name = "sdclk-mux", .name = "sdclk-mux" },
};

static const struct clk_parent_data huartxclk[] = {
	{ .fw_name = "huartxclk", .name = "huartxclk" },
};

static const struct clk_parent_data uxclk[] = {
	{ .fw_name = "uxclk", .name = "uxclk" },
};

static const struct clk_parent_data huxclk[] = {
	{ .fw_name = "huxclk", .name = "huxclk" },
};

static const struct clk_parent_data uart0clk[] = {
	{ .fw_name = "uart0clk", .name = "uart0clk" },
};

static const struct clk_parent_data uart1clk[] = {
	{ .fw_name = "uart1clk", .name = "uart1clk" },
};

static const struct clk_parent_data uart2clk[] = {
	{ .fw_name = "uart2clk", .name = "uart2clk" },
};

static const struct clk_parent_data uart3clk[] = {
	{ .fw_name = "uart3clk", .name = "uart3clk" },
};

static const struct clk_parent_data uart5clk[] = {
	{ .fw_name = "uart5clk", .name = "uart5clk" },
};

static const struct clk_parent_data uart4clk[] = {
	{ .fw_name = "uart4clk", .name = "uart4clk" },
};

static const struct clk_parent_data uart6clk[] = {
	{ .fw_name = "uart6clk", .name = "uart6clk" },
};

static const struct clk_parent_data uart7clk[] = {
	{ .fw_name = "uart7clk", .name = "uart7clk" },
};

static const struct clk_parent_data uart8clk[] = {
	{ .fw_name = "uart8clk", .name = "uart8clk" },
};

static const struct clk_parent_data uart9clk[] = {
	{ .fw_name = "uart9clk", .name = "uart9clk" },
};

static const struct clk_parent_data uart10clk[] = {
	{ .fw_name = "uart10clk", .name = "uart10clk" },
};

static const struct clk_parent_data uart11clk[] = {
	{ .fw_name = "uart11clk", .name = "uart11clk" },
};

static const struct clk_parent_data uart12clk[] = {
	{ .fw_name = "uart12clk", .name = "uart12clk" },
};

static const struct clk_parent_data uart13clk[] = {
	{ .fw_name = "uart13clk", .name = "uart13clk" },
};

static const struct clk_parent_data uart14clk[] = {
	{ .fw_name = "uart14clk", .name = "uart14clk" },
};

static const struct clk_parent_data uart15clk[] = {
	{ .fw_name = "uart15clk", .name = "uart15clk" },
};

static const struct clk_parent_data uart16clk[] = {
	{ .fw_name = "uart16clk", .name = "uart16clk" },
};

static const struct clk_parent_data soc1_ahb[] = {
	{ .fw_name = "soc1-ahb", .name = "soc1-ahb" },
};

static const struct clk_parent_data soc1_i3c[] = {
	{ .fw_name = "soc1-i3c", .name = "soc1-i3c" },
};

static const struct clk_parent_data canclk[] = {
	{ .fw_name = "canclk", .name = "canclk" },
};

static const struct clk_parent_data rmii[] = {
	{ .fw_name = "rmii", .name = "rmii" },
};

static const struct clk_parent_data d_clk_sels[] = {
	{ .fw_name = "soc0-hpll_div2", .name = "soc0-hpll_div2" },
	{ .fw_name = "soc0-mpll_div2", .name = "soc0-mpll_div2" },
};

static const struct clk_parent_data hclk_clk_sels[] = {
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-mpll", .name = "soc0-mpll" },
};

static const struct clk_parent_data mhpll_clk_sels[] = {
	{ .fw_name = "soc0-mpll", .name = "soc0-mpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
};

static const struct clk_parent_data mphy_clk_sels[] = {
	{ .fw_name = "soc0-mpll", .name = "soc0-mpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-dpll", .name = "soc0-dpll" },
	{ .fw_name = "soc0-clk192Mhz", .name = "soc0-clk192Mhz" },
};

static const struct clk_parent_data psp_clk_sels[] = {
	{ .fw_name = "soc0-mpll", .name = "soc0-mpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-mpll_div2", .name = "soc0-mpll_div2" },
	{ .fw_name = "soc0-hpll_div2", .name = "soc0-hpll_div2" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
	{ .fw_name = "soc0-hpll", .name = "soc0-hpll" },
};

static const struct clk_parent_data uart_clk_sels[] = {
	{ .fw_name = "soc0-clk24Mhz", .name = "soc0-clk24Mhz" },
	{ .fw_name = "soc0-clk192Mhz", .name = "soc0-clk192Mhz" },
};

static const struct clk_parent_data emmc_clk_sels[] = {
	{ .fw_name = "soc0-mpll_div4", .name = "soc0-mpll_div4" },
	{ .fw_name = "soc0-hpll_div4", .name = "soc0-hpll_div4" },
};

static const struct clk_parent_data sdio_clk_sels[] = {
	{ .fw_name = "soc1-hpll", .name = "soc1-hpll" },
	{ .fw_name = "soc1-apll", .name = "soc1-apll" },
};

static const struct clk_parent_data ux_clk_sels[] = {
	{ .fw_name = "soc1-apll_div4", .name = "soc1-apll_div4" },
	{ .fw_name = "soc1-apll_div2", .name = "soc1-apll_div2" },
	{ .fw_name = "soc1-apll", .name = "soc1-apll" },
	{ .fw_name = "soc1-hpll", .name = "soc1-hpll" },
};

static const struct clk_parent_data uartx_clk_sels[] = {
	{ .fw_name = "uartxclk", .name = "uartxclk" },
	{ .fw_name = "huartxclk", .name = "huartxclk" },
};

#define FIXED_CLK(_id, _name, _rate) \
	[_id] = { \
		.type = CLK_FIXED, \
		.name = _name, \
		.data = { .rate = { .fixed_rate = _rate, } }, \
	}

#define PLL_CLK(_id, _type, _name, _parent, _reg) \
	[_id] = { \
		.type = _type, \
		.name = _name, \
		.data = { .pll = { .parent = _parent, .reg = _reg, } }, \
	}

#define MUX_CLK(_id, _name, _parents, _num_parents, _reg, _shift, _width) \
	[_id] = { \
		.type = CLK_MUX, \
		.name = _name, \
		.data = { \
			.mux = { \
				.parents = _parents, \
				.num_parents = _num_parents, \
				.reg = _reg, \
				.bit_shift = _shift, \
				.bit_width = _width, \
			}, \
		}, \
	}

#define DIVIDER_CLK(_id, _name, _parent, _reg, _shift, _width, _div_table) \
	[_id] = { \
		.type = CLK_DIVIDER, \
		.name = _name, \
		.data = { \
			.div = { \
				.parent = _parent, \
				.reg = _reg, \
				.bit_shift = _shift, \
				.bit_width = _width, \
				.div_table = _div_table, \
			}, \
		}, \
	}

#define FIXED_FACTOR_CLK(_id, _name, _parent, _mult, _div) \
	[_id] = { \
		.type = CLK_FIXED_FACTOR, \
		.name = _name, \
		.data = { .factor = { .parent = _parent, .mult = _mult, .div = _div, } }, \
	}

#define GATE_CLK(_id, _type, _name, _parent, _reg, _bit, _flags) \
	[_id] = { \
		.type = _type, \
		.name = _name, \
		.data = { \
			.gate = { \
				.parent = _parent, \
				.reg = _reg, \
				.bit = _bit, \
				.flags = _flags, \
			}, \
		}, \
	}

static const struct ast2700_clk_info ast2700_scu0_clk_info[] __initconst = {
	FIXED_CLK(SCU0_CLKIN, "soc0-clkin", SCU_CLK_25MHZ),
	FIXED_CLK(SCU0_CLK_24M, "soc0-clk24Mhz", SCU_CLK_24MHZ),
	FIXED_CLK(SCU0_CLK_192M, "soc0-clk192Mhz", SCU_CLK_192MHZ),
	FIXED_CLK(SCU0_CLK_U2PHY_CLK12M, "u2phy_clk12m", SCU_CLK_12MHZ),
	PLL_CLK(SCU0_CLK_HPLL, CLK_HPLL, "soc0-hpll", soc0_clkin, SCU0_HPLL_PARAM),
	PLL_CLK(SCU0_CLK_DPLL, CLK_PLL, "soc0-dpll", soc0_clkin, SCU0_DPLL_PARAM),
	PLL_CLK(SCU0_CLK_MPLL, CLK_PLL, "soc0-mpll", soc0_clkin, SCU0_MPLL_PARAM),
	PLL_CLK(SCU0_CLK_D0, DCLK_FIXED, "d0clk", NULL, SCU0_D0CLK_PARAM),
	PLL_CLK(SCU0_CLK_D1, DCLK_FIXED, "d1clk", NULL, SCU0_D1CLK_PARAM),
	PLL_CLK(SCU0_CLK_CRT0, DCLK_FIXED, "crt0clk", NULL, SCU0_CRT0CLK_PARAM),
	PLL_CLK(SCU0_CLK_CRT1, DCLK_FIXED, "crt1clk", NULL, SCU0_CRT1CLK_PARAM),
	PLL_CLK(SCU0_CLK_MPHY, CLK_MISC, "mphyclk", mphysrc, SCU0_MPHYCLK_PARAM),
	PLL_CLK(SCU0_CLK_U2PHY_REFCLK, CLK_MISC, "u2phy_refclk", u2phy_refclksrc, SCU0_CLK_SEL2),
	FIXED_FACTOR_CLK(SCU0_CLK_HPLL_DIV2, "soc0-hpll_div2", soc0_hpll, 1, 2),
	FIXED_FACTOR_CLK(SCU0_CLK_HPLL_DIV4, "soc0-hpll_div4", soc0_hpll, 1, 4),
	FIXED_FACTOR_CLK(SCU0_CLK_MPLL_DIV2, "soc0-mpll_div2", soc0_mpll, 1, 2),
	FIXED_FACTOR_CLK(SCU0_CLK_MPLL_DIV4, "soc0-mpll_div4", soc0_mpll, 1, 4),
	FIXED_FACTOR_CLK(SCU0_CLK_MPLL_DIV8, "soc0-mpll_div8", soc0_mpll, 1, 8),
	FIXED_FACTOR_CLK(SCU0_CLK_AXI0, "axi0clk", pspclk, 1, 2),
	FIXED_FACTOR_CLK(SCU0_CLK_AXI1, "axi1clk", soc0_mpll, 1, 4),
	DIVIDER_CLK(SCU0_CLK_AHB, "soc0-ahb", soc0_ahbmux,
		    SCU0_HWSTRAP1, 5, 2, ast2700_hclk_div_table),
	DIVIDER_CLK(SCU0_CLK_EMMC, "emmcclk", emmcsrc_mux,
		    SCU0_CLK_SEL1, 12, 3, ast2700_clk_div_table2),
	DIVIDER_CLK(SCU0_CLK_APB, "soc0-apb", axi0clk,
		    SCU0_CLK_SEL1, 23, 3, ast2700_clk_div_table2),
	DIVIDER_CLK(SCU0_CLK_UART4, "uart4clk", soc0_uartclk,
		    SCU0_CLK_SEL2, 30, 1, ast2700_clk_uart_div_table),
	MUX_CLK(SCU0_CLK_PSP, "pspclk", psp_clk_sels, ARRAY_SIZE(psp_clk_sels),
		SCU0_HWSTRAP1, 2, 3),
	MUX_CLK(SCU0_CLK_AHBMUX, "soc0-ahbmux", hclk_clk_sels, ARRAY_SIZE(hclk_clk_sels),
		SCU0_HWSTRAP1, 7, 1),
	MUX_CLK(SCU0_CLK_EMMCMUX, "emmcsrc-mux", emmc_clk_sels, ARRAY_SIZE(emmc_clk_sels),
		SCU0_CLK_SEL1, 11, 1),
	MUX_CLK(SCU0_CLK_MPHYSRC, "mphysrc", mphy_clk_sels, ARRAY_SIZE(mphy_clk_sels),
		SCU0_CLK_SEL2, 18, 2),
	MUX_CLK(SCU0_CLK_U2PHY_REFCLKSRC, "u2phy_refclksrc", mhpll_clk_sels,
		ARRAY_SIZE(mhpll_clk_sels), SCU0_CLK_SEL2, 23, 1),
	MUX_CLK(SCU0_CLK_UART, "soc0-uartclk", uart_clk_sels, ARRAY_SIZE(uart_clk_sels),
		SCU0_CLK_SEL2, 14, 1),
	GATE_CLK(SCU0_CLK_GATE_MCLK, CLK_GATE_ASPEED, "mclk-gate", soc0_mpll,
		 SCU0_CLK_STOP, 0, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_ECLK, CLK_GATE_ASPEED, "eclk-gate", NULL, SCU0_CLK_STOP, 1, 0),
	GATE_CLK(SCU0_CLK_GATE_2DCLK, CLK_GATE_ASPEED, "gclk-gate", NULL, SCU0_CLK_STOP, 2, 0),
	GATE_CLK(SCU0_CLK_GATE_VCLK, CLK_GATE_ASPEED, "vclk-gate", NULL, SCU0_CLK_STOP, 3, 0),
	GATE_CLK(SCU0_CLK_GATE_BCLK, CLK_GATE_ASPEED, "bclk-gate", NULL,
		 SCU0_CLK_STOP, 4, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_VGA0CLK,  CLK_GATE_ASPEED, "vga0clk-gate", NULL,
		 SCU0_CLK_STOP, 5, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_REFCLK,  CLK_GATE_ASPEED, "soc0-refclk-gate", soc0_clkin,
		 SCU0_CLK_STOP, 6, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_PORTBUSB2CLK, CLK_GATE_ASPEED, "portb-usb2clk-gate", NULL,
		 SCU0_CLK_STOP, 7, 0),
	GATE_CLK(SCU0_CLK_GATE_UHCICLK, CLK_GATE_ASPEED, "uhciclk-gate", NULL, SCU0_CLK_STOP, 9, 0),
	GATE_CLK(SCU0_CLK_GATE_VGA1CLK, CLK_GATE_ASPEED, "vga1clk-gate", NULL,
		 SCU0_CLK_STOP, 10, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_DDRPHYCLK, CLK_GATE_ASPEED, "ddrphy-gate", NULL,
		 SCU0_CLK_STOP, 11, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_E2M0CLK, CLK_GATE_ASPEED, "e2m0clk-gate", NULL,
		 SCU0_CLK_STOP, 12, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_HACCLK, CLK_GATE_ASPEED, "hacclk-gate", NULL, SCU0_CLK_STOP, 13, 0),
	GATE_CLK(SCU0_CLK_GATE_PORTAUSB2CLK, CLK_GATE_ASPEED, "porta-usb2clk-gate", NULL,
		 SCU0_CLK_STOP, 14, 0),
	GATE_CLK(SCU0_CLK_GATE_UART4CLK, CLK_GATE_ASPEED, "uart4clk-gate", uart4clk,
		 SCU0_CLK_STOP, 15, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_SLICLK, CLK_GATE_ASPEED, "soc0-sliclk-gate", NULL,
		 SCU0_CLK_STOP, 16, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_DACCLK, CLK_GATE_ASPEED, "dacclk-gate", NULL,
		 SCU0_CLK_STOP, 17, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_DP, CLK_GATE_ASPEED, "dpclk-gate", NULL,
		 SCU0_CLK_STOP, 18, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_E2M1CLK, CLK_GATE_ASPEED, "e2m1clk-gate", NULL,
		 SCU0_CLK_STOP, 19, CLK_IS_CRITICAL),
	GATE_CLK(SCU0_CLK_GATE_CRT0CLK, CLK_GATE_ASPEED, "crt0clk-gate", NULL,
		 SCU0_CLK_STOP, 20, 0),
	GATE_CLK(SCU0_CLK_GATE_CRT1CLK, CLK_GATE_ASPEED, "crt1clk-gate", NULL,
		 SCU0_CLK_STOP, 21, 0),
	GATE_CLK(SCU0_CLK_GATE_ECDSACLK, CLK_GATE_ASPEED, "eccclk-gate", NULL,
		 SCU0_CLK_STOP, 23, 0),
	GATE_CLK(SCU0_CLK_GATE_RSACLK, CLK_GATE_ASPEED, "rsaclk-gate", NULL,
		 SCU0_CLK_STOP, 24, 0),
	GATE_CLK(SCU0_CLK_GATE_RVAS0CLK, CLK_GATE_ASPEED, "rvas0clk-gate", NULL,
		 SCU0_CLK_STOP, 25, 0),
	GATE_CLK(SCU0_CLK_GATE_UFSCLK, CLK_GATE_ASPEED, "ufsclk-gate", NULL,
		 SCU0_CLK_STOP, 26, 0),
	GATE_CLK(SCU0_CLK_GATE_EMMCCLK, CLK_GATE_ASPEED, "emmcclk-gate", emmcclk,
		 SCU0_CLK_STOP, 27, 0),
	GATE_CLK(SCU0_CLK_GATE_RVAS1CLK, CLK_GATE_ASPEED, "rvas1clk-gate", NULL,
		 SCU0_CLK_STOP, 28, 0),
};

static const struct ast2700_clk_info ast2700_scu1_clk_info[] __initconst = {
	FIXED_CLK(SCU1_CLKIN, "soc1-clkin", SCU_CLK_25MHZ),
	PLL_CLK(SCU1_CLK_HPLL, CLK_PLL, "soc1-hpll", soc1_clkin, SCU1_HPLL_PARAM),
	PLL_CLK(SCU1_CLK_APLL, CLK_PLL, "soc1-apll", soc1_clkin, SCU1_APLL_PARAM),
	PLL_CLK(SCU1_CLK_DPLL, CLK_PLL, "soc1-dpll", soc1_clkin, SCU1_DPLL_PARAM),
	PLL_CLK(SCU1_CLK_UARTX, CLK_UART_PLL, "uartxclk", uxclk, SCU1_UXCLK_CTRL),
	PLL_CLK(SCU1_CLK_HUARTX, CLK_UART_PLL, "huartxclk", huxclk, SCU1_HUXCLK_CTRL),
	FIXED_FACTOR_CLK(SCU1_CLK_APLL_DIV2, "soc1-apll_div2", soc1_apll, 1, 2),
	FIXED_FACTOR_CLK(SCU1_CLK_APLL_DIV4, "soc1-apll_div4", soc1_apll, 1, 4),
	FIXED_FACTOR_CLK(SCU1_CLK_UART13, "uart13clk", huartxclk, 1, 1),
	FIXED_FACTOR_CLK(SCU1_CLK_UART14, "uart14clk", huartxclk, 1, 1),
	FIXED_FACTOR_CLK(SCU1_CLK_CAN, "canclk", soc1_apll, 1, 10),
	DIVIDER_CLK(SCU1_CLK_SDCLK, "sdclk", sdclk_mux,
		    SCU1_CLK_SEL1, 14, 3, ast2700_clk_div_table),
	DIVIDER_CLK(SCU1_CLK_APB, "soc1-apb", soc1_hpll,
		    SCU1_CLK_SEL1, 18, 3, ast2700_clk_div_table2),
	DIVIDER_CLK(SCU1_CLK_RMII, "rmii", soc1_hpll,
		    SCU1_CLK_SEL1, 21, 3, ast2700_rmii_div_table),
	DIVIDER_CLK(SCU1_CLK_RGMII, "rgmii", soc1_hpll,
		    SCU1_CLK_SEL1, 25, 3, ast2700_rgmii_div_table),
	DIVIDER_CLK(SCU1_CLK_MACHCLK, "machclk", soc1_hpll,
		    SCU1_CLK_SEL1, 29, 3, ast2700_clk_div_table),
	DIVIDER_CLK(SCU1_CLK_APLL_DIVN, "soc1-apll_divn", soc1_apll,
		    SCU1_CLK_SEL2, 8, 3, ast2700_clk_div_table),
	DIVIDER_CLK(SCU1_CLK_AHB, "soc1-ahb", soc1_hpll,
		    SCU1_CLK_SEL2, 20, 3, ast2700_clk_div_table),
	DIVIDER_CLK(SCU1_CLK_I3C, "soc1-i3c", soc1_hpll,
		    SCU1_CLK_SEL2, 23, 3, ast2700_clk_div_table),
	MUX_CLK(SCU1_CLK_UART0, "uart0clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 0, 1),
	MUX_CLK(SCU1_CLK_UART1, "uart1clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 1, 1),
	MUX_CLK(SCU1_CLK_UART2, "uart2clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 2, 1),
	MUX_CLK(SCU1_CLK_UART3, "uart3clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 3, 1),
	MUX_CLK(SCU1_CLK_UART5, "uart5clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 5, 1),
	MUX_CLK(SCU1_CLK_UART6, "uart6clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 6, 1),
	MUX_CLK(SCU1_CLK_UART7, "uart7clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 7, 1),
	MUX_CLK(SCU1_CLK_UART8, "uart8clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 8, 1),
	MUX_CLK(SCU1_CLK_UART9, "uart9clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 9, 1),
	MUX_CLK(SCU1_CLK_UART10, "uart10clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 10, 1),
	MUX_CLK(SCU1_CLK_UART11, "uart11clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 11, 1),
	MUX_CLK(SCU1_CLK_UART12, "uart12clk", uartx_clk_sels, ARRAY_SIZE(uartx_clk_sels),
		SCU1_CLK_SEL1, 12, 1),
	MUX_CLK(SCU1_CLK_SDMUX, "sdclk-mux", sdio_clk_sels, ARRAY_SIZE(sdio_clk_sels),
		SCU1_CLK_SEL1, 13, 1),
	MUX_CLK(SCU1_CLK_UXCLK, "uxclk", ux_clk_sels, ARRAY_SIZE(ux_clk_sels),
		SCU1_CLK_SEL2, 0, 2),
	MUX_CLK(SCU1_CLK_HUXCLK, "huxclk", ux_clk_sels, ARRAY_SIZE(ux_clk_sels),
		SCU1_CLK_SEL2, 3, 2),
	GATE_CLK(SCU1_CLK_MAC0RCLK, CLK_GATE, "mac0rclk-gate", rmii, SCU1_MAC12_CLK_DLY, 29, 0),
	GATE_CLK(SCU1_CLK_MAC1RCLK, CLK_GATE, "mac1rclk-gate", rmii, SCU1_MAC12_CLK_DLY, 30, 0),
	GATE_CLK(SCU1_CLK_GATE_LCLK0, CLK_GATE_ASPEED, "lclk0-gate", NULL,
		 SCU1_CLK_STOP, 0, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_LCLK1, CLK_GATE_ASPEED, "lclk1-gate", NULL,
		 SCU1_CLK_STOP, 1, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_ESPI0CLK, CLK_GATE_ASPEED, "espi0clk-gate", NULL,
		 SCU1_CLK_STOP, 2, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_ESPI1CLK, CLK_GATE_ASPEED, "espi1clk-gate", NULL,
		 SCU1_CLK_STOP, 3, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_SDCLK, CLK_GATE_ASPEED, "sdclk-gate", sdclk,
		 SCU1_CLK_STOP, 4, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_IPEREFCLK, CLK_GATE_ASPEED, "soc1-iperefclk-gate", NULL,
		 SCU1_CLK_STOP, 5, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_REFCLK, CLK_GATE_ASPEED, "soc1-refclk-gate", NULL,
		 SCU1_CLK_STOP, 6, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_LPCHCLK, CLK_GATE_ASPEED, "lpchclk-gate", NULL,
		 SCU1_CLK_STOP, 7, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_MAC0CLK, CLK_GATE_ASPEED, "mac0clk-gate", NULL,
		 SCU1_CLK_STOP, 8, 0),
	GATE_CLK(SCU1_CLK_GATE_MAC1CLK, CLK_GATE_ASPEED, "mac1clk-gate", NULL,
		 SCU1_CLK_STOP, 9, 0),
	GATE_CLK(SCU1_CLK_GATE_MAC2CLK, CLK_GATE_ASPEED, "mac2clk-gate", NULL,
		 SCU1_CLK_STOP, 10, 0),
	GATE_CLK(SCU1_CLK_GATE_UART0CLK, CLK_GATE_ASPEED, "uart0clk-gate", uart0clk,
		 SCU1_CLK_STOP, 11, 0),
	GATE_CLK(SCU1_CLK_GATE_UART1CLK, CLK_GATE_ASPEED, "uart1clk-gate", uart1clk,
		 SCU1_CLK_STOP, 12, 0),
	GATE_CLK(SCU1_CLK_GATE_UART2CLK, CLK_GATE_ASPEED, "uart2clk-gate", uart2clk,
		 SCU1_CLK_STOP, 13, 0),
	GATE_CLK(SCU1_CLK_GATE_UART3CLK, CLK_GATE_ASPEED, "uart3clk-gate", uart3clk,
		 SCU1_CLK_STOP, 14, 0),
	GATE_CLK(SCU1_CLK_GATE_I2CCLK, CLK_GATE_ASPEED, "i2cclk-gate", NULL, SCU1_CLK_STOP, 15, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C0CLK, CLK_GATE_ASPEED, "i3c0clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 16, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C1CLK, CLK_GATE_ASPEED, "i3c1clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 17, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C2CLK, CLK_GATE_ASPEED, "i3c2clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 18, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C3CLK, CLK_GATE_ASPEED, "i3c3clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 19, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C4CLK, CLK_GATE_ASPEED, "i3c4clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 20, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C5CLK, CLK_GATE_ASPEED, "i3c5clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 21, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C6CLK, CLK_GATE_ASPEED, "i3c6clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 22, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C7CLK, CLK_GATE_ASPEED, "i3c7clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 23, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C8CLK, CLK_GATE_ASPEED, "i3c8clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 24, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C9CLK, CLK_GATE_ASPEED, "i3c9clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 25, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C10CLK, CLK_GATE_ASPEED, "i3c10clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 26, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C11CLK, CLK_GATE_ASPEED, "i3c11clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 27, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C12CLK, CLK_GATE_ASPEED, "i3c12clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 28, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C13CLK, CLK_GATE_ASPEED, "i3c13clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 29, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C14CLK, CLK_GATE_ASPEED, "i3c14clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 30, 0),
	GATE_CLK(SCU1_CLK_GATE_I3C15CLK, CLK_GATE_ASPEED, "i3c15clk-gate", soc1_i3c,
		 SCU1_CLK_STOP, 31, 0),
	GATE_CLK(SCU1_CLK_GATE_UART5CLK, CLK_GATE_ASPEED, "uart5clk-gate", uart5clk,
		 SCU1_CLK_STOP2, 0, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_UART6CLK, CLK_GATE_ASPEED, "uart6clk-gate", uart6clk,
		 SCU1_CLK_STOP2, 1, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_UART7CLK, CLK_GATE_ASPEED, "uart7clk-gate", uart7clk,
		 SCU1_CLK_STOP2, 2, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_UART8CLK, CLK_GATE_ASPEED, "uart8clk-gate", uart8clk,
		 SCU1_CLK_STOP2, 3, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_UART9CLK, CLK_GATE_ASPEED, "uart9clk-gate", uart9clk,
		 SCU1_CLK_STOP2, 4, 0),
	GATE_CLK(SCU1_CLK_GATE_UART10CLK, CLK_GATE_ASPEED, "uart10clk-gate", uart10clk,
		 SCU1_CLK_STOP2, 5, 0),
	GATE_CLK(SCU1_CLK_GATE_UART11CLK, CLK_GATE_ASPEED, "uart11clk-gate", uart11clk,
		 SCU1_CLK_STOP2, 6, 0),
	GATE_CLK(SCU1_CLK_GATE_UART12CLK, CLK_GATE_ASPEED, "uart12clk-gate", uart12clk,
		 SCU1_CLK_STOP2, 7, 0),
	GATE_CLK(SCU1_CLK_GATE_FSICLK, CLK_GATE_ASPEED, "fsiclk-gate", NULL, SCU1_CLK_STOP2, 8, 0),
	GATE_CLK(SCU1_CLK_GATE_LTPIPHYCLK, CLK_GATE_ASPEED, "ltpiphyclk-gate", NULL,
		 SCU1_CLK_STOP2, 9, 0),
	GATE_CLK(SCU1_CLK_GATE_LTPICLK, CLK_GATE_ASPEED, "ltpiclk-gate", NULL,
		 SCU1_CLK_STOP2, 10, 0),
	GATE_CLK(SCU1_CLK_GATE_VGALCLK, CLK_GATE_ASPEED, "vgalclk-gate", NULL,
		 SCU1_CLK_STOP2, 11, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_UHCICLK, CLK_GATE_ASPEED, "usbuartclk-gate", NULL,
		 SCU1_CLK_STOP2, 12, 0),
	GATE_CLK(SCU1_CLK_GATE_CANCLK, CLK_GATE_ASPEED, "canclk-gate", canclk,
		 SCU1_CLK_STOP2, 13, 0),
	GATE_CLK(SCU1_CLK_GATE_PCICLK, CLK_GATE_ASPEED, "pciclk-gate", NULL,
		 SCU1_CLK_STOP2, 14, 0),
	GATE_CLK(SCU1_CLK_GATE_SLICLK, CLK_GATE_ASPEED, "soc1-sliclk-gate", NULL,
		 SCU1_CLK_STOP2, 15, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_E2MCLK, CLK_GATE_ASPEED, "soc1-e2m-gate", NULL,
		 SCU1_CLK_STOP2, 16, CLK_IS_CRITICAL),
	GATE_CLK(SCU1_CLK_GATE_PORTCUSB2CLK, CLK_GATE_ASPEED, "portcusb2-gate", NULL,
		 SCU1_CLK_STOP2, 17, 0),
	GATE_CLK(SCU1_CLK_GATE_PORTDUSB2CLK, CLK_GATE_ASPEED, "portdusb2-gate", NULL,
		 SCU1_CLK_STOP2, 18, 0),
	GATE_CLK(SCU1_CLK_GATE_LTPI1TXCLK, CLK_GATE_ASPEED, "ltp1tx-gate", NULL,
		 SCU1_CLK_STOP2, 19, 0),
};

static struct clk_hw *ast2700_clk_hw_register_hpll(void __iomem *reg,
						   const char *name, const char *parent_name,
						   struct ast2700_clk_ctrl *clk_ctrl)
{
	unsigned int mult, div;
	u32 val;

	val = readl(clk_ctrl->base + SCU0_HWSTRAP1);
	if ((readl(clk_ctrl->base) & REVISION_ID) && (val & BIT(3))) {
		switch ((val & GENMASK(4, 2)) >> 2) {
		case 2:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1800 * HZ_PER_MHZ);
		case 3:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1700 * HZ_PER_MHZ);
		case 6:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1200 * HZ_PER_MHZ);
		case 7:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 800 * HZ_PER_MHZ);
		default:
			return ERR_PTR(-EINVAL);
		}
	} else if ((val & GENMASK(3, 2)) != 0) {
		switch ((val & GENMASK(3, 2)) >> 2) {
		case 1:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1900 * HZ_PER_MHZ);
		case 2:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1800 * HZ_PER_MHZ);
		case 3:
			return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL,
							       0, 1700 * HZ_PER_MHZ);
		default:
			return ERR_PTR(-EINVAL);
		}
	} else {
		val = readl(reg);

		if (val & BIT(24)) {
			/* Pass through mode */
			mult = 1;
			div = 1;
		} else {
			u32 m = val & 0x1fff;
			u32 n = (val >> 13) & 0x3f;
			u32 p = (val >> 19) & 0xf;

			mult = (m + 1) / (2 * (n + 1));
			div = (p + 1);
		}
	}

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, name, parent_name, 0, mult, div);
}

static struct clk_hw *ast2700_clk_hw_register_pll(int clk_idx, void __iomem *reg,
						  const char *name, const char *parent_name,
						  struct ast2700_clk_ctrl *clk_ctrl)
{
	int scu = clk_ctrl->clk_data->scu;
	unsigned int mult, div;
	u32 val = readl(reg);

	if (val & BIT(24)) {
		/* Pass through mode */
		mult = 1;
		div = 1;
	} else {
		u32 m = val & 0x1fff;
		u32 n = (val >> 13) & 0x3f;
		u32 p = (val >> 19) & 0xf;

		if (scu) {
			mult = (m + 1) / (n + 1);
			div = (p + 1);
		} else {
			if (clk_idx == SCU0_CLK_MPLL) {
				mult = m / (n + 1);
				div = (p + 1);
			} else {
				mult = (m + 1) / (2 * (n + 1));
				div = (p + 1);
			}
		}
	}

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, name, parent_name, 0, mult, div);
}

static struct clk_hw *ast2700_clk_hw_register_dclk(void __iomem *reg, const char *name,
						   struct ast2700_clk_ctrl *clk_ctrl)
{
	unsigned int mult, div, r, n;
	u32 xdclk;
	u32 val;

	val = readl(clk_ctrl->base + 0x284);
	if (val & BIT(29))
		xdclk = 800 * HZ_PER_MHZ;
	else
		xdclk = 1000 * HZ_PER_MHZ;

	val = readl(reg);
	r = val & GENMASK(15, 0);
	n = (val >> 16) & GENMASK(15, 0);
	mult = r;
	div = 2 * n;

	return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, name, NULL, 0, (xdclk * mult) / div);
}

static struct clk_hw *ast2700_clk_hw_register_uartpll(void __iomem *reg,
						      const char *name, const char *parent_name,
						      struct ast2700_clk_ctrl *clk_ctrl)
{
	unsigned int mult, div;
	u32 val = readl(reg);
	u32 r = val & 0xff;
	u32 n = (val >> 8) & 0x3ff;

	mult = r;
	div = n * 2;

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, name,
						 parent_name, 0, mult, div);
}

static struct clk_hw *ast2700_clk_hw_register_misc(int clk_idx, void __iomem *reg,
						   const char *name, const char *parent_name,
						   struct ast2700_clk_ctrl *clk_ctrl)
{
	u32 div = 0;

	if (clk_idx == SCU0_CLK_MPHY) {
		div = readl(reg) + 1;
	} else if (clk_idx == SCU0_CLK_U2PHY_REFCLK) {
		if (readl(clk_ctrl->base) & REVISION_ID)
			div = (GET_USB_REFCLK_DIV(readl(reg)) + 1) << 4;
		else
			div = (GET_USB_REFCLK_DIV(readl(reg)) + 1) << 1;
	} else {
		return ERR_PTR(-EINVAL);
	}

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, name,
						   parent_name, 0, 1, div);
}

static int ast2700_clk_is_enabled(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx);
	u32 reg;

	reg = readl(gate->reg);

	return !(reg & clk);
}

static int ast2700_clk_enable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx);

	if (readl(gate->reg) & clk)
		writel(clk, gate->reg + 0x04);

	return 0;
}

static void ast2700_clk_disable(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	u32 clk = BIT(gate->bit_idx);

	/* Clock is set to enable, so use write to set register */
	writel(clk, gate->reg);
}

static const struct clk_ops ast2700_clk_gate_ops = {
	.enable = ast2700_clk_enable,
	.disable = ast2700_clk_disable,
	.is_enabled = ast2700_clk_is_enabled,
};

static struct clk_hw *ast2700_clk_hw_register_gate(struct device *dev, const char *name,
						   const struct clk_parent_data	*parent,
						   void __iomem *reg, u8 clock_idx,
						   unsigned long clk_gate_flags, spinlock_t *lock)
{
	struct clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret = -EINVAL;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &ast2700_clk_gate_ops;
	init.flags = clk_gate_flags;
	init.parent_names = parent ? &parent->name : NULL;
	init.num_parents = parent ? 1 : 0;

	gate->reg = reg;
	gate->bit_idx = clock_idx;
	gate->flags = 0;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}

static void ast2700_soc1_configure_mac01_clk(struct ast2700_clk_ctrl *clk_ctrl)
{
	struct device_node *np = clk_ctrl->dev->of_node;
	struct mac_delay_config mac_cfg;
	u32 reg[3];
	int ret;

	if (readl(clk_ctrl->base + SCU1_REVISION_ID) & REVISION_ID)
		reg[0] = AST2700_DEF_MAC12_DELAY_1G_A1;
	else
		reg[0] = AST2700_DEF_MAC12_DELAY_1G_A0;
	reg[1] = AST2700_DEF_MAC12_DELAY_100M;
	reg[2] = AST2700_DEF_MAC12_DELAY_10M;

	ret = of_property_read_u32_array(np, "mac0-clk-delay", (u32 *)&mac_cfg,
					 sizeof(mac_cfg) / sizeof(u32));
	if (!ret) {
		reg[0] &= ~(MAC_CLK_1G_INPUT_DELAY_1 | MAC_CLK_1G_OUTPUT_DELAY_1);
		reg[0] |= FIELD_PREP(MAC_CLK_1G_INPUT_DELAY_1, mac_cfg.rx_delay_1000) |
			  FIELD_PREP(MAC_CLK_1G_OUTPUT_DELAY_1, mac_cfg.tx_delay_1000);

		reg[1] &= ~(MAC_CLK_100M_10M_INPUT_DELAY_1 | MAC_CLK_100M_10M_OUTPUT_DELAY_1);
		reg[1] |= FIELD_PREP(MAC_CLK_100M_10M_INPUT_DELAY_1, mac_cfg.rx_delay_100) |
			  FIELD_PREP(MAC_CLK_100M_10M_OUTPUT_DELAY_1, mac_cfg.tx_delay_100);

		reg[2] &= ~(MAC_CLK_100M_10M_INPUT_DELAY_1 | MAC_CLK_100M_10M_OUTPUT_DELAY_1);
		reg[2] |= FIELD_PREP(MAC_CLK_100M_10M_INPUT_DELAY_1, mac_cfg.rx_delay_10) |
			  FIELD_PREP(MAC_CLK_100M_10M_OUTPUT_DELAY_1, mac_cfg.tx_delay_10);
	}

	ret = of_property_read_u32_array(np, "mac1-clk-delay", (u32 *)&mac_cfg,
					 sizeof(mac_cfg) / sizeof(u32));
	if (!ret) {
		reg[0] &= ~(MAC_CLK_1G_INPUT_DELAY_2 | MAC_CLK_1G_OUTPUT_DELAY_2);
		reg[0] |= FIELD_PREP(MAC_CLK_1G_INPUT_DELAY_2, mac_cfg.rx_delay_1000) |
			  FIELD_PREP(MAC_CLK_1G_OUTPUT_DELAY_2, mac_cfg.tx_delay_1000);

		reg[1] &= ~(MAC_CLK_100M_10M_INPUT_DELAY_2 | MAC_CLK_100M_10M_OUTPUT_DELAY_2);
		reg[1] |= FIELD_PREP(MAC_CLK_100M_10M_INPUT_DELAY_2, mac_cfg.rx_delay_100) |
			  FIELD_PREP(MAC_CLK_100M_10M_OUTPUT_DELAY_2, mac_cfg.tx_delay_100);

		reg[2] &= ~(MAC_CLK_100M_10M_INPUT_DELAY_2 | MAC_CLK_100M_10M_OUTPUT_DELAY_2);
		reg[2] |= FIELD_PREP(MAC_CLK_100M_10M_INPUT_DELAY_2, mac_cfg.rx_delay_10) |
			  FIELD_PREP(MAC_CLK_100M_10M_OUTPUT_DELAY_2, mac_cfg.tx_delay_10);
	}

	reg[0] |= (readl(clk_ctrl->base + SCU1_MAC12_CLK_DLY) & ~GENMASK(25, 0));
	writel(reg[0], clk_ctrl->base + SCU1_MAC12_CLK_DLY);
	writel(reg[1], clk_ctrl->base + SCU1_MAC12_CLK_DLY_100M);
	writel(reg[2], clk_ctrl->base + SCU1_MAC12_CLK_DLY_10M);
}

static void ast2700_soc1_configure_i3c_clk(struct ast2700_clk_ctrl *clk_ctrl)
{
	if (readl(clk_ctrl->base + SCU1_REVISION_ID) & REVISION_ID)
		/* I3C 250MHz = HPLL/4 */
		writel((readl(clk_ctrl->base + SCU1_CLK_SEL2) &
			~SCU1_CLK_I3C_DIV_MASK) |
			       FIELD_PREP(SCU1_CLK_I3C_DIV_MASK,
					  SCU1_CLK_I3C_DIV(4)),
		       clk_ctrl->base + SCU1_CLK_SEL2);
}

static int ast2700_soc_clk_probe(struct platform_device *pdev)
{
	const struct ast2700_clk_data *clk_data;
	struct ast2700_clk_ctrl *clk_ctrl;
	struct clk_hw_onecell_data *clk_hw_data;
	struct device *dev = &pdev->dev;
	u32 uart_clk_source = 0;
	void __iomem *clk_base;
	struct clk_hw **hws;
	char *reset_name;
	int ret;
	int i;

	clk_ctrl = devm_kzalloc(dev, sizeof(*clk_ctrl), GFP_KERNEL);
	if (!clk_ctrl)
		return -ENOMEM;
	clk_ctrl->dev = dev;
	dev_set_drvdata(&pdev->dev, clk_ctrl);

	spin_lock_init(&clk_ctrl->lock);

	clk_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(clk_base))
		return PTR_ERR(clk_base);

	clk_ctrl->base = clk_base;

	clk_data = (struct ast2700_clk_data *)device_get_match_data(dev);
	if (!clk_data)
		return -ENODEV;

	clk_ctrl->clk_data = clk_data;
	reset_name = devm_kasprintf(dev, GFP_KERNEL, "reset%d", clk_data->scu);

	clk_hw_data = devm_kzalloc(dev, struct_size(clk_hw_data, hws, clk_data->nr_clks),
				   GFP_KERNEL);
	if (!clk_hw_data)
		return -ENOMEM;

	clk_hw_data->num = clk_data->nr_clks;
	hws = clk_hw_data->hws;

	if (clk_data->scu) {
		of_property_read_u32(dev->of_node, "uart-clk-source", &uart_clk_source);
		if (uart_clk_source) {
			u32 val = readl(clk_base + SCU1_CLK_SEL1) & ~GENMASK(12, 0);

			uart_clk_source &= GENMASK(12, 0);
			writel(val | uart_clk_source, clk_base + SCU1_CLK_SEL1);
		}

		ast2700_soc1_configure_mac01_clk(clk_ctrl);
		ast2700_soc1_configure_i3c_clk(clk_ctrl);
	}

	for (i = 0; i < clk_data->nr_clks; i++) {
		const struct ast2700_clk_info *clk = &clk_data->clk_info[i];
		void __iomem *reg;

		if (clk->type == CLK_FIXED) {
			const struct ast2700_clk_fixed_rate_data *fixed_rate = &clk->data.rate;

			hws[i] = devm_clk_hw_register_fixed_rate(dev, clk->name, NULL, 0,
								 fixed_rate->fixed_rate);
		} else if (clk->type == CLK_FIXED_FACTOR) {
			const struct ast2700_clk_fixed_factor_data *factor = &clk->data.factor;

			hws[i] = devm_clk_hw_register_fixed_factor(dev, clk->name,
								   factor->parent->name,
								   0, factor->mult, factor->div);
		} else if (clk->type == DCLK_FIXED) {
			const struct ast2700_clk_pll_data *pll = &clk->data.pll;

			reg = clk_ctrl->base + pll->reg;
			hws[i] = ast2700_clk_hw_register_dclk(reg, clk->name, clk_ctrl);
		} else if (clk->type == CLK_HPLL) {
			const struct ast2700_clk_pll_data *pll = &clk->data.pll;

			reg = clk_ctrl->base + pll->reg;
			hws[i] = ast2700_clk_hw_register_hpll(reg, clk->name,
							      pll->parent->name, clk_ctrl);
		} else if (clk->type == CLK_PLL) {
			const struct ast2700_clk_pll_data *pll = &clk->data.pll;

			reg = clk_ctrl->base + pll->reg;
			hws[i] = ast2700_clk_hw_register_pll(i, reg, clk->name,
							     pll->parent->name, clk_ctrl);
		} else if (clk->type == CLK_UART_PLL) {
			const struct ast2700_clk_pll_data *pll = &clk->data.pll;

			reg = clk_ctrl->base + pll->reg;
			hws[i] = ast2700_clk_hw_register_uartpll(reg, clk->name,
								 pll->parent->name, clk_ctrl);
		} else if (clk->type == CLK_MUX) {
			const struct ast2700_clk_mux_data *mux = &clk->data.mux;

			reg = clk_ctrl->base + mux->reg;
			hws[i] = devm_clk_hw_register_mux_parent_data_table(dev, clk->name,
									    mux->parents,
									    mux->num_parents, 0,
									    reg, mux->bit_shift,
									    mux->bit_width, 0,
									    NULL, &clk_ctrl->lock);
		} else if (clk->type == CLK_MISC) {
			const struct ast2700_clk_pll_data *misc = &clk->data.pll;

			reg = clk_ctrl->base + misc->reg;
			hws[i] = ast2700_clk_hw_register_misc(i, reg, clk->name,
							      misc->parent->name, clk_ctrl);
		} else if (clk->type == CLK_DIVIDER) {
			const struct ast2700_clk_div_data *div = &clk->data.div;

			reg = clk_ctrl->base + div->reg;
			hws[i] = devm_clk_hw_register_divider_table(dev, clk->name,
								    div->parent->name, 0,
								    reg, div->bit_shift,
								    div->bit_width, 0,
								    div->div_table,
								    &clk_ctrl->lock);
		} else if (clk->type == CLK_GATE_ASPEED) {
			const struct ast2700_clk_gate_data *gate = &clk->data.gate;

			reg = clk_ctrl->base + gate->reg;
			hws[i] = ast2700_clk_hw_register_gate(dev, clk->name, gate->parent,
							      reg, gate->bit, gate->flags, 0);

		} else {
			const struct ast2700_clk_gate_data *gate = &clk->data.gate;

			reg = clk_ctrl->base + gate->reg;
			hws[i] = devm_clk_hw_register_gate_parent_data(dev, clk->name, gate->parent,
								       0, reg, clk->clk_idx, 0,
								       &clk_ctrl->lock);
		}

		if (IS_ERR(hws[i]))
			return PTR_ERR(hws[i]);
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_hw_data);
	if (ret)
		return ret;

	return aspeed_reset_controller_register(dev, clk_base, reset_name);
}

static const struct ast2700_clk_data ast2700_clk0_data = {
	.scu = 0,
	.nr_clks = ARRAY_SIZE(ast2700_scu0_clk_info),
	.clk_info = ast2700_scu0_clk_info,
};

static const struct ast2700_clk_data ast2700_clk1_data = {
	.scu = 1,
	.nr_clks = ARRAY_SIZE(ast2700_scu1_clk_info),
	.clk_info = ast2700_scu1_clk_info,
};

static const struct of_device_id ast2700_scu_match[] = {
	{ .compatible = "aspeed,ast2700-scu0", .data = &ast2700_clk0_data },
	{ .compatible = "aspeed,ast2700-scu1", .data = &ast2700_clk1_data },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ast2700_scu_match);

static struct platform_driver ast2700_scu_driver = {
	.probe = ast2700_soc_clk_probe,
	.driver = {
		.name = "clk-ast2700",
		.of_match_table = ast2700_scu_match,
	},
};

static int __init clk_ast2700_init(void)
{
	return platform_driver_register(&ast2700_scu_driver);
}
arch_initcall(clk_ast2700_init);
