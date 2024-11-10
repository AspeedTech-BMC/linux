// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 ASPEED Technology Inc.
 * Author: Ryan Chen <ryan_chen@aspeedtech.com>
 */

#include <linux/auxiliary_bus.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/units.h>

#include <dt-bindings/clock/aspeed,ast2700-scu.h>
#include <soc/aspeed/reset-aspeed.h>

#define SCU_CLK_12MHZ (12 * HZ_PER_MHZ)
#define SCU_CLK_24MHZ (24 * HZ_PER_MHZ)
#define SCU_CLK_25MHZ (25 * HZ_PER_MHZ)
#define SCU_CLK_192MHZ (192 * HZ_PER_MHZ)

/* SOC0 */
#define SCU0_HWSTRAP1 0x010
#define SCU0_CLK_STOP 0x240
#define SCU0_CLK_SEL1 0x280
#define SCU0_CLK_SEL2 0x284
#define GET_USB_REFCLK_DIV(x) ((GENMASK(23, 20) & (x)) >> 20)
#define UART_DIV13_EN BIT(30)
#define SCU0_HPLL_PARAM 0x300
#define SCU0_DPLL_PARAM 0x308
#define SCU0_MPLL_PARAM 0x310
#define SCU0_D0CLK_PARAM 0x320
#define SCU0_D1CLK_PARAM 0x330
#define SCU0_CRT0CLK_PARAM 0x340
#define SCU0_CRT1CLK_PARAM 0x350
#define SCU0_MPHYCLK_PARAM 0x360

/* SOC1 */
#define SCU1_REVISION_ID 0x0
#define REVISION_ID GENMASK(23, 16)
#define SCU1_CLK_STOP 0x240
#define SCU1_CLK_STOP2 0x260
#define SCU1_CLK_SEL1 0x280
#define SCU1_CLK_SEL2 0x284
#define UXCLK_MASK GENMASK(1, 0)
#define HUXCLK_MASK GENMASK(4, 3)
#define SCU1_HPLL_PARAM 0x300
#define SCU1_APLL_PARAM 0x310
#define SCU1_DPLL_PARAM 0x320
#define SCU1_UXCLK_CTRL 0x330
#define SCU1_HUXCLK_CTRL 0x334
#define SCU1_MAC12_CLK_DLY 0x390
#define SCU1_MAC12_CLK_DLY_100M 0x394
#define SCU1_MAC12_CLK_DLY_10M 0x398

/*
 * MAC Clock Delay settings
 */
#define MAC_CLK_RMII1_50M_RCLK_O_CTRL		BIT(30)
#define   MAC_CLK_RMII1_50M_RCLK_O_DIS		0
#define   MAC_CLK_RMII1_50M_RCLK_O_EN		1
#define MAC_CLK_RMII0_50M_RCLK_O_CTRL		BIT(29)
#define   MAC_CLK_RMII0_5M_RCLK_O_DIS		0
#define   MAC_CLK_RMII0_5M_RCLK_O_EN		1
#define MAC_CLK_RMII_TXD_FALLING_2		BIT(27)
#define MAC_CLK_RMII_TXD_FALLING_1		BIT(26)
#define MAC_CLK_RXCLK_INV_2			BIT(25)
#define MAC_CLK_RXCLK_INV_1			BIT(24)
#define MAC_CLK_1G_INPUT_DELAY_2		GENMASK(23, 18)
#define MAC_CLK_1G_INPUT_DELAY_1		GENMASK(17, 12)
#define MAC_CLK_1G_OUTPUT_DELAY_2		GENMASK(11, 6)
#define MAC_CLK_1G_OUTPUT_DELAY_1		GENMASK(5, 0)

#define MAC_CLK_100M_10M_RESERVED		GENMASK(31, 26)
#define MAC_CLK_100M_10M_RXCLK_INV_2		BIT(25)
#define MAC_CLK_100M_10M_RXCLK_INV_1		BIT(24)
#define MAC_CLK_100M_10M_INPUT_DELAY_2		GENMASK(23, 18)
#define MAC_CLK_100M_10M_INPUT_DELAY_1		GENMASK(17, 12)
#define MAC_CLK_100M_10M_OUTPUT_DELAY_2		GENMASK(11, 6)
#define MAC_CLK_100M_10M_OUTPUT_DELAY_1		GENMASK(5, 0)

#define AST2700_DEF_MAC12_DELAY_1G	0x00CF4D75
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

enum {
	CLK_MUX,
	CLK_PLL,
	CLK_GATE,
	CLK_MISC,
	CLK_FIXED,
	CLK_DIVIDER,
	CLK_UART_PLL,
	CLK_DIV_TABLE,
	CLK_FIXED_FACTOR,
	CLK_GATE_ASPEED,
};

struct ast2700_clk_info {
	const char *name;
	const char * const *parent_names;
	const struct clk_div_table *div_table;
	unsigned long fixed_rate;
	unsigned int mult;
	unsigned int div;
	u32 reg;
	u32 flags;
	u32 type;
	u8 clk_idx;
	u8 bit_shift;
	u8 bit_width;
	u8 num_parents;
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

static const struct clk_div_table ast2700_clk_uart_div_table[] = {
	{ 0x0, 1 },
	{ 0x1, 13 },
	{ 0 }
};

static const struct ast2700_clk_info ast2700_scu0_clk_info[] __initconst = {
	[SCU0_CLKIN] = {
		.type = CLK_FIXED,
		.name = "soc0-clkin",
		.fixed_rate = SCU_CLK_25MHZ,
	},
	[SCU0_CLK_24M] = {
		.type = CLK_FIXED,
		.name = "soc0-clk24Mhz",
		.fixed_rate = SCU_CLK_24MHZ,
	},
	[SCU0_CLK_192M] = {
		.type = CLK_FIXED,
		.name = "soc0-clk192Mhz",
		.fixed_rate = SCU_CLK_192MHZ,
	},
	[SCU0_CLK_HPLL] = {
		.type = CLK_PLL,
		.name = "soc0-hpll",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_HPLL_PARAM,
	},
	[SCU0_CLK_HPLL_DIV2] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc0-hpll_div2",
		.parent_names = (const char *[]){ "soc0-hpll", },
		.mult = 1,
		.div = 2,
	},
	[SCU0_CLK_HPLL_DIV4] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc0-hpll_div4",
		.parent_names = (const char *[]){ "soc0-hpll", },
		.mult = 1,
		.div = 4,
	},
	[SCU0_CLK_HPLL_DIV_AHB] = {
		.type = CLK_DIV_TABLE,
		.name = "soc0-hpll_div_ahb",
		.parent_names = (const char *[]){ "soc0-hpll", },
		.reg = SCU0_HWSTRAP1,
		.bit_shift = 5,
		.bit_width = 2,
		.div_table = ast2700_clk_div_table,
	},
	[SCU0_CLK_DPLL] = {
		.type = CLK_PLL,
		.name = "dpll",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_DPLL_PARAM,
	},
	[SCU0_CLK_MPLL] = {
		.type = CLK_PLL,
		.name = "soc0-mpll",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_MPLL_PARAM,
	},
	[SCU0_CLK_MPLL_DIV2] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc0-mpll_div2",
		.parent_names = (const char *[]){ "soc0-mpll", },
		.mult = 1,
		.div = 2,
	},
	[SCU0_CLK_MPLL_DIV4] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc0-mpll_div4",
		.parent_names = (const char *[]){ "soc0-mpll", },
		.mult = 1,
		.div = 4,
	},
	[SCU0_CLK_MPLL_DIV8] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc0-mpll_div8",
		.parent_names = (const char *[]){ "soc0-mpll", },
		.mult = 1,
		.div = 8,
	},
	[SCU0_CLK_MPLL_DIV_AHB] = {
		.type = CLK_DIV_TABLE,
		.name = "soc0-mpll_div_ahb",
		.parent_names = (const char *[]){ "soc0-mpll", },
		.reg = SCU0_HWSTRAP1,
		.bit_shift = 5,
		.bit_width = 2,
		.div_table = ast2700_clk_div_table,
	},
	[SCU0_CLK_D0] = {
		.type = CLK_PLL,
		.name = "d0clk",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_D0CLK_PARAM,
	},
	[SCU0_CLK_D1] = {
		.type = CLK_PLL,
		.name = "d1clk",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_D1CLK_PARAM,
	},
	[SCU0_CLK_CRT0] = {
		.type = CLK_PLL,
		.name = "crt0clk",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_CRT0CLK_PARAM,
	},
	[SCU0_CLK_CRT1] = {
		.type = CLK_PLL,
		.name = "crt1clk",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_CRT1CLK_PARAM,
	},
	[SCU0_CLK_MPHY] = {
		.type = CLK_MISC,
		.name = "mphyclk",
		.parent_names = (const char *[]){ "soc0-hpll", },
		.reg = SCU0_MPHYCLK_PARAM,
	},
	[SCU0_CLK_PSP] = {
		.type = CLK_MUX,
		.name = "pspclk",
		.parent_names = (const char *[]){"soc0-mpll", "soc0-hpll", },
		.num_parents = 2,
		.reg = SCU0_HWSTRAP1,
		.bit_shift = 4,
		.bit_width = 1,
	},
	[SCU0_CLK_AXI0] = {
		.type = CLK_FIXED_FACTOR,
		.name = "axi0clk",
		.parent_names = (const char *[]){"pspclk", },
		.mult = 1,
		.div = 2,
	},
	[SCU0_CLK_AHB] = {
		.type = CLK_MUX,
		.name = "soc0-ahb",
		.parent_names = (const char *[]){"soc0-mpll_div_ahb", "soc0-hpll_div_ahb", },
		.num_parents = 2,
		.reg = SCU0_HWSTRAP1,
		.bit_shift = 7,
		.bit_width = 1,
	},
	[SCU0_CLK_AXI1] = {
		.type = CLK_FIXED_FACTOR,
		.name = "axi1clk",
		.parent_names = (const char *[]){ "soc0-ahb", },
		.mult = 1,
		.div = 2,
	},
	[SCU0_CLK_APB] = {
		.type = CLK_DIV_TABLE,
		.name = "soc0-apb",
		.parent_names = (const char *[]){ "axi0clk", },
		.reg = SCU0_CLK_SEL1,
		.bit_shift = 23,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table2,
	},
	[SCU0_CLK_GATE_MCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "mclk-gate",
		.parent_names = (const char *[]){ "soc0-mpll", },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 0,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_ECLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "eclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 1,
	},
	[SCU0_CLK_GATE_2DCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "gclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 2,
	},
	[SCU0_CLK_GATE_VCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "vclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 3,
	},
	[SCU0_CLK_GATE_BCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "bclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 4,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_VGA0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "vga0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 5,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_REFCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc0-refclk-gate",
		.parent_names = (const char *[]){ "soc0-clkin", },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 6,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_U2PHY_REFCLK] = {
		.type = CLK_MISC,
		.name = "u2phy_refclk",
		.parent_names = (const char *[]){ "soc0-mpll_div8", },
		.reg = SCU0_CLK_SEL2,
	},
	[SCU0_CLK_U2PHY_CLK12M] = {
		.type = CLK_FIXED,
		.name = "u2phy_clk12m",
		.parent_names = (const char *[]){  },
		.fixed_rate = SCU_CLK_12MHZ,
	},
	[SCU0_CLK_GATE_PORTBUSB2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "portb-usb2clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 7,
	},
	[SCU0_CLK_GATE_UHCICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uhciclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 9,
	},
	[SCU0_CLK_GATE_VGA1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "vga1clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 10,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_DDRPHYCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "ddrphy-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 11,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_E2M0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "e2m0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 12,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_HACCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "hacclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 13,
	},
	[SCU0_CLK_GATE_PORTAUSB2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "porta-usb2clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 14,
	},
	[SCU0_CLK_UART] = {
		.type = CLK_MUX,
		.name = "soc0-uartclk",
		.parent_names = (const char *[]){"soc0-clk24Mhz", "soc0-clk192Mhz", },
		.num_parents = 2,
		.reg = SCU0_CLK_SEL2,
		.bit_shift = 14,
		.bit_width = 1,
	},
	[SCU0_CLK_UART4] = {
		.type = CLK_DIV_TABLE,
		.name = "uart4clk",
		.parent_names = (const char *[]){ "soc0-uartclk", },
		.reg = SCU0_CLK_SEL2,
		.bit_shift = 30,
		.bit_width = 1,
		.div_table = ast2700_clk_uart_div_table,
	},
	[SCU0_CLK_GATE_UART4CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart4clk-gate",
		.parent_names = (const char *[]){"uart4clk" },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 15,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_SLICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc0-sliclk-gate",
		.parent_names = (const char *[]){	},
		.reg = SCU0_CLK_STOP,
		.clk_idx = 16,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_DACCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "dacclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 17,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_DP] = {
		.type = CLK_GATE_ASPEED,
		.name = "dpclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 18,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_E2M1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "e2m1clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 19,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU0_CLK_GATE_CRT0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "crt0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 20,
	},
	[SCU0_CLK_GATE_CRT1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "crt1clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 21,
	},
	[SCU0_CLK_GATE_ECDSACLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "eccclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 23,
	},
	[SCU0_CLK_GATE_RSACLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "rsaclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 24,
	},
	[SCU0_CLK_GATE_RVAS0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "rvas0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 25,
	},
	[SCU0_CLK_GATE_UFSCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "ufsclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 26,
	},
	[SCU0_CLK_EMMCMUX] = {
		.type = CLK_MUX,
		.name = "emmcsrc-mux",
		.parent_names = (const char *[]){"soc0-mpll_div4", "soc0-hpll_div4", },
		.num_parents = 2,
		.reg = SCU0_CLK_SEL1,
		.bit_shift = 11,
		.bit_width = 1,
	},
	[SCU0_CLK_EMMC] = {
		.type = CLK_DIV_TABLE,
		.name = "emmcclk",
		.parent_names = (const char *[]){ "emmcsrc-mux", },
		.reg = SCU0_CLK_SEL1,
		.bit_shift = 12,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table2,
	},
	[SCU0_CLK_GATE_EMMCCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "emmcclk-gate",
		.parent_names = (const char *[]){ "emmcclk", },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 27,
	},
	[SCU0_CLK_GATE_RVAS1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "rvas1clk-gate",
		.parent_names = (const char *[]){ "emmcclk", },
		.reg = SCU0_CLK_STOP,
		.clk_idx = 28,
	},
};

static const struct ast2700_clk_info ast2700_scu1_clk_info[] __initconst = {
	[SCU1_CLKIN] = {
		.type = CLK_FIXED,
		.name = "soc1-clkin",
		.fixed_rate = SCU_CLK_25MHZ,
	},
	[SCU1_CLK_HPLL] = {
		.type = CLK_PLL,
		.name = "soc1-hpll",
		.parent_names = (const char *[]){ "soc1-clkin", },
		.reg = SCU1_HPLL_PARAM,
	},
	[SCU1_CLK_APLL] = {
		.type = CLK_PLL,
		.name = "soc1-apll",
		.parent_names = (const char *[]){ "soc1-clkin", },
		.reg = SCU1_APLL_PARAM,
	},
	[SCU1_CLK_APLL_DIV2] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc1-apll_div2",
		.parent_names = (const char *[]){ "soc1-apll", },
		.mult = 1,
		.div = 2,
	},
	[SCU1_CLK_APLL_DIV4] = {
		.type = CLK_FIXED_FACTOR,
		.name = "soc1-apll_div4",
		.parent_names = (const char *[]){ "soc1-apll", },
		.mult = 1,
		.div = 4,
	},
	[SCU1_CLK_DPLL] = {
		.type = CLK_PLL,
		.name = "soc1-dpll",
		.parent_names = (const char *[]){ "soc1-clkin", },
		.reg = SCU1_DPLL_PARAM,
	},
	[SCU1_CLK_UXCLK] = {
		.type = CLK_MUX,
		.name = "uxclk",
		.parent_names = (const char *[]){ "soc1-apll_div4", "soc1-apll_div2",
						 "soc1-apll", "soc1-hpll",},
		.num_parents = 4,
		.reg = SCU1_CLK_SEL2,
		.bit_shift = 0,
		.bit_width = 2,
	},
	[SCU1_CLK_UARTX] = {
		.type = CLK_UART_PLL,
		.name = "uartxclk",
		.parent_names = (const char *[]){ "uxclk", },
		.reg = SCU1_UXCLK_CTRL,
	},
	[SCU1_CLK_HUXCLK] = {
		.type = CLK_MUX,
		.name = "huxclk",
		.parent_names = (const char *[]){"soc1-apll_div4", "soc1-apll_div2",
						 "soc1-apll", "soc1-hpll",},
		.num_parents = 4,
		.reg = SCU1_CLK_SEL2,
		.bit_shift = 3,
		.bit_width = 2,
	},
	[SCU1_CLK_HUARTX] = {
		.type = CLK_UART_PLL,
		.name = "huartxclk",
		.parent_names = (const char *[]){ "huxclk", },
		.reg = SCU1_HUXCLK_CTRL,
	},
	[SCU1_CLK_AHB] = {
		.type = CLK_DIV_TABLE,
		.name = "soc1-ahb",
		.parent_names = (const char *[]){"soc1-hpll", },
		.reg = SCU1_CLK_SEL2,
		.bit_shift = 20,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table,
	},
	[SCU1_CLK_APB] = {
		.type = CLK_DIV_TABLE,
		.name = "soc1-apb",
		.parent_names = (const char *[]){"soc1-hpll", },
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 18,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table2,
	},
	[SCU1_CLK_RMII] = {
		.type = CLK_DIV_TABLE,
		.name = "rmii",
		.parent_names = (const char *[]){"soc1-hpll", },
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 21,
		.bit_width = 3,
		.div_table = ast2700_rmii_div_table,
	},
	[SCU1_CLK_MAC0RCLK] = {
		.type = CLK_GATE,
		.name = "mac0rclk-gate",
		.parent_names = (const char *[]){ "rmii", },
		.reg = SCU1_MAC12_CLK_DLY,
		.clk_idx = 29,
	},
	[SCU1_CLK_MAC1RCLK] = {
		.type = CLK_GATE,
		.name = "mac1rclk-gate",
		.parent_names = (const char *[]){ "rmii", },
		.reg = SCU1_MAC12_CLK_DLY,
		.clk_idx = 30,
	},
	[SCU1_CLK_RGMII] = {
		.type = CLK_DIV_TABLE,
		.name = "rgmii",
		.parent_names = (const char *[]){"soc1-hpll", },
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 25,
		.bit_width = 3,
		.div_table = ast2700_rgmii_div_table,
	},
	[SCU1_CLK_MACHCLK] = {
		.type = CLK_DIV_TABLE,
		.name = "machclk",
		.parent_names = (const char *[]){"soc1-hpll", },
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 29,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table,
	},
	[SCU1_CLK_GATE_LCLK0] = {
		.type = CLK_GATE_ASPEED,
		.name = "lclk0-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 0,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_LCLK1] = {
		.type = CLK_GATE_ASPEED,
		.name = "lclk1-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 1,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_ESPI0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "espi0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 2,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_ESPI1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "espi1clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 3,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_APLL_DIVN] = {
		.type = CLK_DIV_TABLE,
		.name = "soc1-apll_divn",
		.parent_names = (const char *[]){"soc1-apll", },
		.reg = SCU1_CLK_SEL2,
		.bit_shift = 8,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table,
	},
	[SCU1_CLK_SDMUX] = {
		.type = CLK_MUX,
		.name = "sdclk-mux",
		.parent_names = (const char *[]){ "soc1-hpll", "soc1-apll", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 13,
		.bit_width = 1,
	},
	[SCU1_CLK_SDCLK] = {
		.type = CLK_DIV_TABLE,
		.name = "sdclk",
		.parent_names = (const char *[]){"sdclk-mux", },
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 14,
		.bit_width = 3,
		.div_table = ast2700_clk_div_table,
	},
	[SCU1_CLK_GATE_SDCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "sdclk-gate",
		.parent_names = (const char *[]){"sdclk", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 4,
	},
	[SCU1_CLK_GATE_IPEREFCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc1-iperefclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 5,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_REFCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc1-refclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 6,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_LPCHCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "lpchclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 7,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_MAC0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "mac0clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 8,
	},
	[SCU1_CLK_GATE_MAC1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "mac1clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 9,
	},
	[SCU1_CLK_GATE_MAC2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "mac2clk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 10,
	},
	[SCU1_CLK_UART0] = {
		.type = CLK_MUX,
		.name = "uart0clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 0,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart0clk-gate",
		.parent_names = (const char *[]){ "uart0clk", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 11,
	},
	[SCU1_CLK_UART1] = {
		.type = CLK_MUX,
		.name = "uart1clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 1,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart1clk-gate",
		.parent_names = (const char *[]){ "uart1clk", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 12,
	},
	[SCU1_CLK_UART2] = {
		.type = CLK_MUX,
		.name = "uart2clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 2,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart2clk-gate",
		.parent_names = (const char *[]){ "uart2clk", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 13,
	},
	[SCU1_CLK_UART3] = {
		.type = CLK_MUX,
		.name = "uart3clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 3,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART3CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart3clk-gate",
		.parent_names = (const char *[]){ "uart3clk", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 14,
	},
	[SCU1_CLK_GATE_I2CCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i2cclk-gate",
		.parent_names = (const char *[]){	},
		.reg = SCU1_CLK_STOP,
		.clk_idx = 15,
	},
	[SCU1_CLK_GATE_I3C0CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c0clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 16,
	},
	[SCU1_CLK_GATE_I3C1CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c1clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 17,
	},
	[SCU1_CLK_GATE_I3C2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c2clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 18,
	},
	[SCU1_CLK_GATE_I3C3CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c3clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 19,
	},
	[SCU1_CLK_GATE_I3C4CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c4clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 20,
	},
	[SCU1_CLK_GATE_I3C5CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c5clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 21,
	},
	[SCU1_CLK_GATE_I3C6CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c6clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 22,
	},
	[SCU1_CLK_GATE_I3C7CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c7clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 23,
	},
	[SCU1_CLK_GATE_I3C8CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c8clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 24,
	},
	[SCU1_CLK_GATE_I3C9CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c9clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 25,
	},
	[SCU1_CLK_GATE_I3C10CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c10clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 26,
	},
	[SCU1_CLK_GATE_I3C11CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c11clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 27,
	},
	[SCU1_CLK_GATE_I3C12CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c12clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 28,
	},
	[SCU1_CLK_GATE_I3C13CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c13clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 29,
	},
	[SCU1_CLK_GATE_I3C14CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c14clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 30,
	},
	[SCU1_CLK_GATE_I3C15CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "i3c15clk-gate",
		.parent_names = (const char *[]){ "soc1-ahb", },
		.reg = SCU1_CLK_STOP,
		.clk_idx = 31,
	},
	[SCU1_CLK_UART5] = {
		.type = CLK_MUX,
		.name = "uart5clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 5,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART5CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart5clk-gate",
		.parent_names = (const char *[]){ "uart5clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 0,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_UART6] = {
		.type = CLK_MUX,
		.name = "uart6clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 6,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART6CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart6clk-gate",
		.parent_names = (const char *[]){ "uart6clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 1,
	},
	[SCU1_CLK_UART7] = {
		.type = CLK_MUX,
		.name = "uart7clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 7,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART7CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart7clk-gate",
		.parent_names = (const char *[]){ "uart7clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 2,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_UART8] = {
		.type = CLK_MUX,
		.name = "uart8clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 8,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART8CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart8clk-gate",
		.parent_names = (const char *[]){ "uart8clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 3,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_UART9] = {
		.type = CLK_MUX,
		.name = "uart9clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 9,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART9CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart9clk-gate",
		.parent_names = (const char *[]){ "uart9clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 4,
	},
	[SCU1_CLK_UART10] = {
		.type = CLK_MUX,
		.name = "uart10clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 10,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART10CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart10clk-gate",
		.parent_names = (const char *[]){ "uart10clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 5,
	},
	[SCU1_CLK_UART11] = {
		.type = CLK_MUX,
		.name = "uart11clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 11,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART11CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart11clk-gate",
		.parent_names = (const char *[]){ "uart11clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 6,
	},
	[SCU1_CLK_UART12] = {
		.type = CLK_MUX,
		.name = "uart12clk",
		.parent_names = (const char *[]){"uartxclk", "huartxclk", },
		.num_parents = 2,
		.reg = SCU1_CLK_SEL1,
		.bit_shift = 12,
		.bit_width = 1,
	},
	[SCU1_CLK_GATE_UART12CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "uart12clk-gate",
		.parent_names = (const char *[]){ "uart12clk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 7,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_UART13] = {
		.type = CLK_FIXED_FACTOR,
		.name = "uart13clk",
		.parent_names = (const char *[]){ "huartxclk", },
		.mult = 1,
		.div = 1,
	},
	[SCU1_CLK_UART14] = {
		.type = CLK_FIXED_FACTOR,
		.name = "uart14clk",
		.parent_names = (const char *[]){ "huartxclk", },
		.mult = 1,
		.div = 1,
	},
	[SCU1_CLK_GATE_FSICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "fsiclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 8,
	},
	[SCU1_CLK_GATE_LTPIPHYCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "ltpiphyclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 9,
	},
	[SCU1_CLK_GATE_LTPICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "ltpiclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 10,
	},
	[SCU1_CLK_GATE_VGALCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "vgalclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 11,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_UHCICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "usbuartclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 12,
	},
	[SCU1_CLK_CAN] = {
		.type = CLK_FIXED_FACTOR,
		.name = "canclk",
		.parent_names = (const char *[]){ "soc1-apll", },
		.mult = 1,
		.div = 10,
	},
	[SCU1_CLK_GATE_CANCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "canclk-gate",
		.parent_names = (const char *[]){ "canclk", },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 13,
	},
	[SCU1_CLK_GATE_PCICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "pciclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 14,
	},
	[SCU1_CLK_GATE_SLICLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc1-sliclk-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 15,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_E2MCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "soc1-e2m-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 16,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_PORTCUSB2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "portcusb2-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 17,
		.flags = CLK_IS_CRITICAL,
	},
	[SCU1_CLK_GATE_PORTDUSB2CLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "portdusb2-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 18,
	},
	[SCU1_CLK_GATE_LTPI1TXCLK] = {
		.type = CLK_GATE_ASPEED,
		.name = "ltp1tx-gate",
		.parent_names = (const char *[]){  },
		.reg = SCU1_CLK_STOP2,
		.clk_idx = 19,
	},
};

static struct clk_hw *ast2700_clk_hw_register_pll(int clk_idx, void __iomem *reg,
						  const struct ast2700_clk_info *clk,
						  struct ast2700_clk_ctrl *clk_ctrl)
{
	int scu = clk_ctrl->clk_data->scu;
	unsigned int mult, div;
	u32 val;

	if (!scu && clk_idx == SCU0_CLK_HPLL) {
		val = readl(clk_ctrl->base + SCU0_HWSTRAP1);
		if ((val & GENMASK(3, 2)) != 0) {
			switch ((val & GENMASK(3, 2)) >> 2) {
			case 1:
				return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, "soc0-hpll",
								       NULL, 0, 1900000000);
			case 2:
				return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, "soc0-hpll",
								       NULL, 0, 1800000000);
			case 3:
				return devm_clk_hw_register_fixed_rate(clk_ctrl->dev, "soc0-hpll",
								       NULL, 0, 1700000000);
			default:
				return ERR_PTR(-EINVAL);
			}
		}
	}

	val = readl(reg);

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

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, clk->name,
						 clk->parent_names[0], 0, mult, div);
}

static struct clk_hw *ast2700_clk_hw_register_uartpll(int clk_idx, void __iomem *reg,
						      const struct ast2700_clk_info *clk,
						      struct ast2700_clk_ctrl *clk_ctrl)
{
	unsigned int mult, div;
	u32 val = readl(reg);
	u32 r = val & 0xff;
	u32 n = (val >> 8) & 0x3ff;

	mult = r;
	div = n * 2;

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, clk->name,
						 clk->parent_names[0], 0, mult, div);
}

static struct clk_hw *ast2700_clk_hw_register_misc(int clk_idx, void __iomem *reg,
						   const struct ast2700_clk_info *clk,
						   struct ast2700_clk_ctrl *clk_ctrl)
{
	u32 div = 0;

	if (clk_idx == SCU0_CLK_MPHY)
		div = readl(reg) + 1;
	else if (clk_idx == SCU0_CLK_U2PHY_REFCLK)
		div = (GET_USB_REFCLK_DIV(readl(reg)) + 1) << 1;
	else
		return ERR_PTR(-EINVAL);

	return devm_clk_hw_register_fixed_factor(clk_ctrl->dev, clk->name,
						   clk->parent_names[0], clk->flags,
						   1, div);
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
						   const char *parent_name, unsigned long flags,
						   void __iomem *reg, u8 clock_idx,
						   u8 clk_gate_flags, spinlock_t *lock)
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
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->reg = reg;
	gate->bit_idx = clock_idx;
	gate->flags = clk_gate_flags;
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

	reg[0] = AST2700_DEF_MAC12_DELAY_1G;
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

static int ast2700_soc_clk_probe(struct platform_device *pdev)
{
	struct ast2700_clk_data *clk_data;
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

	clk_data = (struct ast2700_clk_data *)of_device_get_match_data(dev);
	if (!clk_data)
		return devm_of_platform_populate(dev);

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
	}

	for (i = 0; i < clk_data->nr_clks; i++) {
		const struct ast2700_clk_info *clk = &clk_data->clk_info[i];
		void __iomem *reg = clk_ctrl->base + clk->reg;

		if (clk->type == CLK_FIXED) {
			hws[i] = devm_clk_hw_register_fixed_rate(dev, clk->name, NULL,
								 clk->flags, clk->fixed_rate);
		} else if (clk->type == CLK_FIXED_FACTOR) {
			hws[i] = devm_clk_hw_register_fixed_factor(dev, clk->name,
								   clk->parent_names[0], clk->flags,
								   clk->mult, clk->div);
		} else if (clk->type == CLK_PLL) {
			hws[i] = ast2700_clk_hw_register_pll(i, reg, clk, clk_ctrl);
		} else if (clk->type == CLK_UART_PLL) {
			hws[i] = ast2700_clk_hw_register_uartpll(i, reg, clk, clk_ctrl);
		} else if (clk->type == CLK_MUX) {
			hws[i] = devm_clk_hw_register_mux(dev, clk->name, clk->parent_names,
							  clk->num_parents, clk->flags, reg,
							  clk->bit_shift, clk->bit_width,
							  0, &clk_ctrl->lock);
		} else if (clk->type == CLK_MISC) {
			hws[i] = ast2700_clk_hw_register_misc(i, reg, clk, clk_ctrl);
		} else if (clk->type == CLK_DIVIDER) {
			hws[i] = devm_clk_hw_register_divider(dev, clk->name, clk->parent_names[0],
							      clk->flags, reg, clk->bit_shift,
							      clk->bit_width, 0,
							      &clk_ctrl->lock);
		} else if (clk->type == CLK_DIV_TABLE) {
			hws[i] = clk_hw_register_divider_table(dev, clk->name, clk->parent_names[0],
							       clk->flags, reg, clk->bit_shift,
							       clk->bit_width, 0,
							       clk->div_table, &clk_ctrl->lock);
		} else if (clk->type == CLK_GATE_ASPEED) {
			hws[i] = ast2700_clk_hw_register_gate(dev, clk->name, clk->parent_names[0],
							      clk->flags, reg, clk->clk_idx,
							      clk->flags, &clk_ctrl->lock);
		} else {
			hws[i] = clk_hw_register_gate(dev, clk->name, clk->parent_names[0],
						      clk->flags, reg, clk->clk_idx, clk->flags,
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
	.driver = {
		.name = "clk-ast2700",
		.of_match_table = ast2700_scu_match,
	},
};

builtin_platform_driver_probe(ast2700_scu_driver, ast2700_soc_clk_probe);
