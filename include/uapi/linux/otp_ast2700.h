/* SPDX-License-Identifier: GPL-2.0-or-later WITH Linux-syscall-note */
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 */

#ifndef _UAPI_LINUX_OTP_AST2700_H
#define _UAPI_LINUX_OTP_AST2700_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct otp_read {
	unsigned int offset;
	unsigned int len;
	uint8_t *data;
};

struct otp_prog {
	unsigned int w_offset;
	unsigned int len;
	uint8_t *data;
};

struct otp_revid {
	uint32_t revid0;
	uint32_t revid1;
};

enum otp_region_id {
	OTP_REGION_ROM = 0,
	OTP_REGION_RBP,
	OTP_REGION_CFG,
	OTP_REGION_STRAP,
	OTP_REGION_STRAPEXT,
	OTP_REGION_USR,
	OTP_REGION_SEC,
	OTP_REGION_CAL,
	OTP_REGION_PUF,
	OTP_REGION_MAX,
};

struct otp_ecc_policy {
	uint32_t region;		/* enum otp_region_id, set by caller */
	uint32_t ecc_en;		/* 0: disabled, 1: enabled */
	uint32_t ecc_supported;	/* set by ASPEED_OTP_GET_ECC_POLICY, ignored on input */
};

/* Used with ASPEED_OTP_GET_ECC_CODE */
struct otp_read_ecc {
	uint32_t offset;	/* input:  OTP word address */
	uint32_t data;		/* output: 16-bit read data */
	uint32_t ecc_code;	/* output: OTP_DBG01[5:0] after a raw (non-ECC) read */
};

/*
 * Used with ASPEED_OTP_GET_REGION_PROTECT: no input fields, all output.
 * Each is a raw OTPMISCREG value - bit31=LOCK, bits15:8=WEN, bits7:0=REN -
 * except .puf, which has its own 4-bit lock-only layout (bit8=sw_puf_rlock,
 * bit4=hw_puf_rlock, bit1=hw_puf_wlock, bit0=sw_puf_wlock). secure/usr/
 * caliptra additionally have a *_range field: bits31:16=size, bits15:0=
 * start offset, both in OTP words.
 */
struct otp_region_protect {
	uint32_t rom_patch;
	uint32_t otpcfg;
	uint32_t otpstrap;
	uint32_t strap_ext;
	uint32_t secure0;
	uint32_t secure0_range;
	uint32_t secure1;
	uint32_t secure1_range;
	uint32_t secure2;
	uint32_t secure2_range;
	uint32_t secure3;
	uint32_t secure3_range;
	uint32_t usr0;
	uint32_t usr0_range;
	uint32_t usr1;
	uint32_t usr1_range;
	uint32_t usr2;
	uint32_t usr2_range;
	uint32_t usr3;
	uint32_t usr3_range;
	uint32_t caliptra0;
	uint32_t caliptra0_range;
	uint32_t caliptra1;
	uint32_t caliptra1_range;
	uint32_t caliptra2;
	uint32_t caliptra2_range;
	uint32_t caliptra3;
	uint32_t caliptra3_range;
	uint32_t rbp_soc_svn;
	uint32_t rbp_soc_keyretire;
	uint32_t rbp_calip_svn;
	uint32_t rbp_calip_keyretire;
	uint32_t puf;
};

/*
 * ASPEED_OTP_GET_ECC result: distinguishes ECC being off from the two ways
 * it can be on - temporarily by software (ASPEED_OTP_SET_ECC), or latched by
 * a hardware strap that can't be turned back off until the next power-on.
 */
enum otp_ecc_global_status {
	OTP_ECC_GLOBAL_DISABLED = 0,
	OTP_ECC_GLOBAL_SW_ENABLED,
	OTP_ECC_GLOBAL_HW_ENABLED,
};

#define OTP_A0				0
#define OTP_A1				1
#define OTP_A2				2
#define OTP_A3				3

#define OTPIOC_BASE			'O'

#define ASPEED_OTP_READ_DATA		_IOR(OTPIOC_BASE, 0, struct otp_read)
#define ASPEED_OTP_READ_CONF		_IOR(OTPIOC_BASE, 1, struct otp_read)
#define ASPEED_OTP_PROG_DATA		_IOW(OTPIOC_BASE, 2, struct otp_prog)
#define ASPEED_OTP_PROG_CONF		_IOW(OTPIOC_BASE, 3, struct otp_prog)
#define ASPEED_OTP_VER			_IOR(OTPIOC_BASE, 4, unsigned int)
#define ASPEED_OTP_SW_RID		_IOR(OTPIOC_BASE, 5, u32 *)
#define ASPEED_SEC_KEY_NUM		_IOR(OTPIOC_BASE, 6, u32 *)
#define ASPEED_OTP_GET_ECC		_IOR(OTPIOC_BASE, 7, uint32_t)
#define ASPEED_OTP_SET_ECC		_IO(OTPIOC_BASE, 8)
#define ASPEED_OTP_GET_REVID		_IOR(OTPIOC_BASE, 9, struct otp_revid)
#define ASPEED_OTP_GET_ECC_POLICY	_IOR(OTPIOC_BASE, 10, struct otp_ecc_policy)
#define ASPEED_OTP_SET_ECC_POLICY	_IOW(OTPIOC_BASE, 11, struct otp_ecc_policy)
#define ASPEED_OTP_GET_ROM_PATCH_VERSION _IOR(OTPIOC_BASE, 12, uint32_t)
#define ASPEED_OTP_GET_ECC_CODE	_IOWR(OTPIOC_BASE, 13, struct otp_read_ecc)
#define ASPEED_OTP_GET_REGION_PROTECT	_IOR(OTPIOC_BASE, 14, struct otp_region_protect)

#endif /* _UAPI_LINUX_OTP_AST2700_H */
