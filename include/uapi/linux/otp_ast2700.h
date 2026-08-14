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

#endif /* _UAPI_LINUX_OTP_AST2700_H */
