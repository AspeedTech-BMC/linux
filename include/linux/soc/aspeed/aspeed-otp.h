/* SPDX-License-Identifier: GPL-2.0-or-later WITH Linux-syscall-note */
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 */

#ifndef _LINUX_ASPEED_OTP_H
#define _LINUX_ASPEED_OTP_H

void otp_read_data_buf(u32 offset, u32 *buf, u32 len);

#endif /* _LINUX_ASPEED_OTP_H */
