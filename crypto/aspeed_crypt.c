// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Aspeed Technology Inc.
 */

#define ASPEED_CRYPT_NAME "aspeed-crypt"
#define pr_fmt(fmt) ASPEED_CRYPT_NAME ": " fmt

#include <linux/cdev.h>

#include "internal.h"

struct aspeed_crypt_test {
	char *test_item;
	int num;
	char *algo[4];
};

#define ASPEED_DES_TEST_SUITE                            \
	"des", 3,                                        \
	{                                                \
		"ecb(des)", "cbc(des)", "ctr(des)", NULL \
	}
#define ASPEED_DES3_TEST_SUITE                                          \
	"des3", 3,                                                      \
	{                                                               \
		"ecb(des3_ede)", "cbc(des3_ede)", "ctr(des3_ede)", NULL \
	}
#define ASPEED_SHA3_TEST_SUITE                                 \
	"sha-3", 4,                                            \
	{                                                      \
		"sha3-224", "sha3-256", "sha3-384", "sha3-512" \
	}
#define ASPEED_RSA_TEST_SUITE           \
	"rsa", 1,                       \
	{                               \
		"rsa", NULL, NULL, NULL \
	}
#define ASPEED_ECDSA_TEST_SUITE                                  \
	"ecdsa", 2,                                              \
	{                                                        \
		"ecdsa-nist-p256", "ecdsa-nist-p384", NULL, NULL \
	}

static const struct aspeed_crypt_test *
aspeed_crypt_get_alg(const char *algo_name)
{
	int i = 0;
	static const struct aspeed_crypt_test algo_ts[] = {
		{ ASPEED_DES_TEST_SUITE },   { ASPEED_DES3_TEST_SUITE },
		{ ASPEED_SHA3_TEST_SUITE },  { ASPEED_RSA_TEST_SUITE },
		{ ASPEED_ECDSA_TEST_SUITE },
	};

	for (i = 0; i < ARRAY_SIZE(algo_ts); i++) {
		if (strlen(algo_ts[i].test_item) == strlen(algo_name) &&
		    strncmp(algo_ts[i].test_item, algo_name,
			    strlen(algo_name)) == 0)
			return &algo_ts[i];
	}

	return NULL;
}

static int aspeed_crypt_run_tests(char *alg_name)
{
	const struct aspeed_crypt_test *ts;
	int ret = 0;
	int i = 0;

	ts = aspeed_crypt_get_alg(alg_name);
	if (!ts) {
		pr_err("No test suite found for algorithm: %s", alg_name);
		return -EINVAL;
	}

	for (i = 0; i < ts->num && ret == 0; i++) {
		if (!ts->algo[i])
			continue;

		ret = alg_test(ts->algo[i], ts->algo[i], 0, 0);

		pr_info("alg: %s tests %s\n", ts->algo[i],
			!ret ? "passed" : "failed");
	}

	return ret;
}

static ssize_t aspeed_crypt_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	u8 kbuf[16] = { 0 };

	if (count > sizeof(kbuf) - 1)
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, min(count, sizeof(kbuf))))
		return -EFAULT;
	strim(kbuf);

	if (!aspeed_crypt_run_tests(kbuf))
		pr_err("alg: %s tests passed\n", kbuf);
	else
		pr_err("alg: %s tests failed\n", kbuf);

	return min(count, sizeof(kbuf));
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.write = aspeed_crypt_write,
};

static dev_t dev;
static struct cdev cdev;
static struct class *class;
static int __init acrypt_mod_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&dev, 0, 1, ASPEED_CRYPT_NAME);
	if (ret < 0)
		return ret;

	cdev_init(&cdev, &fops);
	ret = cdev_add(&cdev, dev, 1);
	if (ret)
		goto del_region;

	class = class_create(ASPEED_CRYPT_NAME);
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		goto del_cdev;
	}

	device_create(class, NULL, dev, NULL, ASPEED_CRYPT_NAME);
	return 0;
del_cdev:
	cdev_del(&cdev);
del_region:
	unregister_chrdev_region(dev, 1);
	return ret;
}

static void __exit acrypt_mod_fini(void)
{
	device_destroy(class, dev);
	class_destroy(class);
	cdev_del(&cdev);
	unregister_chrdev_region(dev, 1);
}

subsys_initcall(acrypt_mod_init);
module_exit(acrypt_mod_fini);
