// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 Aspeed Technology Inc.
 */

#define ASPEED_CRYPT_NAME "aspeed-crypt"
#define pr_fmt(fmt) ASPEED_CRYPT_NAME ": " fmt

#include <crypto/akcipher.h>
#include <crypto/hash.h>
#include <crypto/sig.h>
#include <crypto/skcipher.h>
#include <linux/cdev.h>

#include "internal.h"

enum aspeed_crypt_type {
	ASPEED_SKCIPHER_TYPE,
	ASPEED_AHASH_TYPE,
	ASPEED_AKCIPHER_TYPE,
	ASPEED_SIG_TYPE,
};

struct aspeed_crypt_drv {
	char *algo;
	char *drv_name;
};

struct aspeed_crypt_test {
	char *test_item;
	enum aspeed_crypt_type type;
	int num;
	struct aspeed_crypt_drv *algo;
};

struct aspeed_crypt_drv aspeed_des_binding[] = {
	{ "ecb(des)", "aspeed-ecb-des" },
	{ "cbc(des)", "aspeed-cbc-des" },
	{ "ctr(des)", "aspeed-ctr-des" },
};

struct aspeed_crypt_drv aspeed_tdes_binding[] = {
	{ "ecb(des3_ede)", "aspeed-ecb-tdes" },
	{ "cbc(des3_ede)", "aspeed-cbc-tdes" },
	{ "ctr(des3_ede)", "aspeed-ctr-tdes" },
};

struct aspeed_crypt_drv aspeed_sha3_binding[] = {
	{ "sha3-224", "aspeed-sha3-224" },
	{ "sha3-256", "aspeed-sha3-256" },
	{ "sha3-384", "aspeed-sha3-384" },
	{ "sha3-512", "aspeed-sha3-512" },
};

struct aspeed_crypt_drv aspeed_rsa_binding[] = {
	{ "rsa", "aspeed-rsa" },
};

struct aspeed_crypt_drv aspeed_ecdsa_binding[] = {
	{ "ecdsa-nist-p256", "aspeed-ecdsa-nist-p256" },
	{ "ecdsa-nist-p384", "aspeed-ecdsa-nist-p384" },
};

static bool aspeed_detect_skcipher_drv(char *drv_name)
{
	struct crypto_skcipher *tfm = NULL;
	bool found = false;

	tfm = crypto_alloc_skcipher(drv_name, 0, 0);
	if (!IS_ERR(tfm)) {
		found = true;
		crypto_free_skcipher(tfm);
	}

	return found;
}

static bool aspeed_detect_ahash_drv(char *drv_name)
{
	struct crypto_ahash *tfm = NULL;
	bool found = false;

	tfm = crypto_alloc_ahash(drv_name, 0, 0);
	if (!IS_ERR(tfm)) {
		found = true;
		crypto_free_ahash(tfm);
	}

	return found;
}

static bool aspeed_detect_akcipher_drv(char *drv_name)
{
	struct crypto_akcipher *tfm = NULL;
	bool found = false;

	tfm = crypto_alloc_akcipher(drv_name, 0, 0);
	if (!IS_ERR(tfm)) {
		found = true;
		crypto_free_akcipher(tfm);
	}

	return found;
}

static bool aspeed_detect_sig_drv(char *drv_name)
{
	struct crypto_sig *tfm = NULL;
	bool found = false;

	tfm = crypto_alloc_sig(drv_name, 0, 0);
	if (!IS_ERR(tfm)) {
		found = true;
		crypto_free_sig(tfm);
	}

	return found;
}

static bool aspeed_detect_drv(enum aspeed_crypt_type type, char *drv_name)
{
	switch (type) {
	case ASPEED_SKCIPHER_TYPE:
		return aspeed_detect_skcipher_drv(drv_name);
	case ASPEED_AHASH_TYPE:
		return aspeed_detect_ahash_drv(drv_name);
	case ASPEED_AKCIPHER_TYPE:
		return aspeed_detect_akcipher_drv(drv_name);
	case ASPEED_SIG_TYPE:
		return aspeed_detect_sig_drv(drv_name);
	default:
		return false;
	}
}

static const struct aspeed_crypt_test *
aspeed_crypt_get_alg(const char *algo_name)
{
	int i = 0;
	static const struct aspeed_crypt_test algo_ts[] = {
		{ "des", ASPEED_SKCIPHER_TYPE, ARRAY_SIZE(aspeed_des_binding),
		  aspeed_des_binding },
		{ "des3", ASPEED_SKCIPHER_TYPE, ARRAY_SIZE(aspeed_tdes_binding),
		  aspeed_tdes_binding },
		{ "sha-3", ASPEED_AHASH_TYPE, ARRAY_SIZE(aspeed_sha3_binding),
		  aspeed_sha3_binding },
		{ "rsa", ASPEED_AKCIPHER_TYPE, ARRAY_SIZE(aspeed_rsa_binding),
		  aspeed_rsa_binding },
		{ "ecdsa", ASPEED_SIG_TYPE, ARRAY_SIZE(aspeed_ecdsa_binding),
		  aspeed_ecdsa_binding },
	};

	for (i = 0; i < ARRAY_SIZE(algo_ts); i++) {
		if (strlen(algo_ts[i].test_item) == strlen(algo_name) &&
		    strncmp(algo_ts[i].test_item, algo_name,
			    strlen(algo_name)) == 0)
			return &algo_ts[i];
	}

	return NULL;
}

static __maybe_unused int aspeed_crypt_run_tests(char *alg_name)
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
		if (aspeed_detect_drv(ts->type, ts->algo[i].drv_name))
			ret = alg_test(ts->algo[i].drv_name, ts->algo[i].algo,
				       0, 0);
		else
			ret = -ENOENT;

		pr_info("alg: %s tests %s\n", ts->algo[i].algo,
			!ret ? "passed" : "failed");
	}

	return ret;
}

#if defined(CONFIG_CRYPTO_MANAGER_DISABLE_TESTS) && \
	!defined(CONFIG_CRYPTO_DEV_ASPEED_ACRY)
static ssize_t aspeed_crypt_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	pr_err("Crypto tests are disabled\n");
	return -EOPNOTSUPP;
}
#else
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
#endif

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
