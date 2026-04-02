// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2026 Aspeed Technology Inc.
 */

#include "aspeed-hace.h"
#include <crypto/gcm.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/cipher.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO_DEBUG
#define CIPHER_DBG(h, fmt, ...) \
	dev_info((h)->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#else
#define CIPHER_DBG(h, fmt, ...) \
	dev_dbg((h)->dev, "%s() " fmt, __func__, ##__VA_ARGS__)
#endif

static bool aspeed_crypto_aead_need_fallback(struct aead_request *areq)
{
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(areq);

	if (areq->assoclen != 0)
		return true;

	if (rctx->enc_cmd & HACE_CMD_ENCRYPT && areq->cryptlen == 0)
		return true;

	if (!(rctx->enc_cmd & HACE_CMD_ENCRYPT) &&
	    areq->cryptlen == AES_GCM_TAG_SIZE)
		return true;

	return false;
}

static int aspeed_crypto_aead_do_fallback(struct aead_request *areq)
{
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(areq);
	struct crypto_aead *tfm = crypto_aead_reqtfm(areq);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(tfm);
	int err;

	aead_request_set_tfm(&rctx->fallback_aead_req, ctx->fallback_aead_tfm);

	aead_request_set_callback(&rctx->fallback_aead_req, areq->base.flags,
				  NULL, NULL);
	aead_request_set_crypt(&rctx->fallback_aead_req, areq->src, areq->dst,
			       areq->cryptlen, areq->iv);
	aead_request_set_ad(&rctx->fallback_aead_req, areq->assoclen);

	if (rctx->enc_cmd & HACE_CMD_ENCRYPT)
		err = crypto_aead_encrypt(&rctx->fallback_aead_req);
	else
		err = crypto_aead_decrypt(&rctx->fallback_aead_req);

	return err;
}

static int
aspeed_hace_crypto_aead_handle_queue(struct aspeed_hace_dev *hace_dev,
				     struct aead_request *req)
{
	CIPHER_DBG(hace_dev, "\n");

	if (aspeed_crypto_aead_need_fallback(req))
		return aspeed_crypto_aead_do_fallback(req);

	return crypto_transfer_aead_request_to_engine(hace_dev->crypt_engine_crypto, req);
}

static void aspeed_hace_aead_sg_unmap(struct aspeed_hace_dev *hace_dev,
				      struct aead_request *req)
{
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);

	if (rctx->dst_sg_len) {
		dma_unmap_sg(hace_dev->dev, req->dst, rctx->dst_nents,
			     DMA_FROM_DEVICE);
	}

	if (rctx->src_sg_len) {
		dma_unmap_sg(hace_dev->dev, req->src, rctx->src_nents,
			     DMA_TO_DEVICE);
	}
}

static int aspeed_hace_aead_sg_map(struct aspeed_hace_dev *hace_dev,
				   struct aead_request *req)
{
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);

	CIPHER_DBG(hace_dev, "\n");

	rctx->src_sg_len = dma_map_sg(hace_dev->dev, req->src, rctx->src_nents,
				      DMA_TO_DEVICE);
	if (!rctx->src_sg_len) {
		dev_warn(hace_dev->dev, "dma_map_sg() src error\n");
		return -EINVAL;
	}

	rctx->dst_sg_len = dma_map_sg(hace_dev->dev, req->dst, rctx->dst_nents,
				      DMA_FROM_DEVICE);
	if (!rctx->dst_sg_len) {
		dev_warn(hace_dev->dev, "dma_map_sg() dst error\n");
		return -EINVAL;
	}

	return 0;
}

static int aspeed_hace_aead_build_sg(struct aspeed_sg_list *sg_list,
				     struct scatterlist *src_sg, int src_sg_len,
				     int total)
{
	struct scatterlist *s;
	int i;

	for_each_sg(src_sg, s, src_sg_len, i) {
		u32 phy_addr = sg_dma_address(s);
		u32 len = sg_dma_len(s);

		len = total > len ? len : total | BIT(31);
		total = total > len ? total - len : 0;

		sg_list[i].phy_addr = cpu_to_le32(phy_addr);
		sg_list[i].len = cpu_to_le32(len);
	}

	sg_list[src_sg_len].phy_addr = 0;
	sg_list[src_sg_len].len = 0;

	if (total != 0)
		return -EINVAL;

	return 0;
}

static int aspeed_aead_complete(struct aspeed_hace_dev *hace_dev, int err)
{
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aead_request *req = crypto_engine->aead_req;
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);

	if (rctx->enc_cmd & HACE_CMD_ENCRYPT)
		scatterwalk_map_and_copy(crypto_engine->tag_addr, req->dst,
					 req->cryptlen, AES_GCM_TAG_SIZE, 1);

	if (!(rctx->enc_cmd & HACE_CMD_ENCRYPT))
		req->cryptlen += AES_GCM_TAG_SIZE;

	crypto_engine->flags &= ~CRYPTO_FLAGS_BUSY;

	crypto_finalize_aead_request(hace_dev->crypt_engine_crypto, req, err);

	return err;
}

static int aspeed_aead_transfer_sg(struct aspeed_hace_dev *hace_dev)
{
	struct aead_request *req = hace_dev->crypto_engine.aead_req;

	aspeed_hace_aead_sg_unmap(hace_dev, req);

	return aspeed_aead_complete(hace_dev, 0);
}

static void aspeed_hace_trigger(struct aspeed_hace_dev *hace_dev)
{
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aead_request *req = crypto_engine->aead_req;
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);

	CIPHER_DBG(hace_dev, "\n");

	ast_hace_write(hace_dev, crypto_engine->cipher_ctx_dma,
		       ASPEED_HACE_CONTEXT);
	ast_hace_write(hace_dev, crypto_engine->tag_dma_addr,
		       ASPEED_HACE_GCM_TAG_BASE_ADDR);
	ast_hace_write(hace_dev, crypto_engine->cipher_dma_addr,
		       ASPEED_HACE_SRC);
	ast_hace_write(hace_dev, crypto_engine->dst_sg_dma_addr,
		       ASPEED_HACE_DEST);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	ast_hace_write(hace_dev, crypto_engine->tag_dma_addr >> 32,
		       ASPEED_HACE_TAG_H);
	ast_hace_write(hace_dev, crypto_engine->cipher_ctx_dma >> 32,
		       ASPEED_HACE_CONTEXT_H);
	ast_hace_write(hace_dev, crypto_engine->cipher_dma_addr >> 32,
		       ASPEED_HACE_SRC_H);
	ast_hace_write(hace_dev, crypto_engine->dst_sg_dma_addr >> 32,
		       ASPEED_HACE_DEST_H);
#endif

	ast_hace_write(hace_dev, req->cryptlen, ASPEED_HACE_DATA_LEN);
	ast_hace_write(hace_dev, req->assoclen, ASPEED_HACE_GCM_ADD_LEN);
	ast_hace_write(hace_dev, rctx->enc_cmd, ASPEED_HACE_CMD);
}

static int aspeed_aead_start_sg(struct aspeed_hace_dev *hace_dev)
{
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aead_request *req = crypto_engine->aead_req;
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);
	struct aspeed_sg_list *src_list, *dst_list;
	int ret = 0;

	CIPHER_DBG(hace_dev, "\n");

	/* Scatter list mapping */
	ret = aspeed_hace_aead_sg_map(hace_dev, req);
	if (ret)
		goto fail;

	/* Build scatter-gather source list */
	src_list = (struct aspeed_sg_list *)crypto_engine->cipher_addr;
	ret = aspeed_hace_aead_build_sg(src_list, req->src, rctx->src_sg_len,
					req->cryptlen);
	if (ret)
		goto fail;

	/* Build scatter-gather destination list */
	dst_list = (struct aspeed_sg_list *)crypto_engine->dst_sg_addr;
	ret = aspeed_hace_aead_build_sg(dst_list, req->dst, rctx->dst_sg_len,
					req->cryptlen);
	if (ret)
		goto fail;

	crypto_engine->resume = aspeed_aead_transfer_sg;

	/* Memory barrier to ensure all data setup before engine starts */
	mb();
	aspeed_hace_trigger(hace_dev);

	return -EINPROGRESS;
fail:
	aspeed_hace_aead_sg_unmap(hace_dev, req);
	return ret;
}

static int aspeed_hace_aead_subkey(u8 *subkey, u8 *key, u32 keylen)
{
	struct crypto_cipher *tfm = NULL;
	int ret;
	u8 src[AES_BLOCK_SIZE] = { 0 };

	tfm = crypto_alloc_cipher("aes", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	ret = crypto_cipher_setkey(tfm, key, keylen);
	if (ret)
		goto out;

	/* Encrypt one 16-byte block */
	crypto_cipher_encrypt_one(tfm, subkey, src);

out:
	crypto_free_cipher(tfm);
	return ret;
}

static int aspeed_hace_aead_trigger(struct aspeed_hace_dev *hace_dev)
{
	struct aspeed_engine_crypto *crypto_engine = &hace_dev->crypto_engine;
	struct aead_request *req = crypto_engine->aead_req;
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	u8 iv_count[] = { 0x00, 0x00, 0x00, 0x01 };
	u8 subkey_offset = 0;
	u8 subkey[AES_BLOCK_SIZE] = { 0 };

	CIPHER_DBG(hace_dev, "\n");

	if (!(rctx->enc_cmd & HACE_CMD_ENCRYPT))
		req->cryptlen = req->cryptlen - AES_GCM_TAG_SIZE;

	aspeed_hace_aead_subkey(subkey, ctx->key, ctx->key_len);
	if (ctx->key_len == AES_KEYSIZE_128)
		subkey_offset = ASPEED_HACE_CTX_128_SUBKEY_OFFSET;
	else
		subkey_offset = ASPEED_HACE_CTX_256_SUBKEY_OFFSET;

	rctx->dst_nents = sg_nents(req->dst);
	rctx->src_nents = sg_nents(req->src);

	memcpy(crypto_engine->cipher_ctx, req->iv, GCM_AES_IV_SIZE);
	memcpy(crypto_engine->cipher_ctx + GCM_AES_IV_SIZE, iv_count,
	       sizeof(iv_count));
	memcpy(crypto_engine->cipher_ctx + ASPEED_HACE_CTX_KEY_OFFSET, ctx->key,
	       ctx->key_len);
	memcpy(crypto_engine->cipher_ctx + subkey_offset, subkey,
	       AES_BLOCK_SIZE);

	return aspeed_aead_start_sg(hace_dev);
}

static int aspeed_aes_aead_setkey(struct crypto_aead *aead, const u8 *key,
				  unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aspeed_hace_dev *hace_dev = ctx->hace_dev;

	CIPHER_DBG(hace_dev, "keylen: %d bits\n", (keylen * 8));

	ctx->key_idx = ASPEED_VAULT_KEY_NOT_FOUND;

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;

	memcpy(ctx->key, key, keylen);
	ctx->key_len = keylen;

	crypto_aead_clear_flags(ctx->fallback_aead_tfm, CRYPTO_TFM_REQ_MASK);
	crypto_aead_set_flags(ctx->fallback_aead_tfm,
			      aead->base.crt_flags & CRYPTO_TFM_REQ_MASK);

	return crypto_aead_setkey(ctx->fallback_aead_tfm, key, keylen);
}

static int aspeed_aes_aead_setauthsize(struct crypto_aead *aead,
				       unsigned int authsize)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aspeed_hace_dev *hace_dev = ctx->hace_dev;

	CIPHER_DBG(hace_dev, "authsize: %d bits\n", (authsize * 8));

	return crypto_aead_setauthsize(ctx->fallback_aead_tfm, authsize);
}

static int aspeed_aes_gcm_crypt(struct aead_request *req, u32 cmd)
{
	struct aspeed_cipher_reqctx *rctx = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aspeed_hace_dev *hace_dev = ctx->hace_dev;

	CIPHER_DBG(hace_dev, "\n");

	cmd = cmd | HACE_CMD_GCM | HACE_CMD_AES_SELECT |
	      HACE_CMD_RI_WO_DATA_ENABLE | HACE_CMD_CONTEXT_LOAD_ENABLE |
	      HACE_CMD_CONTEXT_SAVE_ENABLE | HACE_CMD_ISR_EN |
	      HACE_CMD_DES_SG_CTRL | HACE_CMD_SRC_SG_CTRL |
	      HACE_CMD_AES_KEY_HW_EXP | HACE_CMD_MBUS_REQ_SYNC_EN |
	      HACE_CMD_CTR_IV_AES_96 | HACE_CMD_GCM_TAG_ADDR_SEL;

	switch (ctx->key_len) {
	case AES_KEYSIZE_128:
		cmd |= HACE_CMD_AES128;
		break;
	case AES_KEYSIZE_192:
		cmd |= HACE_CMD_AES192;
		break;
	case AES_KEYSIZE_256:
		cmd |= HACE_CMD_AES256;
		break;
	default:
		return -EINVAL;
	}

	rctx->enc_cmd = cmd;

	return aspeed_hace_crypto_aead_handle_queue(ctx->hace_dev, req);
}

static int aspeed_aes_gcm_encrypt(struct aead_request *req)
{
	return aspeed_aes_gcm_crypt(req, HACE_CMD_ENCRYPT);
}

static int aspeed_aes_gcm_decrypt(struct aead_request *req)
{
	return aspeed_aes_gcm_crypt(req, HACE_CMD_DECRYPT);
}

static int aspeed_aead_do_request(struct crypto_engine *engine, void *areq)
{
	struct aead_request *req = aead_request_cast(areq);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aspeed_hace_dev *hace_dev = ctx->hace_dev;
	struct aspeed_engine_crypto *crypto_engine;
	int rc;

	crypto_engine = &hace_dev->crypto_engine;
	crypto_engine->aead_req = (struct aead_request *)req;
	crypto_engine->flags |= CRYPTO_FLAGS_BUSY;

	rc = aspeed_hace_aead_trigger(hace_dev);
	if (rc != -EINPROGRESS)
		return -EIO;

	return 0;
}

static int aspeed_crypto_aead_init(struct crypto_aead *aead)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct aspeed_hace_alg *crypto_alg = NULL;
	const char *name = crypto_tfm_alg_name(&aead->base);

	crypto_alg = container_of(alg, struct aspeed_hace_alg, alg.aead.base);
	ctx->hace_dev = crypto_alg->hace_dev;

	CIPHER_DBG(ctx->hace_dev, "\n");

	ctx->fallback_aead_tfm =
		crypto_alloc_aead(name, 0, CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback_aead_tfm)) {
		dev_err(ctx->hace_dev->dev, "Failed to allocate %s %ld\n", name,
			PTR_ERR(ctx->fallback_aead_tfm));
		return PTR_ERR(ctx->fallback_aead_tfm);
	}

	crypto_aead_set_reqsize(aead, sizeof(struct aead_request) +
			      crypto_aead_reqsize(ctx->fallback_aead_tfm));

	return 0;
}

static void aspeed_crypto_aead_exit(struct crypto_aead *aead)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(aead);
	struct aspeed_hace_dev *hace_dev = ctx->hace_dev;

	CIPHER_DBG(hace_dev, "\n");

	if (ctx->fallback_aead_tfm) {
		crypto_free_aead(ctx->fallback_aead_tfm);
		ctx->fallback_aead_tfm = NULL;
	}

	memzero_explicit(ctx, sizeof(struct aspeed_cipher_ctx));
}

static struct aspeed_hace_alg aspeed_aead_algs[] = {
	{
		.alg.aead.base = {
			.ivsize		= GCM_AES_IV_SIZE,
			.maxauthsize = AES_BLOCK_SIZE,
			.setkey		= aspeed_aes_aead_setkey,
			.setauthsize = aspeed_aes_aead_setauthsize,
			.encrypt	= aspeed_aes_gcm_encrypt,
			.decrypt	= aspeed_aes_gcm_decrypt,
			.init		= aspeed_crypto_aead_init,
			.exit		= aspeed_crypto_aead_exit,
			.base = {
				.cra_name		= "gcm(aes)",
				.cra_driver_name	= "aspeed-gcm-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AEAD |
							  CRYPTO_ALG_ASYNC |
							  CRYPTO_ALG_NEED_FALLBACK,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		},
		.alg.aead.op = {
			.do_one_request = aspeed_aead_do_request,
		},
	},
};

void aspeed_register_hace_aead_algs(struct aspeed_hace_dev *hace_dev)
{
	int rc, i;

	CIPHER_DBG(hace_dev, "\n");

	if (hace_dev->version == AST2500_VERSION ||
	    hace_dev->version == AST2600_VERSION)
		return;

	for (i = 0; i < ARRAY_SIZE(aspeed_aead_algs); i++) {
		aspeed_aead_algs[i].hace_dev = hace_dev;
		rc = crypto_engine_register_aead(&aspeed_aead_algs[i].alg.aead);
		if (rc) {
			CIPHER_DBG(hace_dev, "Failed to register %s\n",
				   aspeed_aead_algs[i].alg.aead.base.base.cra_name);
		}
	}
}
