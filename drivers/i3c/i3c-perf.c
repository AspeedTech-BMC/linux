// SPDX-License-Identifier: GPL-2.0-only
/*
 * I3C performance measurement test driver
 *
 * Binds to a generic I3C device and exposes a debugfs interface that
 * triggers configurable bursts of SDR private transfers, reporting
 * throughput and per-transfer latency.
 *
 * Copyright (C) 2026 ASPEED Technology Inc.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/i3c/device.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

/*
 * ASPEED silicon starts a transaction either when the full payload is
 * buffered or when the TX queue fills, so there is no per-transaction
 * payload ceiling beyond what the descriptor can express.  Cap at the
 * u16 i3c_priv_xfer.len maximum, which also matches the HC theoretical
 * 64 KB data length.
 */
#define I3C_PERF_MAX_LEN	65535
/*
 * The MIPI HCI DMA driver uses a single 255-entry transfer ring with the
 * standard enqueue==dequeue full-detection, so it can only hold 254
 * descriptors at a time.  Anything above that makes the very first
 * i3c_device_do_priv_xfers() call return -EBUSY before any transaction
 * is even attempted.
 */
#define I3C_PERF_MAX_BATCH	254
#define I3C_PERF_DEFAULT_LEN	64
#define I3C_PERF_DEFAULT_COUNT	1000
#define I3C_PERF_DEFAULT_BATCH	1

enum i3c_perf_dir {
	I3C_PERF_DIR_WRITE,
	I3C_PERF_DIR_READ,
	I3C_PERF_DIR_RW,	/* alternating write/read */
};

struct i3c_perf_result {
	bool			valid;
	u32			xfer_len;
	u32			xfer_count;
	u32			batch_size;
	u32			ncalls;
	u32			ok_calls;
	enum i3c_perf_dir	direction;
	u64			total_ns;
	u64			min_ns;	/* per i3c_device_do_priv_xfers() call */
	u64			max_ns;
	u64			sum_ns;	/* over successful calls only */
	u32			ok;
	u32			errors;
	int			last_errno;
};

struct i3c_perf {
	struct i3c_device	*i3cdev;
	struct dentry		*dbg_root;
	struct mutex		lock; /* serialises run vs result/config */
	void			*tx_buf;
	void			*rx_buf;
	struct i3c_priv_xfer	*xfers;	/* array, sized I3C_PERF_MAX_BATCH */
	u32			xfer_len;
	u32			xfer_count;
	u32			batch_size;
	enum i3c_perf_dir	direction;
	struct i3c_perf_result	last;
};

static const char *i3c_perf_dir_str(enum i3c_perf_dir d)
{
	switch (d) {
	case I3C_PERF_DIR_WRITE: return "write";
	case I3C_PERF_DIR_READ:  return "read";
	case I3C_PERF_DIR_RW:    return "rw";
	default:                 return "?";
	}
}

static int i3c_perf_run_locked(struct i3c_perf *perf)
{
	struct i3c_perf_result *r = &perf->last;
	u64 t_start, t_end, t0, t1, dt;
	u32 done = 0;
	int ret;

	if (perf->xfer_len == 0 || perf->xfer_len > I3C_PERF_MAX_LEN)
		return -EINVAL;
	if (perf->xfer_count == 0)
		return -EINVAL;
	if (perf->batch_size == 0 || perf->batch_size > I3C_PERF_MAX_BATCH)
		return -EINVAL;

	memset(perf->tx_buf, 0xa5, perf->xfer_len);

	memset(r, 0, sizeof(*r));
	r->xfer_len = perf->xfer_len;
	r->xfer_count = perf->xfer_count;
	r->batch_size = perf->batch_size;
	r->direction = perf->direction;
	r->min_ns = U64_MAX;

	t_start = ktime_get_ns();
	while (done < perf->xfer_count) {
		u32 n = min(perf->batch_size, perf->xfer_count - done);
		u32 i;

		for (i = 0; i < n; i++) {
			bool rd;

			switch (perf->direction) {
			case I3C_PERF_DIR_WRITE:
				rd = false;
				break;
			case I3C_PERF_DIR_READ:
				rd = true;
				break;
			case I3C_PERF_DIR_RW:
				rd = (done + i) & 1;
				break;
			default:
				return -EINVAL;
			}

			perf->xfers[i].rnw = rd;
			perf->xfers[i].len = perf->xfer_len;
			perf->xfers[i].err = 0;
			if (rd)
				perf->xfers[i].data.in = perf->rx_buf;
			else
				perf->xfers[i].data.out = perf->tx_buf;
		}

		t0 = ktime_get_ns();
		ret = i3c_device_do_priv_xfers(perf->i3cdev, perf->xfers, n);
		t1 = ktime_get_ns();

		r->ncalls++;

		if (ret) {
			r->errors += n;
			r->last_errno = ret;
			done += n;
			cond_resched();
			continue;
		}

		dt = t1 - t0;
		if (dt < r->min_ns)
			r->min_ns = dt;
		if (dt > r->max_ns)
			r->max_ns = dt;
		r->sum_ns += dt;
		r->ok += n;
		r->ok_calls++;
		done += n;

		cond_resched();
	}
	t_end = ktime_get_ns();
	r->total_ns = t_end - t_start;
	r->valid = true;

	return 0;
}

static ssize_t i3c_perf_run_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct i3c_perf *perf = file->private_data;
	int ret;

	if (mutex_lock_interruptible(&perf->lock))
		return -EINTR;
	ret = i3c_perf_run_locked(perf);
	mutex_unlock(&perf->lock);

	return ret ? ret : count;
}

static const struct file_operations i3c_perf_run_fops = {
	.open	= simple_open,
	.write	= i3c_perf_run_write,
	.llseek	= noop_llseek,
};

static int i3c_perf_result_show(struct seq_file *s, void *unused)
{
	struct i3c_perf *perf = s->private;
	struct i3c_perf_result r;
	u64 bytes, bps, avg;

	mutex_lock(&perf->lock);
	r = perf->last;
	mutex_unlock(&perf->lock);

	if (!r.valid) {
		seq_puts(s, "no run yet\n");
		return 0;
	}

	bytes = (u64)r.xfer_len * r.ok;
	bps = r.total_ns ? div64_u64(bytes * NSEC_PER_SEC, r.total_ns) : 0;
	avg = r.ok_calls ? div64_u64(r.sum_ns, r.ok_calls) : 0;

	seq_printf(s, "direction: %s\n", i3c_perf_dir_str(r.direction));
	seq_printf(s, "xfer_len: %u\n", r.xfer_len);
	seq_printf(s, "xfer_count: %u\n", r.xfer_count);
	seq_printf(s, "batch_size: %u\n", r.batch_size);
	seq_printf(s, "ncalls: %u\n", r.ncalls);
	seq_printf(s, "ok: %u\n", r.ok);
	seq_printf(s, "errors: %u\n", r.errors);
	if (r.errors)
		seq_printf(s, "last_errno: %d\n", r.last_errno);
	seq_printf(s, "total_ns: %llu\n", r.total_ns);
	seq_printf(s, "throughput_Bps: %llu\n", bps);
	if (r.ok_calls) {
		seq_printf(s, "lat_call_min_ns: %llu\n", r.min_ns);
		seq_printf(s, "lat_call_max_ns: %llu\n", r.max_ns);
		seq_printf(s, "lat_call_avg_ns: %llu\n", avg);
		seq_printf(s, "lat_per_xfer_avg_ns: %llu\n",
			   r.batch_size ? div64_u64(avg, r.batch_size) : 0);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(i3c_perf_result);

static int i3c_perf_dir_show(struct seq_file *s, void *unused)
{
	struct i3c_perf *perf = s->private;

	mutex_lock(&perf->lock);
	seq_printf(s, "%s\n", i3c_perf_dir_str(perf->direction));
	mutex_unlock(&perf->lock);
	return 0;
}

static ssize_t i3c_perf_dir_write(struct file *file, const char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct i3c_perf *perf = s->private;
	enum i3c_perf_dir new_dir;
	char buf[8];
	size_t n;

	n = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, ubuf, n))
		return -EFAULT;
	buf[n] = '\0';
	strim(buf);

	if (!strcmp(buf, "w") || !strcmp(buf, "write"))
		new_dir = I3C_PERF_DIR_WRITE;
	else if (!strcmp(buf, "r") || !strcmp(buf, "read"))
		new_dir = I3C_PERF_DIR_READ;
	else if (!strcmp(buf, "rw") || !strcmp(buf, "wr"))
		new_dir = I3C_PERF_DIR_RW;
	else
		return -EINVAL;

	mutex_lock(&perf->lock);
	perf->direction = new_dir;
	mutex_unlock(&perf->lock);
	return count;
}

static int i3c_perf_dir_open(struct inode *inode, struct file *file)
{
	return single_open(file, i3c_perf_dir_show, inode->i_private);
}

static const struct file_operations i3c_perf_dir_fops = {
	.owner		= THIS_MODULE,
	.open		= i3c_perf_dir_open,
	.read		= seq_read,
	.write		= i3c_perf_dir_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int i3c_perf_probe(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);
	struct i3c_perf *perf;

	perf = devm_kzalloc(dev, sizeof(*perf), GFP_KERNEL);
	if (!perf)
		return -ENOMEM;

	perf->tx_buf = devm_kmalloc(dev, I3C_PERF_MAX_LEN, GFP_KERNEL);
	perf->rx_buf = devm_kmalloc(dev, I3C_PERF_MAX_LEN, GFP_KERNEL);
	perf->xfers = devm_kcalloc(dev, I3C_PERF_MAX_BATCH,
				   sizeof(*perf->xfers), GFP_KERNEL);
	if (!perf->tx_buf || !perf->rx_buf || !perf->xfers)
		return -ENOMEM;

	mutex_init(&perf->lock);
	perf->i3cdev = i3cdev;
	perf->xfer_len = I3C_PERF_DEFAULT_LEN;
	perf->xfer_count = I3C_PERF_DEFAULT_COUNT;
	perf->batch_size = I3C_PERF_DEFAULT_BATCH;
	perf->direction = I3C_PERF_DIR_WRITE;

	perf->dbg_root = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_u32("xfer_len", 0644, perf->dbg_root, &perf->xfer_len);
	debugfs_create_u32("xfer_count", 0644, perf->dbg_root,
			   &perf->xfer_count);
	debugfs_create_u32("batch_size", 0644, perf->dbg_root,
			   &perf->batch_size);
	debugfs_create_file("direction", 0644, perf->dbg_root, perf,
			    &i3c_perf_dir_fops);
	debugfs_create_file("run", 0200, perf->dbg_root, perf,
			    &i3c_perf_run_fops);
	debugfs_create_file("result", 0444, perf->dbg_root, perf,
			    &i3c_perf_result_fops);

	i3cdev_set_drvdata(i3cdev, perf);

	dev_info(dev, "i3c-perf bound; debugfs at /sys/kernel/debug/%s/\n",
		 dev_name(dev));
	return 0;
}

static void i3c_perf_remove(struct i3c_device *i3cdev)
{
	struct i3c_perf *perf = i3cdev_get_drvdata(i3cdev);

	debugfs_remove_recursive(perf->dbg_root);
}

/*
 * Match any device that reports DCR = generic.  Adjust this table or
 * use sysfs manual bind/unbind to target a specific peer.
 */
static const struct i3c_device_id i3c_perf_ids[] = {
	I3C_CLASS(I3C_DCR_GENERIC_DEVICE, NULL),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, i3c_perf_ids);

static struct i3c_driver i3c_perf_driver = {
	.driver = {
		.name = "i3c-perf",
	},
	.probe		= i3c_perf_probe,
	.remove		= i3c_perf_remove,
	.id_table	= i3c_perf_ids,
};
module_i3c_driver(i3c_perf_driver);

MODULE_AUTHOR("Billy Tsai <billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("I3C performance measurement test driver");
MODULE_LICENSE("GPL");
