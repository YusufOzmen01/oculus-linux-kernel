/* SPDX-License-Identifier: GPL-2.0-only */
/* Experimental native timeline opaque-FD lifetime. Included by kgsl_timeline.c. */
struct qkx_timeline_file {
	struct kgsl_timeline *timeline;
	struct file *owner_file;
};

static void qkx_timeline_file_drop(struct qkx_timeline_file *payload)
{
	struct kgsl_device_private *owner = payload->owner_file->private_data;
	struct kgsl_device *device = owner->device;
	struct kgsl_timeline *timeline = payload->timeline;
	bool remove;

	spin_lock(&device->timelines_lock);
	timeline->exported_files--;
	remove = !timeline->exported_files && !timeline->dev_priv;
	spin_unlock(&device->timelines_lock);
	if (remove)
		qkx_timeline_remove(device, NULL, timeline->id);
	kgsl_timeline_put(timeline);
	/* May now release the original KGSL open instance and its other timelines. */
	fput(payload->owner_file);
	kfree(payload);
}

static int qkx_timeline_file_release(struct inode *inode, struct file *file)
{
	qkx_timeline_file_drop(file->private_data);
	return 0;
}

static long qkx_timeline_file_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct qkx_timeline_file *payload = file->private_data;
	u32 id = payload->timeline->id;

	if (cmd != IOCTL_KGSL_QKX_TIMELINE_ID)
		return -ENOTTY;
	return copy_to_user((void __user *)arg, &id, sizeof(id)) ? -EFAULT : 0;
}

static const struct file_operations qkx_timeline_file_ops = {
	.owner = THIS_MODULE,
	.release = qkx_timeline_file_release,
	.unlocked_ioctl = qkx_timeline_file_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = qkx_timeline_file_ioctl,
#endif
	.llseek = no_llseek,
};

long kgsl_ioctl_qkx_timeline_export(struct kgsl_device_private *dev_priv,
		unsigned int cmd, void *data)
{
	struct kgsl_qkx_timeline_export *param = data;
	struct kgsl_device *device = dev_priv->device;
	struct qkx_timeline_file *payload;
	struct kgsl_timeline *timeline;
	int fd;

	payload = kzalloc(sizeof(*payload), GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	spin_lock(&device->timelines_lock);
	timeline = idr_find(&device->timelines, param->timeline);
	if (!timeline || timeline->dev_priv != dev_priv) {
		spin_unlock(&device->timelines_lock);
		kfree(payload);
		return -EINVAL;
	}
	payload->timeline = kgsl_timeline_get(timeline);
	payload->owner_file = get_file(dev_priv->file);
	timeline->exported_files++;
	spin_unlock(&device->timelines_lock);

	fd = anon_inode_getfd("qkx-kgsl-timeline", &qkx_timeline_file_ops,
		payload, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		qkx_timeline_file_drop(payload);
		return fd;
	}
	param->fd = fd;
	return 0;
}

/* Original driver did not release timelines when an open instance closed. */
void kgsl_timeline_release_owner(struct kgsl_device_private *dev_priv)
{
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_timeline *timeline;
	int id;

	for (;;) {
		int found = 0;

		spin_lock(&device->timelines_lock);
		idr_for_each_entry(&device->timelines, timeline, id) {
			if (timeline && timeline->dev_priv == dev_priv) {
				found = id;
				break;
			}
		}
		spin_unlock(&device->timelines_lock);
		if (!found)
			break;
		qkx_timeline_remove(device, dev_priv, found);
	}
}
