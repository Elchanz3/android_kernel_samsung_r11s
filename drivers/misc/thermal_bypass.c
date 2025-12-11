/*
 * A Simple fake temperature driver
 *
 * Copyright (C) 2025
 * Murilo Sales <inutilidades639@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include "linux/thermal_bypass.h"

static int thermal_bypass = 0;

int thermal_bypass_get_value(void)
{
	if (thermal_bypass) {
		if (thermal_bypass == 1)
			return TEMP_EMULATION;
		else
			return thermal_bypass;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(thermal_bypass_get_value);

static ssize_t thermal_bypass_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", thermal_bypass);
}

static ssize_t thermal_bypass_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	thermal_bypass = value;

	return count;
}

static ssize_t version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", THERMAL_BYPASS_VERSION);
}

static struct kobj_attribute thermal_bypass_attribute =
	__ATTR(thermal_bypass, 0644, thermal_bypass_show, thermal_bypass_store);

static struct kobj_attribute version_attribute =
	__ATTR(version, 0444, version_show, NULL);

static struct attribute *thermal_bypass_attributes[] = {
	&thermal_bypass_attribute.attr,
	&version_attribute.attr,
	NULL
};

static struct attribute_group thermal_bypass_control_group = {
	.attrs = thermal_bypass_attributes,
};

static struct kobject *thermal_bypass_kobj;

static int __init thermal_bypass_init(void)
{
	int sysfs_result;

	thermal_bypass_kobj = kobject_create_and_add("thermal_bypass", kernel_kobj);
	if (!thermal_bypass_kobj) {
		pr_err("%s thermal_bypass kobject create failed!\n", __func__);
		return -ENOMEM;
	}

	sysfs_result = sysfs_create_group(thermal_bypass_kobj,
			&thermal_bypass_control_group);

	if (sysfs_result) {
		pr_err("%s thermal_bypass sysfs create failed!\n", __func__);
		kobject_put(thermal_bypass_kobj);
	}

	return sysfs_result;
}

static void __exit thermal_bypass_exit(void)
{
	if (thermal_bypass_kobj != NULL)
		kobject_put(thermal_bypass_kobj);
}

module_init(thermal_bypass_init);
module_exit(thermal_bypass_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Thermal Bypass Control Driver");
MODULE_VERSION(THERMAL_BYPASS_VERSION);
