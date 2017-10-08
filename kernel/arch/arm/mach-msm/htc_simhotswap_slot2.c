/* arch/arm/mach-msm/htc_simhotswap.c
 *
 * Copyright (C) 2012 HTC Corporation.
 * Author: YaWen Su <YaWen_Su@htc.com>
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
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define SIM2_GPIO_DETECT 39

static uint32_t sim2_gpio_table[] = {
	GPIO_CFG(SIM2_GPIO_DETECT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void hotswap_work_func_slot2(struct work_struct *work);
static DECLARE_DELAYED_WORK(hotswap_work, hotswap_work_func_slot2);

static int htc_hotswap_open(struct inode *inode, struct file *file)
{
	return 0;
}

const struct file_operations htc_hotswap_fops2 = {
	.owner = THIS_MODULE,
	.open = htc_hotswap_open,
};

static struct miscdevice sim_hotswap_misc2 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_simhotswap_slot2",
	.fops = &htc_hotswap_fops2,
};

struct htc_simhotswap_info2 {
	struct kobject simhotswap_kobj;
	struct work_struct hotswap_work;
	struct workqueue_struct *hotswap_wq;
};

static struct htc_simhotswap_info2 htc_hotswap_info;
static struct kset *htc_hotswap_kset;

static void htc_simhotswap_kobject_release(struct kobject *kobj)
{
	printk(KERN_ERR "htc_hotswap_kobject_release.\n");
	return;
}

static struct kobj_type htc_hotswap_ktype = {
	.release = htc_simhotswap_kobject_release,
};

static void hotswap_work_func_slot2(struct work_struct *work)
{
	int status;
	char message[32] = "SIMHOTSWAP_SLOT2=";
	char *envp[] = { message, NULL };
	status = gpio_get_value(SIM2_GPIO_DETECT);
	pr_info("SIM_DETECT_SLOT2 = %d\n", status);
	if (status)
		strncat(message, "REMOVE", 6);
	else
		strncat(message, "INSERT", 6);

	kobject_uevent_env(&htc_hotswap_info.simhotswap_kobj, KOBJ_CHANGE, envp);

	return;
}

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static irqreturn_t sim_detect_irq(int irq, void *dev_id)
{
        int slot2_status = gpio_get_value(SIM2_GPIO_DETECT);
        pr_info("slot2_detect_status = %d\n", slot2_status);
	cancel_delayed_work_sync(&hotswap_work);
	queue_delayed_work(htc_hotswap_info.hotswap_wq, &hotswap_work,
		msecs_to_jiffies(500));
	return IRQ_HANDLED;
}

static int htc_simhotswap_probe2(struct platform_device *pdev)
{
	int ret = 0;

	config_gpio_table(sim2_gpio_table, ARRAY_SIZE(sim2_gpio_table));
	INIT_WORK(&htc_hotswap_info.hotswap_work, hotswap_work_func_slot2);
	htc_hotswap_info.hotswap_wq = create_singlethread_workqueue("htc_simhotswap_slot2");

	ret = request_any_context_irq(gpio_to_irq(SIM2_GPIO_DETECT),
			sim_detect_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"sim_detect", NULL);

	if (ret) {
		pr_err("%s:Failed to request irq for SIM1, ret=%d\n", __func__, ret);
	}

	ret = misc_register(&sim_hotswap_misc2);
	if (ret) {
		pr_err("failed to register misc device!\n");
		goto fail;
	}

	htc_hotswap_kset = kset_create_and_add("event", NULL,
			kobject_get(&sim_hotswap_misc2.this_device->kobj));

	if (!htc_hotswap_kset) {
		ret = -ENOMEM;
		goto fail;
	}

	htc_hotswap_info.simhotswap_kobj.kset = htc_hotswap_kset;

	ret = kobject_init_and_add(&htc_hotswap_info.simhotswap_kobj,
			&htc_hotswap_ktype, NULL, "simhotswap");
	if (ret) {
		kobject_put(&htc_hotswap_info.simhotswap_kobj);
		goto fail;
	}
	pr_info("htc_simhotswap_probe2(): finish\n");


fail:
	return ret;
}

static struct platform_driver htc_simhotswap_driver2 = {
	.probe	= htc_simhotswap_probe2,
	.driver	= {
		.name	= "htc_simhotswap_slot2",
		.owner	= THIS_MODULE,
	},
};

static int __init sim_hotswap_init2(void)
{
	platform_driver_register(&htc_simhotswap_driver2);
	return 0;
}

module_init(sim_hotswap_init2);
MODULE_DESCRIPTION("HTC SIMHOTSWAP Driver");
