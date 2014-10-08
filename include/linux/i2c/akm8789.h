#ifndef __AKM8789_H
#define __AKM8789_H

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>

#ifdef CONFIG_BOARD_ZTEMT_NX504J
#define HALL_DEVICE_INT_S    67
#else
#define HALL_DEVICE_INT_S    62
#endif
#define HALL_DEVICE_INT_N    68

#define MAGNETIC_DEVICE_NEAR   1  //Near
#define MAGNETIC_DEVICE_FAR    2  //Far

struct akm8789_irq {
    unsigned int irq_num;
    unsigned int irq_pin;
    bool enabled;
};

struct akm8789_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};


struct akm8789_chip {
	struct mutex lock;
	struct i2c_client *client;

    struct akm8789_irq irq_s;
    struct akm8789_irq irq_n;

	struct work_struct irq_work_s;
	struct work_struct irq_work_n;

	struct input_dev *idev;

    struct device *hall_device_dev;

    struct akm8789_wake_lock wakeup_wakelock;

	struct hrtimer unlock_wakelock_timer;

    bool enabled;
    bool on_irq_working;
};


static void akm8789_check_state(struct akm8789_chip *chip);
static void akm8789_enable(struct akm8789_chip *chip, int on);
static ssize_t akm8789_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t akm8789_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static void akm8789_irq_enable(struct akm8789_irq * irq, bool enable, bool flag_sync);
static irqreturn_t akm8789_irq_n(int irq, void *handle);
static irqreturn_t akm8789_irq_s(int irq, void *handle);
static void akm8789_irq_work_n(struct work_struct *work);
static void akm8789_irq_work_s(struct work_struct *work);
static int create_sysfs_interfaces(struct device *dev);
static int akm8789_suspend(struct device *dev);
static int akm8789_resume(struct device *dev);
static int __devinit akm8789_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit akm8789_remove(struct i2c_client *client);
static int __init akm8789_init(void);
static void __exit akm8789_exit(void);
static void akm8789_chip_data_init(struct akm8789_chip *chip);
static void akm8789_wakelock_ops(struct akm8789_wake_lock *wakelock, bool enable);



#endif /* __AKM8789_H */
