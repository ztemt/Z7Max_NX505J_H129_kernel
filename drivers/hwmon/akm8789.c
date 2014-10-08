/******************** (C) COPYRIGHT 2013 ZTEMT ********************
*
* File Name          : akm8789.c
* Authors            : Zhu Bing
* Version            : V.1.0.0
* Date               : 09/17/2013
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.
 
Revision 1-0-0 09/17/2013
 first revision

*******************************************************************************/
 
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
#include <linux/i2c/akm8789.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>



#define LOG_TAG "HALL_DEVICE"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_ZTEMT_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
                                              
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif

#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

 
static dev_t const   hall_device_dev_t   = MKDEV(MISC_MAJOR, 252);

static struct class  *hall_device_class;

static const struct dev_pm_ops akm8789_pm_ops = {
    .suspend = akm8789_suspend,
    .resume  = akm8789_resume,
};

static const struct i2c_device_id akm8789_idtable_id[] = {
     { "akm,akm8789", 0 },
     { },
 };
 
static struct of_device_id of_akm8789_idtable[] = {
     { .compatible = "akm,akm8789",},
     {}
};

static struct i2c_driver akm8789_driver = {
    .driver = {
        .name = "akm8789",
        .of_match_table = of_akm8789_idtable,
        .pm = &akm8789_pm_ops,
    },
    .id_table = akm8789_idtable_id,
    .probe = akm8789_probe,
    .remove = __devexit_p(akm8789_remove),
};


static int __init akm8789_init(void)
{
        SENSOR_LOG_INFO("driver: init\n");
        return i2c_add_driver(&akm8789_driver);
}
 
static void __exit akm8789_exit(void)
{
        SENSOR_LOG_INFO("driver: exit\n");
        i2c_del_driver(&akm8789_driver);
}

static void akm8789_wakelock_ops(struct akm8789_wake_lock *wakelock, bool enable)
{
    if (enable == wakelock->locked)
    {
        SENSOR_LOG_INFO("doubule %s %s, retern here\n",enable? "lock" : "unlock",wakelock->name);
        return;
    }

    if (enable)
    {
        wake_lock(&wakelock->lock);
    }
    else
    {
        wake_unlock(&wakelock->lock);
    }

    wakelock->locked = enable;

    SENSOR_LOG_INFO("%s %s \n",enable? "lock" : "unlock",wakelock->name);
}

static enum hrtimer_restart akm8789_unlock_wakelock_work_func(struct hrtimer *timer)
{ 
    struct akm8789_chip *chip = container_of(timer, struct akm8789_chip, unlock_wakelock_timer);

    if (false == chip->on_irq_working )
    {
        akm8789_wakelock_ops(&(chip->wakeup_wakelock),false);
    }

    return HRTIMER_NORESTART;
}


static void akm8789_irq_work_s(struct work_struct *work)
{
	struct akm8789_chip *chip = container_of(work, struct akm8789_chip, irq_work_s);

	mutex_lock(&chip->lock);
    //SENSOR_LOG_INFO("enter\n");

    if (0 == gpio_get_value(chip->irq_s.irq_pin))
    {  
       SENSOR_LOG_INFO("MAGNETIC_DEVICE NEAR\n");
       input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_NEAR);
       akm8789_wakelock_ops(&(chip->wakeup_wakelock),false);
    }
    else
    {
        SENSOR_LOG_INFO("MAGNETIC_DEVICE FAR\n");
        input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_FAR);
        hrtimer_start(&chip->unlock_wakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);
    }
    input_sync(chip->idev);

	chip->on_irq_working = false;

    akm8789_irq_enable(&(chip->irq_s), true, true);
    //SENSOR_LOG_INFO("exit\n");
	mutex_unlock(&chip->lock);
};

static void akm8789_irq_work_n(struct work_struct *work)
{
	struct akm8789_chip *chip = container_of(work, struct akm8789_chip, irq_work_n);
	mutex_lock(&chip->lock);
    //SENSOR_LOG_INFO("enter\n");

    if (0 == gpio_get_value(chip->irq_n.irq_pin))
    {
        SENSOR_LOG_INFO("MAGNETIC_DEVICE NEAR\n");
        input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_NEAR);
        akm8789_wakelock_ops(&(chip->wakeup_wakelock),false);
    }
    else
    {
        SENSOR_LOG_INFO("MAGNETIC_DEVICE FAR\n");
        input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_FAR);
        hrtimer_start(&chip->unlock_wakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);
    }
    input_sync(chip->idev);

    chip->on_irq_working = false;

    akm8789_irq_enable(&(chip->irq_n), true, true);
    //SENSOR_LOG_INFO("exit\n");
	mutex_unlock(&chip->lock);
};


static void akm8789_check_state(struct akm8789_chip *chip)
{
    int pin_state_n = -1;
    int pin_state_s = -1;

    pin_state_n = gpio_get_value(chip->irq_n.irq_pin);
    pin_state_s = gpio_get_value(chip->irq_s.irq_pin);

    if ((1==pin_state_n) && (1==pin_state_s))
    {
        SENSOR_LOG_INFO("MAGNETIC_DEVICE FAR\n");
        input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_FAR);
        input_sync(chip->idev);
    }
    else
    {
        if ((0==pin_state_n) || (0==pin_state_s))
        {
            SENSOR_LOG_INFO("MAGNETIC_DEVICE NEAR\n");
            input_report_rel(chip->idev, REL_RX, MAGNETIC_DEVICE_NEAR);
            input_sync(chip->idev);
        }
    }

};



static irqreturn_t akm8789_irq_s(int irq, void *handle)
{
	struct akm8789_chip *chip = handle;
    //SENSOR_LOG_INFO("enter\n");
    akm8789_irq_enable(&(chip->irq_s), false, false);
	chip->on_irq_working = true;
	hrtimer_cancel(&chip->unlock_wakelock_timer);

    if (true == chip->enabled)
    {
        akm8789_wakelock_ops(&(chip->wakeup_wakelock),true);
    }

	if (0==schedule_work(&chip->irq_work_s))
    {
        SENSOR_LOG_INFO("schedule_work failed!\n");
    }

    //SENSOR_LOG_INFO("exit\n");

	return IRQ_HANDLED;
}

static irqreturn_t akm8789_irq_n(int irq, void *handle)
{    
	struct akm8789_chip *chip = handle;
    //SENSOR_LOG_INFO("enter\n");
    akm8789_irq_enable(&(chip->irq_n), false, false);
	chip->on_irq_working = true;
    hrtimer_cancel(&chip->unlock_wakelock_timer);

    if (true == chip->enabled)
    {
        akm8789_wakelock_ops(&(chip->wakeup_wakelock),true);
    }

	if (0==schedule_work(&chip->irq_work_n))
    {
        SENSOR_LOG_INFO("schedule_work failed!\n");
    }

    //SENSOR_LOG_INFO("exit\n");

	return IRQ_HANDLED;
}


static void akm8789_irq_enable(struct akm8789_irq * irq, bool enable, bool flag_sync)
{
    if (enable == irq->enabled)
    {
        SENSOR_LOG_INFO("doubule %s irq %d, retern here\n",enable? "enable" : "disable", irq->irq_num);
        return;
    }
    else
    {
        irq->enabled  = enable;
        SENSOR_LOG_INFO("%s irq %d\n",enable? "enable" : "disable",irq->irq_num);
    }

    if (enable)
    {
        enable_irq(irq->irq_num);
    }
    else
    {
        if (flag_sync)
        {
            disable_irq(irq->irq_num);
        }
        else
        {
            disable_irq_nosync(irq->irq_num);
        }
    }
}


static void akm8789_enable(struct akm8789_chip *chip, int on)
{
    SENSOR_LOG_INFO("%s hall_device\n",on? "enable" : "disable");

	if (on) 
    {
		akm8789_irq_enable(&(chip->irq_s), true, true);
		akm8789_irq_enable(&(chip->irq_n), true, true);
        akm8789_check_state(chip);
	} 
    else 
    {
        akm8789_irq_enable(&(chip->irq_s), false, true);
        akm8789_irq_enable(&(chip->irq_n), false, true);
    }
}


static ssize_t akm8789_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm8789_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->enabled);
}

static ssize_t akm8789_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct akm8789_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
    mutex_lock(&chip->lock);

    chip->enabled = (value>0) ? true : false;
    akm8789_enable(chip, chip->enabled);

    mutex_unlock(&chip->lock);

	return size;
}


static struct device_attribute attrs_hall_device[] = {
	__ATTR(enable,                          0640,   akm8789_enable_show,            akm8789_enable_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_hall_device); i++)
		if (device_create_file(dev, attrs_hall_device + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_hall_device + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void akm8789_chip_data_init(struct akm8789_chip *chip)
{
    chip->enabled = false;
    chip->irq_s.enabled = true;
    chip->irq_n.enabled = true;
    chip->wakeup_wakelock.name = "hall_device_wakelock";
    chip->wakeup_wakelock.locked = false;
    chip->on_irq_working = false;

}


static int __devinit akm8789_probe(struct i2c_client *client,
                  const struct i2c_device_id *id)
{
    int ret = 0;
	static struct akm8789_chip *chip;
    struct device_node *np = (client->dev).of_node;

    SENSOR_LOG_INFO("prob start\n");

    chip = kzalloc(sizeof(struct akm8789_chip), GFP_KERNEL);
    if (!chip) {
        ret = -ENOMEM;
        goto malloc_failed;
    }

    akm8789_chip_data_init(chip);

	chip->client = client;
	i2c_set_clientdata(client, chip);

    chip->irq_s.irq_pin = of_get_gpio(np, 0);
    chip->irq_n.irq_pin = of_get_gpio(np, 1);

    SENSOR_LOG_INFO("hall_device_int_s is %d",chip->irq_s.irq_pin);
    SENSOR_LOG_INFO("hall_device_int_n is %d",chip->irq_n.irq_pin);

	mutex_init(&chip->lock);


    hall_device_class   = class_create(THIS_MODULE, "hall_device");

    chip->hall_device_dev = device_create(hall_device_class, NULL, hall_device_dev_t, &akm8789_driver ,"hall_device");
    if (IS_ERR(chip->hall_device_dev)) 
    {
       ret = PTR_ERR(chip->hall_device_dev);
       goto create_hall_device_dev_failed;
    }

	dev_set_drvdata(chip->hall_device_dev, chip);


    ret = gpio_request(chip->irq_s.irq_pin, "chip->irq_s.irq_pin");
    if (ret)    
    {
        SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_s.irq_pin);
        
        gpio_free(chip->irq_s.irq_pin);
        ret = gpio_request(chip->irq_s.irq_pin, "chip->irq_s.irq_pin");
        if (ret) 
        {
            SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_s.irq_pin);
            return ret;
        }
    }
    
    ret = gpio_tlmm_config(GPIO_CFG(chip->irq_s.irq_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    chip->irq_s.irq_num = gpio_to_irq(chip->irq_s.irq_pin);
    INIT_WORK(&chip->irq_work_s, akm8789_irq_work_s);
    ret = request_threaded_irq(chip->irq_s.irq_num, NULL, &akm8789_irq_s, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "hall_device_irq_s", chip);
    if (ret) {
        SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->irq_s.irq_num);
        goto irq_s_register_fail;
    }


    ret = gpio_request(chip->irq_n.irq_pin, "chip->irq_n.irq_pin");
    if (ret)    
    {
        SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_n.irq_pin);
        
        gpio_free(chip->irq_n.irq_pin);
        ret = gpio_request(chip->irq_n.irq_pin, "chip->irq_n.irq_pin");
        if (ret) 
        {
            SENSOR_LOG_INFO("gpio %d is busy and then to free it\n",chip->irq_n.irq_pin);
            return ret;
        }
    }
    
    ret = gpio_tlmm_config(GPIO_CFG(chip->irq_n.irq_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    chip->irq_n.irq_num = gpio_to_irq(chip->irq_n.irq_pin);
    INIT_WORK(&chip->irq_work_n, akm8789_irq_work_n);
    ret = request_threaded_irq(chip->irq_n.irq_num , NULL, &akm8789_irq_n, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "hall_device_irq_n", chip);
    if (ret) {
        SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->irq_n.irq_num );
        goto irq_n_register_fail;
    }

    chip->idev = input_allocate_device();
    if (!chip->idev) 
    {
        SENSOR_LOG_ERROR("no memory for idev\n");
        ret = -ENODEV;
        goto input_alloc_failed;
    }
    chip->idev->name = "hall_device";
    chip->idev->id.bustype = BUS_I2C;

    set_bit(EV_REL,     chip->idev->evbit);
    set_bit(REL_RX,     chip->idev->relbit);  //NEAR
    set_bit(REL_RY,     chip->idev->relbit);  //FAR


    ret = input_register_device(chip->idev);
    if (ret) {
        input_free_device(chip->idev);
        SENSOR_LOG_ERROR("cant register input '%s'\n",chip->idev->name);
        goto input_register_failed;
    }

    create_sysfs_interfaces(chip->hall_device_dev);

    akm8789_irq_enable(&(chip->irq_s), false, true);
    akm8789_irq_enable(&(chip->irq_n), false, true);

    wake_lock_init(&chip->wakeup_wakelock.lock, WAKE_LOCK_SUSPEND, chip->wakeup_wakelock.name);
    hrtimer_init(&chip->unlock_wakelock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    chip->unlock_wakelock_timer.function = akm8789_unlock_wakelock_work_func;


    SENSOR_LOG_INFO("prob success\n");

    return 0;

input_register_failed:
    input_free_device(chip->idev);
input_alloc_failed:
malloc_failed:
irq_n_register_fail:
irq_s_register_fail:
create_hall_device_dev_failed:
    chip->hall_device_dev = NULL;
    class_destroy(hall_device_class);
    SENSOR_LOG_INFO("prob failed\n");

    return -1;

}

//resume
static int akm8789_resume(struct device *dev)
{
	struct akm8789_chip *chip = dev_get_drvdata(dev);

    SENSOR_LOG_INFO("enter\n");
    if (true==chip->enabled)
    {
        disable_irq_wake(chip->irq_s.irq_num);
        disable_irq_wake(chip->irq_n.irq_num);
    }
    SENSOR_LOG_INFO("eixt\n");
    return 0 ;
}
 
//suspend  
static int akm8789_suspend(struct device *dev)
{
	struct akm8789_chip *chip = dev_get_drvdata(dev);

    SENSOR_LOG_INFO("enter\n");
    if (true==chip->enabled)
    {
        enable_irq_wake(chip->irq_s.irq_num);
        enable_irq_wake(chip->irq_n.irq_num);
    }
    SENSOR_LOG_INFO("eixt\n");
    return 0 ;
}

 
 /**
  * akm8789_remove() - remove device
  * @client: I2C client device
  */
 static int __devexit akm8789_remove(struct i2c_client *client)
 {
     struct shtc1_data *chip_data = i2c_get_clientdata(client);
 
     SENSOR_LOG_INFO("akm8789_remove\n");
    
     kfree(chip_data);
     return 0;
 }

 
MODULE_DEVICE_TABLE(i2c, akm8789_idtable);
 
module_init(akm8789_init);
module_exit(akm8789_exit);
 
MODULE_DESCRIPTION("AKM akm8789 driver");
MODULE_AUTHOR("ZhuBing, ZTEMT");
MODULE_LICENSE("GPL");
