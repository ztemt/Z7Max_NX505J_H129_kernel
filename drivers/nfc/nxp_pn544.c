#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/pn544.h>
#include <linux/fs.h>

#define PN544_DRIVER_NAME	"pn544"
//#denfie NFC_HOST_OFF_CE

//#define NXP_PN544_DEBUG

#define DRIVER_DESC	"NFC driver for PN544"

#define MAX_BUFFER_SIZE		512
#define PN544_MSG_MAX_SIZE	0x21 /* at normal HCI mode */

/* Timing restrictions (ms) */
#define PN544_RESETVEN_TIME	35 /* 7 */

//struct pn544_nfc_platform_data pn544_nfc_platform_data;

enum pn544_irq {
	PN544_NONE,
	PN544_INT,
};

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	//struct pn544_nfc_platform_data *pdata;
	struct mutex read_mutex;
	struct i2c_client	*client;
	struct miscdevice	miscdev;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	enum pn544_irq read_irq;
	
	int updata_gpio;
	int ven_gpio;
	int irq_gpio;
	int (*request_resources) (void);
	void (*free_resources) (void);
	void (*enable) (int fw);
	int (*test) (void);
	void (*disable) (void);
	int (*irq_status) (void);
};

static struct pn544_dev *pn544_dev;

#if 1
#define PN544_NFC_SW_UPDATE   (pn544_dev->updata_gpio) //#define PN544_NFC_SW_UPDATE 13
#define PN544_NFC_nVEN        (pn544_dev->ven_gpio) //#define PN544_NFC_nVEN      85
#define PN544_NFC_GPIO_INT    (pn544_dev->irq_gpio)//#define PN544_NFC_GPIO_INT  68


static int pn544_nfc_request_resources(void)
{
	printk("[%s] pn544_nfc_request_resources start.\n", __func__);
	
	if(gpio_request(PN544_NFC_SW_UPDATE, "NFC_SW_UPDATE")) {
		pr_err("failed request NFC_SW_UPDATE.\n");
		return -1;
	}
	gpio_tlmm_config(GPIO_CFG(PN544_NFC_SW_UPDATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_set_value(PN544_NFC_SW_UPDATE,0);
	#if 0
	error = gpio_direction_output(PN544_NFC_SW_UPDATE, 0);
	if (error) 
	{
		printk("unable to set direction for gpio [%d]\n",pn544_dev->updata_gpio);
	}
	#endif
	if(gpio_request(PN544_NFC_nVEN, "NFC_VEN")) {
		pr_err("failed request NFC_VEN.\n");
		return -1;
	}
	gpio_direction_output(PN544_NFC_nVEN, 1);
	gpio_set_value_cansleep(PN544_NFC_nVEN,1);
	
	if(gpio_request(PN544_NFC_GPIO_INT, "NFC_IRQ")) {
		pr_err("failed request NFC_IRQ.\n");
		goto err_rst_gpio_req;
	}	
	gpio_tlmm_config(GPIO_CFG(PN544_NFC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	#if 0
	error = gpio_direction_input(pn544_dev->irq_gpio);
	if (error) 
	{
		printk("unable to set direction for gpio [%d]\n",pn544_dev->irq_gpio);
	}
	#endif
	return 0;
err_rst_gpio_req:
	gpio_free(PN544_NFC_SW_UPDATE);
	gpio_free(PN544_NFC_nVEN);
	return -1;
}
static void pn544_nfc_free_resources(void)
{
	/* Release the HW resources */
	printk("%s:goto pn544_nfc_free_resources\n",__func__);
	//pmapp_clock_vote("NNFC", PMAPP_CLOCK_ID_A1, PMAPP_CLOCK_VOTE_OFF);
	gpio_tlmm_config(GPIO_CFG(PN544_NFC_SW_UPDATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(PN544_NFC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_free(PN544_NFC_SW_UPDATE);
	gpio_free(PN544_NFC_nVEN);
	gpio_free(PN544_NFC_GPIO_INT);
}
static void pn544_nfc_enable(int fw)
{
	printk("%s:goto pn544_nfc_enable\n",__func__);
	/* Turn the device on */
    if (HCI_MODE == fw )
    {
		printk("%s:enable HCI_MODE pn544\n",__func__);
        gpio_set_value(PN544_NFC_SW_UPDATE, 0);
        gpio_set_value_cansleep(PN544_NFC_nVEN,0);
    }
    else // FW_MODE
    {
        
		printk("%s:enable FW_MODE pn544\n",__func__);
		gpio_set_value_cansleep(PN544_NFC_nVEN, 0);/* 1 */
        gpio_set_value(PN544_NFC_SW_UPDATE, 1); /* 1 */
        msleep(10);
        gpio_set_value_cansleep(PN544_NFC_nVEN, 1);/* 0 */
        msleep(50);
        gpio_set_value_cansleep(PN544_NFC_nVEN, 0);/* 1 */
        msleep(10);
    }
}
static int pn544_nfc_test(void)
{
	/*
	 * Put the device into the FW update mode
	 * and then back to the normal mode.
	 * Check the behavior and return one on success,
	 * zero on failure.
	 */
	return 0;
}

static void pn544_nfc_disable(void)
{
	/* turn the power off */
 	gpio_set_value(PN544_NFC_SW_UPDATE, 0);
	gpio_set_value_cansleep(PN544_NFC_nVEN, 1);
}

static int pn544_pt_irq_status(void)
{
  return (gpio_get_value(PN544_NFC_GPIO_INT) != 0);
}
#endif //CONFIG_PN544_NFC

static int pn544_dev_enable(struct pn544_dev *dev, int mode)
{
#ifdef NXP_PN544_DEBUG
    printk("%s: mode: %d\n", __func__, mode);
#endif

	dev->read_irq = PN544_NONE;
	if (dev->enable)
		dev->enable(mode);
	usleep_range(10000, 15000);
	//usleep_range(20000, 35000);
	return 0;
}

static void pn544_dev_disable(struct pn544_dev *dev)
{
	//struct i2c_client *client = dev->client;
	
	if (dev->disable)
		dev->disable();

	msleep(PN544_RESETVEN_TIME);

	dev->read_irq = PN544_NONE;

#ifdef NXP_PN544_DEBUG
	//dev_dbg(&client->dev, "%s: Now in OFF-mode\n", __func__);//delete by chengdongsheng 
    printk(KERN_DEBUG "%s: Now in OFF-mode\n", __func__);
#endif
}

static int pn544_irq_status(struct pn544_dev *dev)
{
	if ( dev->irq_status ) {
		return 0;
	} else {
		return dev->irq_status();
	}
}

static void pn544_disable_irq(struct pn544_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->irq_enabled_lock, flags);
	if (dev->irq_enabled) {
		disable_irq_nosync(dev->client->irq);
		dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&dev->irq_enabled_lock, flags);
}


static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = pn544_dev;
	
	printk("%s : (imajor) %d, (iminor) %d\n", __func__, imajor(inode), iminor(inode));
	
	return pn544_dev_enable(pn544_dev, HCI_MODE);
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	//ztemt changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn544_dev *dev = filp->private_data;	
	struct pn544_dev *dev = pn544_dev;
	//ztemt end
	struct i2c_client *client = dev->client;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

#ifdef NXP_PN544_DEBUG
    printk(KERN_DEBUG "%s: reading %zu bytes.\n", __func__, count);
#endif
	mutex_lock(&dev->read_mutex);

	if ( !pn544_irq_status(dev) ) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		dev->irq_enabled = true;
		enable_irq(dev->client->irq);
		ret = wait_event_interruptible(dev->read_wq, 
			(dev->read_irq == PN544_INT));
		pn544_disable_irq(dev);

		if (ret)
			goto fail;
	}
	/* Read data */
	ret = i2c_master_recv(dev->client, tmp, count);
	dev->read_irq = PN544_NONE;
	mutex_unlock(&dev->read_mutex);

	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c_master_recv returned %d\n", 
				__func__, ret);
		return ret;
	}
	if (ret > count) {
		dev_err(&client->dev, "%s: received too many bytes from i2c (%d)\n",
				__func__, ret);
		return -EIO;
	}
#ifdef NXP_PN544_DEBUG
	print_hex_dump(KERN_DEBUG, " read: ", DUMP_PREFIX_NONE, 16, 1, tmp, ret, false);
#endif
	if (copy_to_user(buf, tmp, ret)) {
		dev_err(&client->dev, "%s : failed to copy to user space\n", 
				__func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	//ztem changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn544_dev *dev = filp->private_data;
	struct pn544_dev *dev = pn544_dev;
	//ztemt end
	struct i2c_client *client = dev->client;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&client->dev, "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

#ifdef NXP_PN544_DEBUG
	dev_dbg(&client->dev, "%s : writing %zu bytes.\n", __func__, count);
	print_hex_dump(KERN_DEBUG, "write: ", DUMP_PREFIX_NONE, 16, 1, tmp, count, false);
#endif
	/* Write data */
	ret = i2c_master_send(client, tmp, count);
	if (ret != count) {
		dev_err(&client->dev, "%s : addr is 0x%x, i2c_master_send returned %d\n", __func__, client->addr, ret);
		ret = -EIO;
	}

	return ret;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{	
	//ztemt changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn544_dev *dev = filp->private_data;
	struct pn544_dev *dev = pn544_dev;	
	//ztemt end
	struct i2c_client *client = dev->client;
	unsigned int val;
	int r = 0;

#ifdef NXP_PN544_DEBUG
	dev_dbg(&client->dev, "%s: cmd: 0x%x,PN544_SET_PWR:0x%x\n", __func__, cmd, PN544_SET_PWR);
#endif
	switch (cmd) {
		case PN544_SET_PWR:
			val = arg;
#ifdef NXP_PN544_DEBUG
			dev_dbg(&client->dev, "%s:  PN544_SET_PWR: %d\n", __func__, val);
#endif
			switch (val) {
				case 0: // power off
					pn544_dev_disable(dev);
#ifdef NFC_HOST_OFF_CE
					irq_set_irq_wake(pn544_dev->client->irq,0);
					msleep(10);
#endif
					break;
				case 1: // power on
					r = pn544_dev_enable(dev, HCI_MODE);
					if (r < 0)
						goto out;
#ifdef NFC_HOST_OFF_CE
					irq_set_irq_wake(pn544_dev->client->irq,1);
					msleep(10);
#endif
					break;
				case 2: // reset and power on with firmware download enabled
					r = pn544_dev_enable(dev, FW_MODE);
					if (r < 0)
						goto out;
#ifdef NFC_HOST_OFF_CE
					irq_set_irq_wake(pn544_dev->client->irq,1);
					msleep(10);
#endif
					break;
				default:
					r = -ENOIOCTLCMD;
					goto out;
					break;
			}
			break;
		default:
			dev_err(&client->dev, "%s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}
out:
	return r;
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *dev = dev_id;

	pn544_disable_irq(dev);

	//dev_dbg(&dev->client->dev, "IRQ\n");

	dev->read_irq = PN544_INT;
	
	/* Wake up waiting readers */
	wake_up(&dev->read_wq);

	return IRQ_HANDLED;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.open	= pn544_dev_open,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.unlocked_ioctl = pn544_dev_ioctl,
};

#ifdef NXP_PN544_STANDBY_MODE
static void set_standby_mode(struct pn544_dev *pn544dev)
{
	char standby_commands[] = {0x09, 0x9B, 0x82, 0x3F, 0x00, 0x9E, 0xAA, 0x01, 0x9B, 0x03};
	int ret = -1;

	pn544dev = pn544_dev;
	if(pn544dev == NULL){
		printk(KERN_ERR "[%s]dev is NULL.\n", __func__);
		return;
	}
	
	//Reset and write standy mode commands
	ret = pn544_dev_enable(pn544dev, HCI_MODE);
	if (ret < 0){
		printk(KERN_ERR "[%s]enable nfc failed.\n", __func__);
		return;
	}
	
	pn544_dev_disable(pn544dev);

	ret = pn544_dev_enable(pn544dev, HCI_MODE);
	if (ret < 0){
		printk(KERN_ERR "[%s]enable nfc failed.\n", __func__);
		return;
	}

	ret = i2c_master_send(pn544dev->client , standby_commands, sizeof(standby_commands));
	if (ret != sizeof(standby_commands)) {
		printk(KERN_ERR "[%s]i2c_master_send failed. returned %d\n", __func__, ret);
		return;
	}
	pn544_dev_disable(pn544dev);
	
    printk(KERN_DEBUG "[%s]Reset and write standby commands successfully.write bytes:%d\n", __func__, ret);
}
#endif

static __devinit int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int irq_gpio = -1;
	int updata_gpio = -1;
	int ven_gpio = -1;
	int irq;
	int addr;
	int ret;

	dev_dbg(&client->dev, "IRQ: %d\n", client->irq);

	if(client->dev.of_node)
	{
		irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,irq-gpio", 0, NULL);
		updata_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,updata-gpio", 0, NULL);
		ven_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,ven-gpio", 0, NULL);
		printk("pn544,--irq_gpio---:%d\n", irq_gpio);
		printk("pn544,--updata_gpio---:%d\n", updata_gpio);
		printk("pn544,--ven_gpio---:%d\n", ven_gpio);
	}

	irq = client->irq;
	addr = client->addr;

	if (pn544_dev != NULL) {
		dev_warn(&client->dev, "only one PN544 supported.\n");
		return -EBUSY;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	
	pn544_dev = kzalloc(sizeof(struct pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	pn544_dev->client = client;
	pn544_dev->irq_gpio = irq_gpio;
	pn544_dev->updata_gpio = updata_gpio;
	pn544_dev->ven_gpio = ven_gpio;
	pn544_dev->request_resources = pn544_nfc_request_resources;
	pn544_dev->free_resources = pn544_nfc_free_resources;
	pn544_dev->enable = pn544_nfc_enable;
	pn544_dev->disable = pn544_nfc_disable;
	pn544_dev->test = pn544_nfc_test;
	pn544_dev->irq_status = pn544_pt_irq_status;
	/* init mutex and queues */
	pn544_dev->read_irq = PN544_NONE;
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);
	
	i2c_set_clientdata(client, pn544_dev);

	if (!pn544_dev->request_resources) {
		dev_err(&client->dev, "request_resources() missing\n");
		ret = -EINVAL;
		goto err_request_resources;
	}

	ret = pn544_dev->request_resources();
	if (ret < 0) {
		dev_err(&client->dev, "Cannot get platform resources\n");
		goto err_request_resources;
	}
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	printk("--client->irq:%d,client->name:%s\n", client->irq,client->name);
	
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq;
	}
	
	printk("pn544_dev->client->irq:%d\n",pn544_dev->client->irq);
	pn544_disable_irq(pn544_dev);	

	pn544_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->miscdev.name = PN544_DRIVER_NAME;
	pn544_dev->miscdev.fops = &pn544_dev_fops;
	pn544_dev->miscdev.parent = &client->dev;
	ret = misc_register(&pn544_dev->miscdev);
	if (ret) {
		dev_err(&client->dev, "%s : misc_register failed\n", __func__);
		goto err_misc_register;
	}

	printk("%s: dev: %p, client %p\n",__func__, pn544_dev, client);

#ifdef NXP_PN544_STANDBY_MODE
	set_standby_mode(pn544_dev);
#endif

	return 0;

err_misc_register:
	free_irq(client->irq, pn544_dev);
err_request_irq:
	if (pn544_dev->free_resources)
		pn544_dev->free_resources();
err_request_resources:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_exit:
	return ret;
	
}

static __devexit int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);

	misc_deregister(&dev->miscdev);
	if (dev->disable)
		dev->disable();
	dev->read_irq = PN544_NONE;
	free_irq(client->irq, dev);
	if (dev->free_resources)
		dev->free_resources();
	mutex_destroy(&dev->read_mutex);
	kfree(dev);

	pn544_dev = NULL;
	
	return 0;
}

static struct of_device_id nxp_match_table[] = {
	{ .compatible = "nxp,i2c_adapter",},
	{ },
};

static const struct i2c_device_id pn544_id[] = {
	{ PN544_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN544_DRIVER_NAME,
		.of_match_table = nxp_match_table,
	},
};

module_i2c_driver(pn544_driver);

MODULE_DESCRIPTION("I2C_TEST_NXP");
MODULE_LICENSE("GPL");
