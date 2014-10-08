#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/i2c/tmg399x.h>
#include <linux/i2c/taos_common.h>
#include <linux/qpnp/qpnp-device-info.h>


#define LOG_TAG "SENSOR_ALS_PROX_COMMON"
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

extern struct i2c_driver tmd2772_driver;
extern struct i2c_driver tmg399x_driver;
extern int ztemt_get_device_index(char* result);


static int __init als_prox_init(void) 
{
    int project_id = ztemt_get_device_index(NULL);

    SENSOR_LOG_INFO("project_id is %d\n",project_id);

    if (DEVICE_01AMB_B_WTR1605_L_EMMC_16_32 == project_id)
    {
        SENSOR_LOG_INFO("add tmd2772 driver\n");
        return i2c_add_driver(&tmd2772_driver);

    }
    else
    {        
        SENSOR_LOG_INFO("add tmg399x driver\n");
	    return i2c_add_driver(&tmg399x_driver);
    }
}


module_init(als_prox_init);

