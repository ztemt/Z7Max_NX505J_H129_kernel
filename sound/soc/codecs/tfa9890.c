/*
 * File:         sound/soc/codecs/ssm2602.c
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Tue June 06 2008
 * Description:  Driver for ssm2602 sound chip
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>

static int __devinit tfa9890_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
    gpio_set_value(56,0);
    printk("%s... getvalue %d\n",__func__,gpio_get_value(56));
	return 0;
}

static int __devexit tfa9890_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tfa9890_i2c_id[] = {
	{ "nxp,tfa9890", 0},
	{ },
};

static const struct of_device_id of_tfa_device_idtable[] = {
    { .compatible = "nxp,tfa9890",0},
	{ },
};

/* corgi i2c codec control layer */
static struct i2c_driver tfa9890_i2c_driver = {
	.driver = {
		.name = "tfa9890",
        .of_match_table = of_tfa_device_idtable,
		.owner = THIS_MODULE,
	},
	.probe = tfa9890_i2c_probe,
	.remove = __devexit_p(tfa9890_i2c_remove),
	.id_table = tfa9890_i2c_id,
};


static int __init tfa9890_modinit(void)
{
	int ret = 0;

    printk("%s ============",__func__);
	ret = i2c_add_driver(&tfa9890_i2c_driver);
    return ret;
    
}
module_init(tfa9890_modinit);

static void __exit tfa9890_exit(void)
{
	i2c_del_driver(&tfa9890_i2c_driver);
}
module_exit(tfa9890_exit);

MODULE_DESCRIPTION("ASoC SSM2602/SSM2603/SSM2604 driver");
MODULE_AUTHOR("Cliff Cai");
MODULE_LICENSE("GPL");
