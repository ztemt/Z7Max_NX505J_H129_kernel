/*
* pn547.h
* Copyright (C) 2011	NXP Semiconductors
*/

//ALERT:please relocate pn547.h under .\kernel\include\linux

#define PN547_DRIVER_NAME	"pn547"

#define HCI_MODE	0
#define FW_MODE		1

#define PN547_MAGIC	0xE9

/*
 * PN547 power control via ioctl
 * PN547_SET_PWR(0): power off
 * PN547_SET_PWR(1): power on
 * PN547_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN547_SET_PWR	_IOW(PN547_MAGIC, 0x01, unsigned int)

#ifdef __KERNEL__
/* board config */
struct pn547_nfc_platform_data {
	int (*request_resources) (struct i2c_client *client);
	void (*free_resources) (void);
	void (*enable) (int fw);
	int (*test) (void);
	void (*disable) (void);
	int (*irq_status) (void);
};
#endif /* __KERNEL__ */