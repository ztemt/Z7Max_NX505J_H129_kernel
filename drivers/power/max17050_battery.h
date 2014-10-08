#ifndef __LINUX_MAX17050_CHARGER_H__
#define __LINUX_MAX17050_CHARGER_H__

#define DEFAULT_TEMP 250
#define DEFAULT_SOC 50


int max17050_get_batt_voltage(void);
int max17050_get_batt_temp(void);
int max17050_get_batt_soc(void);
int max17050_report_batt_capacity(void);
int max17050_get_ibatt_now(void);
int max17050_set_power_supply(struct power_supply *batt_psy);

#endif
