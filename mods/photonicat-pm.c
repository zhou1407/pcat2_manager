// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/completion.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/hwmon.h>
#include <linux/miscdevice.h>

#define PCAT_PM_READ_BUFFER_SIZE 4096
#define PCAT_PM_WATCHDOG_DEFAULT_INTERVAL 10

typedef enum {
	PCAT_PM_COMMAND_HEARTBEAT = 0x1,
	PCAT_PM_COMMAND_HEARTBEAT_ACK = 0x2,
	PCAT_PM_COMMAND_PMU_HW_VERSION_GET = 0x3,
	PCAT_PM_COMMAND_PMU_HW_VERSION_GET_ACK = 0x4,
	PCAT_PM_COMMAND_PMU_FW_VERSION_GET = 0x5,
	PCAT_PM_COMMAND_PMU_FW_VERSION_GET_ACK = 0x6,
	PCAT_PM_COMMAND_STATUS_REPORT = 0x7,
	PCAT_PM_COMMAND_STATUS_REPORT_ACK = 0x8,
	PCAT_PM_COMMAND_DATE_TIME_SYNC = 0x9,
	PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK = 0xA,
	PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET = 0xB,
	PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET_ACK = 0xC,
	PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN = 0xD,
	PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN_ACK = 0xE,
	PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN = 0xF,
	PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK = 0x10,
	PCAT_PM_COMMAND_PMU_REQUEST_FACTORY_RESET = 0x11,
	PCAT_PM_COMMAND_PMU_REQUEST_FACTORY_RESET_ACK = 0x12,
	PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET = 0x13,
	PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET_ACK = 0x14,
	PCAT_PM_COMMAND_CHARGER_ON_AUTO_START = 0x15,
	PCAT_PM_COMMAND_CHARGER_ON_AUTO_START_ACK = 0x16,
	PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET = 0x17,
	PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET_ACK = 0x18,
	PCAT_PM_COMMAND_NET_STATUS_LED_SETUP = 0x19,
	PCAT_PM_COMMAND_NET_STATUS_LED_SETUP_ACK = 0x1A,
	PCAT_PM_COMMAND_POWER_ON_EVENT_GET = 0x1B,
	PCAT_PM_COMMAND_POWER_ON_EVENT_GET_ACK = 0x1C,
}PCatPMCommandType;

static enum power_supply_property pcat_pm_battery_v1_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property pcat_pm_battery_v2_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property pcat_pm_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

struct pcat_pm_data {
	struct serdev_device *serdev;
	struct gpio_desc *power_gpio;
	struct power_supply *battery_psy;
	struct power_supply *charger_psy;
	struct kthread_worker *kworker;
	struct kthread_work check_work;
	struct hrtimer check_timer;
	struct power_supply_battery_info *battery_info;
	struct rtc_device *rtc;
	struct device *hwmon_dev;
	struct miscdevice ctl_device;
	
	u32 pm_version;
	bool work_flag;
	bool poweroff_ok;
	u32 baudrate;
	u32 force_poweroff_timeout;
	u16 write_framenum;
	
	u8 read_buffer[PCAT_PM_READ_BUFFER_SIZE];
	size_t read_buffer_used;
	
	u8 ctl_write_buffer[4096];
	size_t ctl_write_buffer_used;
	
	unsigned int battery_technology;
	int battery_design_uwh;
	int battery_design_min_uv;
	int battery_design_max_uv;
	struct mutex charger_mutex;
	int battery_voltage_now;
	int charger_voltage_now;
	int battery_current_now;
	int battery_soc;
	bool on_battery;
	bool on_charger;
	int board_temp;
	
	u16 rtc_year;
	u8 rtc_month;
	u8 rtc_day;
	u8 rtc_hour;
	u8 rtc_min;
	u8 rtc_sec;
	u8 rtc_status;
};

static inline u16 pcat_pm_compute_crc16(const u8 *data, size_t len) 
{
	u16 crc = 0xFFFF;
	size_t i;
	unsigned int j;

	for (i=0;i<len;i++) {
		crc ^= data[i];
		for (j=0;j<8;j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}

	return crc;
}

static int pcat_pm_uart_write_data(struct pcat_pm_data *pm_data,
	u16 command, const u8 *extra_data, u16 extra_data_len,
	bool need_ack, long timeout)
{
	u8 data[1024];
	size_t data_size = 0;
	u16 sv;
	u16 dp_size;
	int ret;
	
	memcpy(data, (const u8 *)"\xA5\x01\x81", 3);
	data_size += 3;
	
	data[data_size] = pm_data->write_framenum & 0xFF;
	data[data_size+1] = (pm_data->write_framenum >> 8)& 0xFF;
	data_size += 2;
	pm_data->write_framenum++;
	
	if (extra_data!=NULL && extra_data_len > 0 && extra_data_len <= 512) {
		sv = extra_data_len + 3;
		data[data_size] = sv & 0xFF;
		data[data_size+1] = (sv >> 8) & 0xFF;
		data_size += 2;
		
		sv = command;
		data[data_size] = sv & 0xFF;
		data[data_size+1] = (sv >> 8) & 0xFF;
		data_size += 2;
		
		memcpy(data+data_size, extra_data, extra_data_len);
		data_size += extra_data_len;

		dp_size = extra_data_len + 3;
	} else {
		sv = 3;
		data[data_size] = sv & 0xFF;
		data[data_size+1] = (sv >> 8) & 0xFF;
		data_size += 2;
		
		sv = command;
		data[data_size] = sv & 0xFF;
		data[data_size+1] = (sv >> 8) & 0xFF;
		data_size += 2;
		
		dp_size = 3;
	}
	
	data[data_size] = need_ack ? 1 : 0;
	data_size++;
	
	sv = pcat_pm_compute_crc16(data + 1, dp_size + 6);
	data[data_size] = sv & 0xFF;
	data[data_size+1] = (sv >> 8) & 0xFF;
	data_size += 2;
	
	data[data_size] = 0x5A;
	data_size++;
	
	if (timeout > 0)
		ret = serdev_device_write(pm_data->serdev, data, data_size, timeout);
	else
		ret = serdev_device_write_buf(pm_data->serdev, data, data_size);
	
	if (ret < 0)
		dev_err(&pm_data->serdev->dev, "Failed to write serial port: %d\n", ret);
	
	return ret;
}

static void pcat_pm_watchdog_timeout_set(struct pcat_pm_data *pm_data,
	u8 interval, long timeout)
{
	u8 timeouts[3] = {60, pm_data->force_poweroff_timeout, interval};

	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET,
		timeouts, 3, false, timeout);
}

static void pcat_pm_worker_stop(struct pcat_pm_data *pm_data)
{
	if (IS_ERR(pm_data->kworker))
		return;

	hrtimer_cancel(&pm_data->check_timer);
	kthread_cancel_work_sync(&pm_data->check_work);
}

static int pcat_pm_battery_get_prop(struct power_supply *ps,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct pcat_pm_data *pm_data = power_supply_get_drvdata(ps);
	switch (prop) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pm_data->battery_technology;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "photonicat-pm";
		break;
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&pm_data->charger_mutex);
		if (pm_data->on_battery) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			if (pm_data->battery_soc >= 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		mutex_unlock(&pm_data->charger_mutex);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pm_data->battery_design_max_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pm_data->battery_design_min_uv;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = pm_data->battery_design_uwh;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = pm_data->battery_design_uwh;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&pm_data->charger_mutex);
		val->intval = pm_data->battery_voltage_now;
		mutex_unlock(&pm_data->charger_mutex);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&pm_data->charger_mutex);
		val->intval = pm_data->battery_current_now;
		mutex_unlock(&pm_data->charger_mutex);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!pm_data->battery_info)
			return -ENODEV;
		mutex_lock(&pm_data->charger_mutex);
		val->intval = pm_data->battery_soc;
		mutex_unlock(&pm_data->charger_mutex);
		if (val->intval > 100)
			val->intval = 100;
		else if (val->intval < 0)
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pcat_pm_charger_get_prop(struct power_supply *ps,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct pcat_pm_data *pm_data = power_supply_get_drvdata(ps);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pm_data->on_charger ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&pm_data->charger_mutex);
		val->intval = pm_data->charger_voltage_now;
		mutex_unlock(&pm_data->charger_mutex);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc pcat_pm_battery_v1_desc = {   
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pcat_pm_battery_v1_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_battery_v1_properties), 
	.get_property = pcat_pm_battery_get_prop,
};

static const struct power_supply_desc pcat_pm_battery_v2_desc = {   
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pcat_pm_battery_v2_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_battery_v2_properties), 
	.get_property = pcat_pm_battery_get_prop,
};

static const struct power_supply_desc pcat_pm_charger_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = pcat_pm_ac_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_ac_properties),
	.get_property = pcat_pm_charger_get_prop,
};

static int pcat_pm_do_poweroff(struct sys_off_data *data)
{
	struct pcat_pm_data *pm_data = data->cb_data;
	unsigned int try_count;

	dev_info(&pm_data->serdev->dev, "Shutdown process starts.\n");

	pm_data->work_flag = false;
	pcat_pm_worker_stop(pm_data);

	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN,
		NULL, 0, true, msecs_to_jiffies(1000));

	for (try_count = 0;try_count < 50;try_count++) {
		if (pm_data->poweroff_ok)
			break;
		mdelay(100);
	}
	
	if (pm_data->poweroff_ok)
		dev_info(&pm_data->serdev->dev, "Shutdown request received.\n");
	else
		dev_err(&pm_data->serdev->dev, "Shutdown request timeout!\n");

	if (!IS_ERR(pm_data->power_gpio))
		gpiod_direction_output(pm_data->power_gpio, 0);
	mdelay(100);

	WARN_ON(1);

	return NOTIFY_DONE;
}

static int pcat_pm_do_restart(struct sys_off_data *data)
{
	struct pcat_pm_data *pm_data = data->cb_data;

	dev_info(&pm_data->serdev->dev, "Reboot process starts.\n");

	pcat_pm_watchdog_timeout_set(pm_data, 120, msecs_to_jiffies(1000));

	mdelay(100);

	pm_data->work_flag = false;
	pcat_pm_worker_stop(pm_data);

	dev_info(&pm_data->serdev->dev, "Reboot process completed.\n");

	return NOTIFY_DONE;
}

static void pcat_pm_status_report_parse(struct pcat_pm_data *pm_data,
	const u8 *data, size_t data_len)
{
	u16 battery_voltage, charger_voltage;
	u16 gpio_input, gpio_output;
	int temp = 0;
	u16 battery_current_raw = 0;
	s16 battery_current;
	bool on_battery;
	int soc;

	if (data_len < 16)
		return;
		
	battery_voltage = data[0] | ((u16)data[1] << 8);
	charger_voltage = data[2] + ((u16)data[3] << 8);
	gpio_input = data[4] + ((u16)data[5] << 8);
	gpio_output = data[6] + ((u16)data[7] << 8);

	soc = power_supply_batinfo_ocv2cap(
		pm_data->battery_info, (int)battery_voltage * 1000, 20);

	if (data_len >= 20) {
		temp = (int)data[17] - 100;
		battery_current_raw = data[18] + ((u16)data[19] << 8);
		battery_current = (s16)battery_current_raw;
		on_battery = (battery_current > 0);
		
	} else {
		on_battery = (charger_voltage < 4200);
	}
	
	mutex_lock(&pm_data->charger_mutex);
	pm_data->battery_voltage_now = battery_voltage * 1000;
	pm_data->charger_voltage_now = charger_voltage * 1000;
	pm_data->battery_current_now = -battery_current * 1000;
	pm_data->battery_soc = soc;
	pm_data->on_battery = on_battery;
	pm_data->on_charger = (charger_voltage >= 4200);
	pm_data->rtc_year = data[8] + ((u16)data[9] << 8);
	pm_data->rtc_month = data[10];
	pm_data->rtc_day = data[11];
	pm_data->rtc_hour = data[12];
	pm_data->rtc_min = data[13];
	pm_data->rtc_sec = data[14];
	pm_data->rtc_status = data[15];
	pm_data->board_temp = temp;
	mutex_unlock(&pm_data->charger_mutex);
}

static size_t pcat_pm_uart_receive_parse(struct pcat_pm_data *pm_data)
{
	size_t used_size = 0, remaining_size;
	size_t i;
	u8 *buffer = pm_data->read_buffer;
	const u8 *p, *extra_data;
	u16 expect_len, extra_data_len;
	u16 checksum, rchecksum;
	u16 command;
	u8 src, dst;
	bool need_ack;
	u16 frame_num;
	
	if (pm_data->read_buffer_used < 13)
		return 0;

	for (i=0;i<pm_data->read_buffer_used;i++) {
		if (buffer[i]!=0xA5) {
			used_size = i + 1;
			continue;
		}
		
		p = buffer + i;
		remaining_size = pm_data->read_buffer_used - i;
		used_size = i + 1;

		if (remaining_size < 13)
			break;

		expect_len = p[5] + ((u16)p[6] << 8);
		if (expect_len < 3 || expect_len > 512) {
			used_size = i + 1;
			continue;
		}

		if (expect_len + 10 > remaining_size)
			break;
		if (p[9+expect_len]!=0x5A) {
			used_size = i + 1;
			continue;
		}
		
		checksum = p[7+expect_len] + ((u16)p[8+expect_len] << 8);
		rchecksum = pcat_pm_compute_crc16(p+1, 6+expect_len);
		
		if (checksum!=rchecksum) {
			dev_err(&pm_data->serdev->dev,
				"Serial port got incorrect checksum %X, "
				"should be %X!", checksum ,rchecksum);
			i += 9 + expect_len;
			used_size = i + 1;
			continue;
		}
		
		src = p[1];
		dst = p[2];
		frame_num = p[3] + ((u16)p[4] << 8);			
		command = p[7] + ((u16)p[8] << 8);
		extra_data_len = expect_len - 3;
		
		if (expect_len > 3)
			extra_data = p + 9;
		else
			extra_data = NULL;
		need_ack = (p[6 + expect_len]!=0);
		
		if (dst==0x1 || dst==0x80 || dst==0xFF) {
			switch (command) {
			case PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK:
				pm_data->poweroff_ok = true;
				need_ack = false;
				break;
			case PCAT_PM_COMMAND_STATUS_REPORT:
				pcat_pm_status_report_parse(
					pm_data, extra_data, extra_data_len);
				break;
			case PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK:
				if (extra_data_len > 0 && extra_data[0])
					dev_err(&pm_data->serdev->dev,
						"Failed to sync date: %d\n",
						extra_data[0]);
				break;
			default:
				dev_info(&pm_data->serdev->dev,
					"Got command %X from %X to %X, frame num %d, "
					"need ACK %d.\n", command, src, dst, frame_num, need_ack);		
				break;
			}
		}
		
		if (need_ack)
			pcat_pm_uart_write_data(pm_data, command+1,
				NULL, 0, false, 0);
		
		i += 9 + expect_len;
    		used_size = i + 1;
	}
	
	if (used_size > 0) {
        	memmove(buffer, buffer + used_size,
        		pm_data->read_buffer_used - used_size);
        	pm_data->read_buffer_used -= used_size;
	}

	return used_size;
}

static size_t pcat_pm_uart_serdev_receive_buf(
	struct serdev_device *serdev, const u8 *buf, size_t count)
{
	struct pcat_pm_data *pm_data = serdev_device_get_drvdata(serdev);
	size_t used_size = 0;

	while (used_size < count) {
		if (pm_data->read_buffer_used + count - used_size >
			PCAT_PM_READ_BUFFER_SIZE) {
			memcpy(pm_data->read_buffer + pm_data->read_buffer_used,
				buf + used_size,
				PCAT_PM_READ_BUFFER_SIZE - pm_data->read_buffer_used);
			pm_data->read_buffer_used = PCAT_PM_READ_BUFFER_SIZE;
			used_size += PCAT_PM_READ_BUFFER_SIZE - pm_data->read_buffer_used;
		} else {
			memcpy(pm_data->read_buffer + pm_data->read_buffer_used,
				buf + used_size, count - used_size);
			pm_data->read_buffer_used += count - used_size;
			used_size += count - used_size;
		}

		pcat_pm_uart_receive_parse(pm_data);
	}

	return count;
}

static const struct serdev_device_ops pcat_pm_serdev_ops = {
	.receive_buf = pcat_pm_uart_serdev_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int pcat_pm_uart_serdev_open(struct pcat_pm_data *pm_data)
{
	struct serdev_device *serdev = pm_data->serdev;
	struct device *dev = &serdev->dev;
	int ret;
	
	ret = devm_serdev_device_open(dev, serdev);
	if (ret < 0)
		return ret;
		
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret < 0) {
		dev_err(dev, "set parity failed\n");
		return ret;
	}

	serdev_device_set_baudrate(serdev, pm_data->baudrate);
	serdev_device_set_flow_control(serdev, false);

	return 0;
}

static void pcat_pm_check_work(struct kthread_work *work)
{
        struct pcat_pm_data *pm_data;

        pm_data = container_of(work, struct pcat_pm_data, check_work);
        
        if (pm_data->work_flag)
	        pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_HEARTBEAT,
	        	NULL, 0, false, 0);
}

static enum hrtimer_restart pcat_pm_check_timer_expired(struct hrtimer *timer)
{
        struct pcat_pm_data *pm_data;

        pm_data = container_of(timer, struct pcat_pm_data, check_timer);
        
	kthread_queue_work(pm_data->kworker, &pm_data->check_work);
	
	hrtimer_forward_now(timer, ms_to_ktime(1000));

	return HRTIMER_RESTART;
}

static int pcat_pm_charger_probe(struct pcat_pm_data *pm_data)
{
	struct serdev_device *serdev = pm_data->serdev;
	struct device *dev = &serdev->dev;
	const struct power_supply_desc *battery_psy_desc;
	struct power_supply_config pscfg = {};
	struct device_node *charger_node;
	struct power_supply_battery_info *bat_info;
	int ret;

	if (device_property_read_u32(dev, "pm-version",
				&pm_data->pm_version)) {
		pm_data->pm_version = 1;
	}
	
	if (pm_data->pm_version > 1)
		battery_psy_desc = &pcat_pm_battery_v2_desc;
	else
		battery_psy_desc = &pcat_pm_battery_v1_desc;

	charger_node = of_get_child_by_name(dev->of_node, "charger");
	if (!charger_node) {
		dev_err(dev, "Missing charger node!\n");
		return -ENODEV;
	}
	
	pscfg.drv_data = pm_data;
	pscfg.of_node = charger_node;

	pm_data->battery_psy = power_supply_register(dev, battery_psy_desc, &pscfg);
	if (IS_ERR(pm_data->battery_psy)) {
		ret = PTR_ERR(pm_data->battery_psy);
		dev_err(dev, "Failed to register battery power supply: %d\n", ret);
		
		return -EINVAL;
	}
	
	pm_data->charger_psy = power_supply_register(dev, &pcat_pm_charger_desc, &pscfg);
	if (IS_ERR(pm_data->charger_psy)) {
		ret = PTR_ERR(pm_data->charger_psy);
		dev_err(dev, "Failed to register battery power supply: %d\n", ret);
		
		return -EINVAL;
	}
	
	ret = power_supply_get_battery_info(pm_data->battery_psy, &bat_info);
	if (ret) {
		dev_err(dev, "Failed to get battery info: %d\n", ret);

		return ret;
	}
	
	pm_data->battery_technology = bat_info->technology;
	pm_data->battery_design_uwh = bat_info->energy_full_design_uwh;
	pm_data->battery_design_min_uv = bat_info->voltage_min_design_uv;
	pm_data->battery_design_max_uv = bat_info->voltage_max_design_uv;
	pm_data->battery_voltage_now = pm_data->battery_design_max_uv;
	pm_data->battery_soc = 100;
	pm_data->battery_info = bat_info;
	pm_data->on_charger = true;

	return 0;
}

static int pcat_pm_rtc_read_time(struct device *dev, struct rtc_time *t)
{
	struct serdev_device *serdev;
	struct pcat_pm_data *pm_data;
	
	serdev = container_of(dev, struct serdev_device, dev);
	pm_data = serdev_device_get_drvdata(serdev);

	mutex_lock(&pm_data->charger_mutex);
	t->tm_year = pm_data->rtc_year - 1900;
	t->tm_mon = pm_data->rtc_month;
	t->tm_mday = pm_data->rtc_day;
	t->tm_hour = pm_data->rtc_hour;
	t->tm_min = pm_data->rtc_min;
	t->tm_sec = pm_data->rtc_sec;
	mutex_unlock(&pm_data->charger_mutex);

	return 0;
}

static int pcat_pm_rtc_set_time(struct device *dev, struct rtc_time *t)
{
	struct serdev_device *serdev;
	struct pcat_pm_data *pm_data;
	u8 date_data[7];
	u16 y;
	
	serdev = container_of(dev, struct serdev_device, dev);
	pm_data = serdev_device_get_drvdata(serdev);

	y = t->tm_year + 1900;
	date_data[0] = y & 0xFF;
	date_data[1] = (y >> 8) & 0xFF;
	date_data[2] = t->tm_mon;
	date_data[3] = t->tm_mday;
	date_data[4] = t->tm_hour;
	date_data[5] = t->tm_min;
	date_data[6] = t->tm_sec;
	
	return pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_DATE_TIME_SYNC,
		date_data, 7, true, 0);
}

static int pcat_pm_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct serdev_device *serdev;
	struct pcat_pm_data *pm_data;
	int status;
	int flags = 0;
	
	serdev = container_of(dev, struct serdev_device, dev);
	pm_data = serdev_device_get_drvdata(serdev);

	switch (cmd) {
	case RTC_VL_READ:
		mutex_lock(&pm_data->charger_mutex);
		status = pm_data->rtc_status;
		mutex_unlock(&pm_data->charger_mutex);

		if (status)
			flags |= RTC_VL_DATA_INVALID;

		return put_user(flags, (unsigned int __user *)arg);

	default:
		return -ENOIOCTLCMD;  
	}
}

static const struct rtc_class_ops pcat_pm_rtcops = {
	.read_time	= pcat_pm_rtc_read_time,
	.set_time	= pcat_pm_rtc_set_time,
	.ioctl		= pcat_pm_rtc_ioctl,
};

static int pcat_pm_rtc_probe(struct pcat_pm_data *pm_data)
{
	int ret;

	pm_data->rtc = devm_rtc_allocate_device(&pm_data->serdev->dev);
	if (IS_ERR(pm_data->rtc)) {
		ret = PTR_ERR(pm_data->rtc);
		dev_err(&pm_data->serdev->dev, "Cannot allocate RTC device: %d\n", ret);
		return ret;
	}
		
	pm_data->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	pm_data->rtc->range_max = RTC_TIMESTAMP_END_2099;
	pm_data->rtc->ops = &pcat_pm_rtcops;

	ret = devm_rtc_register_device(pm_data->rtc);
	if (ret) {
		dev_err(&pm_data->serdev->dev, "Failed to register RTC device: %d\n", ret);
		return ret;
	}

	return 0;
}

static umode_t pcat_pm_hwmon_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static int pcat_pm_hwmon_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *temp)
{
	struct serdev_device *serdev;
	struct pcat_pm_data *pm_data;
	int err = 0;
	
	serdev = container_of(dev, struct serdev_device, dev);
	pm_data = serdev_device_get_drvdata(serdev);


	switch (attr) {
	case hwmon_temp_input:
		mutex_lock(&pm_data->charger_mutex);
		*temp = pm_data->board_temp * 1000;
		mutex_unlock(&pm_data->charger_mutex);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static u32 pcat_pm_hwmon_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info pcat_pm_hwmon_temp = {
	.type = hwmon_temp,
	.config = pcat_pm_hwmon_temp_config,
};

static const struct hwmon_channel_info *pcat_pm_hwmon_info[] = {
	&pcat_pm_hwmon_temp,
	NULL
};

static const struct hwmon_ops pcat_pm_hwmon_ops = {
	.is_visible = pcat_pm_hwmon_is_visible,
	.read = pcat_pm_hwmon_read,
};

static const struct hwmon_chip_info pcat_pm_hwmon_chip_info = {
	.ops = &pcat_pm_hwmon_ops,
	.info = pcat_pm_hwmon_info,
};

static int pcat_pm_hwmon_probe(struct pcat_pm_data *pm_data)
{
	int ret = 0;

	pm_data->hwmon_dev = devm_hwmon_device_register_with_info(
		&pm_data->serdev->dev, "pcat-pm-hwmon", pm_data,
		&pcat_pm_hwmon_chip_info, NULL);
		
	if (IS_ERR(pm_data->hwmon_dev)) {
		ret = PTR_ERR(pm_data->hwmon_dev);
		dev_err(&pm_data->serdev->dev, "Failed to register hwmon: %d\n", ret);
	}
		
	return ret;
}

static ssize_t pcat_pm_ctl_dev_read(struct file *file, char *buffer,
	size_t count, loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct pcat_pm_data *pm_data;
	
	pm_data = container_of(mdev, struct pcat_pm_data, ctl_device);

	return 0;
}

static ssize_t pcat_pm_ctl_dev_write(struct file *file, const char *buffer,
	size_t count, loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct pcat_pm_data *pm_data;
	
	pm_data = container_of(mdev, struct pcat_pm_data, ctl_device);

	return 0;
}

static struct file_operations pcat_pm_ctl_dev_ops = {
	.owner = THIS_MODULE,
	.read = pcat_pm_ctl_dev_read,
	.write = pcat_pm_ctl_dev_write,
};

static int pcat_pm_probe(struct serdev_device *serdev)
{
	struct pcat_pm_data *pm_data;
	int ret;

	pm_data = devm_kzalloc(&serdev->dev, sizeof(*pm_data), GFP_KERNEL);
	if (!pm_data)
		return -ENOMEM;

	pm_data->serdev = serdev;
	pm_data->work_flag = true;
	
	mutex_init(&pm_data->charger_mutex);

	if (device_property_read_u32(&serdev->dev, "baudrate",
				    &pm_data->baudrate)) {
		pm_data->baudrate = 115200;
	}
	
	if (device_property_read_u32(&serdev->dev, "force-poweroff-timeout",
				    &pm_data->force_poweroff_timeout)) {
		pm_data->force_poweroff_timeout = 0;
	}

	serdev_device_set_drvdata(serdev, pm_data);
	serdev_device_set_client_ops(serdev, &pcat_pm_serdev_ops);
	
	pm_data->power_gpio = devm_gpiod_get(&serdev->dev,
		"power", GPIOD_OUT_HIGH);
	if (IS_ERR(pm_data->power_gpio)) {
		dev_err(&serdev->dev, "Failed to setup power GPIO!\n");
	}

        ret = devm_register_sys_off_handler(&serdev->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
		SYS_OFF_PRIO_FIRMWARE, pcat_pm_do_poweroff, pm_data);
        if (ret)
                dev_err(&serdev->dev, "Cannot register poweroff handler: %d\n", ret);

        ret = devm_register_sys_off_handler(&serdev->dev, SYS_OFF_MODE_RESTART,
		SYS_OFF_PRIO_HIGH, pcat_pm_do_restart, pm_data);
        if (ret)
                dev_err(&serdev->dev, "Cannot register poweroff handler: %d\n", ret);

	do {
		ret = pcat_pm_uart_serdev_open(pm_data);
		if (ret) {
			dev_err(&serdev->dev, "Cannot open serial device: %d\n", ret);
			break;
		}
		
		pm_data->kworker = kthread_create_worker(0, "pcat-pm-kworker");
		if (IS_ERR(pm_data->kworker)) {
			dev_err(&serdev->dev, "Failed to create kworker!\n");
			break;
		}
		sched_set_fifo(pm_data->kworker->task);
		
		kthread_init_work(&pm_data->check_work, pcat_pm_check_work);
		hrtimer_init(&pm_data->check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
		pm_data->check_timer.function = pcat_pm_check_timer_expired;
		
		hrtimer_start(&pm_data->check_timer, ms_to_ktime(1000), HRTIMER_MODE_REL_HARD);
		
		pcat_pm_watchdog_timeout_set(pm_data, PCAT_PM_WATCHDOG_DEFAULT_INTERVAL, 0);
		pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_PMU_FW_VERSION_GET,
			NULL, 0, true, 0);
		
	} while (0);
	
	ret = pcat_pm_charger_probe(pm_data);

	ret = pcat_pm_rtc_probe(pm_data);
	
	ret = pcat_pm_hwmon_probe(pm_data);

	pm_data->ctl_device.minor = MISC_DYNAMIC_MINOR;
	pm_data->ctl_device.name = "pcat-pm-ctl";
	pm_data->ctl_device.fops = &pcat_pm_ctl_dev_ops;
	pm_data->ctl_device.mode = 0;
	ret = misc_register(&pm_data->ctl_device);
	if (ret)
		dev_err(&serdev->dev, "Failed to register control device: %d\n", ret);
	
	dev_info(&serdev->dev, "photonicat power manager initialized OK.\n");

	return 0;
}

static struct of_device_id pcat_pm_of_match[] = {
	{ .compatible = "photonicat-pm" },
	{}
};
MODULE_DEVICE_TABLE(of, pcat_pm_of_match);

static struct serdev_device_driver pcat_pm_driver = {
	.driver = {
		.name = "photonicat-pm",
		.owner = THIS_MODULE,
	        .of_match_table = of_match_ptr(pcat_pm_of_match),
	},
	.probe = pcat_pm_probe,
};

module_serdev_device_driver(pcat_pm_driver);

MODULE_DESCRIPTION("photonicat power manager driver");
MODULE_AUTHOR("Kyosuke Nekoyashiki <supercatexpert@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:photonicat-pm");
