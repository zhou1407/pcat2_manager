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

#define PCAT_PM_READ_BUFFER_SIZE 4096

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

struct pcat_pm_data {
	struct serdev_device *serdev;
	struct completion rx_byte_received;
	struct gpio_desc *power_gpio;
	struct kthread_worker *kworker;
	struct kthread_work check_work;
	struct hrtimer check_timer;
	struct task_struct *tsk;
	bool work_flag;
	bool poweroff_ok;
	u32 baudrate;
	u16 write_framenum;
	
	u8 read_buffer[PCAT_PM_READ_BUFFER_SIZE];
	size_t read_buffer_used;
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
	u8 timeouts[3] = {60, 60, interval};

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
	const u8 *extra_data, size_t extra_data_len)
{

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
		if (buffer[i]==0xA5) {
			p = buffer + i;
			remaining_size = pm_data->read_buffer_used - i;
			used_size = i;

			if (remaining_size < 13)
				break;

			expect_len = p[5] + ((u16)p[6] << 8);
			if (expect_len < 3 || expect_len > 512) {
				used_size = i;
				continue;
			}

			if (expect_len + 10 > remaining_size)
				break;
			if (p[9+expect_len]!=0x5A) {
				used_size = i;
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
			
			dev_info(&pm_data->serdev->dev,
				"Got command %X from %X to %X, frame num %d, "
				"need ACK %d.\n", command, src, dst, frame_num, need_ack);
			
			if (dst==0x1 || dst==0x80 || dst==0xFF) {
				switch (command) {
					case PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK: {
						pm_data->poweroff_ok = true;
						need_ack = false;
						break;
					}
					case PCAT_PM_COMMAND_STATUS_REPORT: {
						pcat_pm_status_report_parse(
							pm_data, extra_data, extra_data_len);
						break;
					}
					default: {
						break;
					}
				}
			}
			
			if (need_ack)
				pcat_pm_uart_write_data(pm_data, command+1,
					NULL, 0, false, 0);
			
			i += 9 + expect_len;
            		used_size = i + 1;
		}
		else {
			used_size = i;
		}
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

static int pcat_pm_probe(struct serdev_device *serdev)
{
	struct pcat_pm_data *pm_data;
	int ret;

	pm_data = devm_kzalloc(&serdev->dev, sizeof(*pm_data), GFP_KERNEL);
	if (!pm_data)
		return -ENOMEM;

	pm_data->serdev = serdev;
	pm_data->work_flag = true;
	init_completion(&pm_data->rx_byte_received);

	if (device_property_read_u32(&serdev->dev, "baudrate",
				    &pm_data->baudrate)) {
		pm_data->baudrate = 115200;
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
		
		pcat_pm_watchdog_timeout_set(pm_data, 10, 0);
		pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_PMU_FW_VERSION_GET,
			NULL, 0, true, 0);
		
	} while(0);
	
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
