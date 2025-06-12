// SPDX-License-Identifier: GPL-2.0+
/*
 * Copied from simple-panel
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 * Copyright (c) 2018 Sjoerd Simons <sjoerd.simons@collabora.co.uk>
 * Modified by Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 *
 * Panel Initialization for photonicat2 SPI LCD display
 * Resolution: 170x320
 * Color-Mode: RGB
 *
 */

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <spi.h>
#include <asm/gpio.h>

#include "pcat-spi-display-logo.h"

DECLARE_GLOBAL_DATA_PTR;

struct pcat_spi_display_priv {
	struct spi_slave *spi;
	struct udevice *backlight;
	struct gpio_desc reset_gpio;
	struct gpio_desc ctrl_gpio;
	int ctrl_state;
};

static int pcat_spi_display_enable_backlight(struct udevice *dev)
{
	struct pcat_spi_display_priv *priv = dev_get_priv(dev);
	int ret;

	debug("%s: start, backlight = '%s'\n", __func__, priv->backlight->name);
	ret = backlight_enable(priv->backlight);
	debug("%s: done, ret = %d\n", __func__, ret);
	if (ret)
		return ret;

	return 0;
}

static int pcat_spi_display_ofdata_to_platdata(struct udevice *dev)
{
	struct pcat_spi_display_priv *priv = dev_get_priv(dev);
	int ret;

	priv->spi = dev_get_parent_priv(dev);

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
		"backlight", &priv->backlight);
	if (ret) {
		printf("%s: Cannot get backlight: ret=%d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset_gpio,
		GPIOD_IS_OUT);
	if (ret) {
		printf("%s: Warning: cannot get reset GPIO: ret=%d\n",
			__func__, ret);
		if (ret != -ENOENT)
			return ret;
	}

	ret = gpio_request_by_name(dev, "ctrl-gpios", 0, &priv->ctrl_gpio,
		GPIOD_IS_OUT);
	if (ret) {
		printf("%s: Warning: cannot get control GPIO: ret=%d\n",
			__func__, ret);
		if (ret != -ENOENT)
			return ret;
	}

	return 0;
}

static void pcat_spi_display_send_command(struct pcat_spi_display_priv *priv,
	u8 cmd)
{
	int ret;

	if (priv->ctrl_state != 0) {
		dm_gpio_set_value(&priv->ctrl_gpio, 0);
		priv->ctrl_state = 0;
	}

	ret = spi_xfer(priv->spi, 8, &cmd, NULL, SPI_XFER_ONCE);
	if (ret) {
		printf("Failed to send command to SPI: %d\n", ret);
	}
}

static void pcat_spi_display_send_data(struct pcat_spi_display_priv *priv,
	const u8 *data, size_t len)
{
	size_t off;
	int ret = 0;

	if (priv->ctrl_state != 1) {
		dm_gpio_set_value(&priv->ctrl_gpio, 1);
		priv->ctrl_state = 1;
	}

	for (off = 0; off < len; off++) {
		ret = spi_xfer(priv->spi, 8, data + off, NULL, SPI_XFER_ONCE);
		if (ret)
			break;
	}

	if (ret)
		printf("Failed to send data to SPI: %d\n", ret);
}

static void pcat_spi_display_show_logo(struct udevice *dev)
{
	struct pcat_spi_display_priv *priv = dev_get_priv(dev);
	u8 v8;
	const u8 col[] = {0x00, 0x22, 0x00, 0xCD};
	const u8 row[] = {0x00, 0x00, 0x01, 0x3F};
	unsigned int x, y;
	u8 pix_buf[2];
	u16 pix;

	dm_gpio_set_value(&priv->reset_gpio, 1);
	dm_gpio_set_value(&priv->ctrl_gpio, 1);

	mdelay(10);
	dm_gpio_set_value(&priv->reset_gpio, 0);
	mdelay(10);
	dm_gpio_set_value(&priv->reset_gpio, 1);
	mdelay(120);

	pcat_spi_display_send_command(priv, 0x01);
	mdelay(150);
	pcat_spi_display_send_command(priv, 0x11);
	mdelay(150);

	pcat_spi_display_send_command(priv, 0x3A);
	v8 = 0x55;
	pcat_spi_display_send_data(priv, &v8, 1);

	pcat_spi_display_send_command(priv, 0x36);
	v8 = 0x40;
	pcat_spi_display_send_data(priv, &v8, 1);

	pcat_spi_display_send_command(priv, 0x29);
	mdelay(50);

	pcat_spi_display_send_command(priv, 0x2A);
	pcat_spi_display_send_data(priv, col, sizeof(col));

	pcat_spi_display_send_command(priv, 0x2B);
	pcat_spi_display_send_data(priv, row, sizeof(row));

	pcat_spi_display_send_command(priv, 0x2C);

	for (y = 0; y < image_height; y++) {
		for (x = 0; x < image_width; x++) {
			pix = image_data[y][x];
			pix_buf[0] = pix >> 8;
			pix_buf[1] = pix & 0xFF;
			pcat_spi_display_send_data(priv, pix_buf, 2);
		}
	}

}

static int pcat_spi_display_probe(struct udevice *dev)
{
	struct pcat_spi_display_priv *priv = dev_get_priv(dev);
	int ret;

	ret = spi_claim_bus(priv->spi);
	if (ret) {
		printf("Failed to claim bus: %d\n", ret);
		return ret;
	}
	priv->ctrl_state = -1;

	pcat_spi_display_show_logo(dev);

	ret = pcat_spi_display_enable_backlight(dev);
	if (ret)
		printf("Failed to enable backlight: %d\n", ret);

	printf("photonicat SPI LCD display probed.\n");

	return ret;
}

static const struct udevice_id pcat_spi_display_ids[] = {
	{ .compatible = "ariaboard,pcat-spi-display" },
	{ }
};

U_BOOT_DRIVER(pcat_spi_display) = {
	.name = "pcat-spi-display",
	.id = UCLASS_PANEL,
	.of_match = pcat_spi_display_ids,
	.ofdata_to_platdata = pcat_spi_display_ofdata_to_platdata,
	.probe = pcat_spi_display_probe,
	.priv_auto_alloc_size = sizeof(struct pcat_spi_display_priv),
};

