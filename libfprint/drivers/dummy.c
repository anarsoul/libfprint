/*
 * Dummy imaging driver for libfprint
 * Copyright (C) 2012 Vasily Khoruzhick
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "dummy"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libusb.h>

#include <fp_internal.h>

#include "driver_ids.h"

#define DUMMY_DEVICE_WIDTH 192

struct dummy_dev {
	unsigned int img_height;
	char *filename;
};

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	struct dummy_dev *dummydev;
	char *env_val;

	dev->priv = dummydev = g_malloc0(sizeof(struct dummy_dev));
	if (!dummydev)
		return -ENOMEM;

	env_val = getenv("FPRINT_DUMMY_HEIGHT");
	if (!env_val) {
		fp_err("FPRINT_DUMMY_HEIGHT is not defined!");
		g_free(dummydev);
		return -ENODEV;
	}
	errno = 0;
	dummydev->img_height = strtol(env_val, NULL, 10);
	if (errno) {
		fp_err("FPRINT_DUMMY_HEIGHT is invalid!");
		g_free(dummydev);
		return -ENODEV;
	}

	env_val = getenv("FPRINT_DUMMY_FILENAME");
	if (!env_val) {
		fp_err("FPRINT_DUMMY_FILENAME is not defined!");
		g_free(dummydev);
		return -ENODEV;
	}
	dummydev->filename = strdup(env_val);

	fpi_imgdev_open_complete(dev, 0);

	return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	struct dummy_dev *dummydev = dev->priv;
	g_free(dummydev->filename);
	g_free(dummydev);
	fpi_imgdev_close_complete(dev);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct dummy_dev *dummydev = dev->priv;
	struct fp_img *img;
	FILE *in;

	fpi_imgdev_activate_complete(dev, 0);

	img = fpi_img_new(DUMMY_DEVICE_WIDTH * dummydev->img_height);
	img->width = DUMMY_DEVICE_WIDTH;
	img->height = dummydev->img_height;

	in = fopen(dummydev->filename, "rb");
	if (in) {
		fp_dbg("Loading data from %s\n", dummydev->filename);
		fread(img->data, img->width * img->height, 1, in);
		fclose(in);
	}

	fpi_imgdev_report_finger_status(dev, TRUE);
	fpi_imgdev_image_captured(dev, img);
	fpi_imgdev_report_finger_status(dev, FALSE);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
}

struct fp_img_driver dummy_driver = {
	.driver = {
		.id = DUMMY_ID,
		.name = FP_COMPONENT,
		.full_name = "Dummy swipe device",
		.id_table = NULL,
		.scan_type = FP_SCAN_TYPE_SWIPE,
	},
	.flags = 0,
	.img_height = -1,
	.img_width = DUMMY_DEVICE_WIDTH,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};
