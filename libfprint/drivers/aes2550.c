/*
 * AuthenTec AES2550/AES2810 driver for libfprint
 * Copyright (C) 2007-2008 Daniel Drake <dsd@gentoo.org>
 * Copyright (C) 2007 Cyrille Bagard
 * Copyright (C) 2007-2012 Vasily Khoruzhick
 *
 * Based on AES2501 driver
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

#define FP_COMPONENT "aes2550"

#include <errno.h>
#include <string.h>

#include <libusb.h>

#include <aeslib.h>
#include <fp_internal.h>

static void start_capture(struct fp_img_dev *dev);
static void complete_deactivation(struct fp_img_dev *dev);

#define EP_IN			(1 | LIBUSB_ENDPOINT_IN)
#define EP_OUT			(2 | LIBUSB_ENDPOINT_OUT)
#define BULK_TIMEOUT 4000

/*
 * The AES2550 is an imaging device using a swipe-type sensor. It samples
 * the finger at preprogrammed intervals, sending a 192x16 frame to the
 * computer.
 * Unless the user is scanning their finger unreasonably fast, the frames
 * *will* overlap. The implementation below detects this overlap and produces
 * a contiguous image as the end result.
 * The fact that the user determines the length of the swipe (and hence the
 * number of useful frames) and also the fact that overlap varies means that
 * images returned from this driver vary in height.
 */

#define FRAME_WIDTH		192
#define FRAME_HEIGHT		8
#define FRAME_SIZE		(FRAME_WIDTH * FRAME_HEIGHT)
/* maximum number of frames to read during a scan */
#define MAX_FRAMES		150

/* E-data msg */
#define STRIP_SIZE		(0x31e + 3)

struct aes2550_dev {
	unsigned char data_buf[STRIP_SIZE];
	int data_len;
	GSList *strips;
	size_t strips_len;
	gboolean deactivating;
};

/****** IMAGE PROCESSING ******/

/* find overlapping parts of  frames */
static unsigned int find_overlap(unsigned char *first_frame,
	unsigned char *second_frame, unsigned int *min_error)
{
	unsigned int dy;
	unsigned int not_overlapped_height = 0;
	*min_error = 255 * FRAME_SIZE;
	for (dy = 0; dy < FRAME_HEIGHT; dy++) {
		/* Calculating difference (error) between parts of frames */
		unsigned int i;
		unsigned int error = 0;
		for (i = 0; i < FRAME_WIDTH * (FRAME_HEIGHT - dy); i++) {
			/* Using ? operator to avoid abs function */
			error += first_frame[i] > second_frame[i] ?
					(first_frame[i] - second_frame[i]) :
					(second_frame[i] - first_frame[i]);
		}

		/* Normalize error */
		error *= 15;
		error /= i;
		if (error < *min_error) {
			*min_error = error;
			not_overlapped_height = dy;
		}
		first_frame += FRAME_WIDTH;
	}

	return not_overlapped_height;
}

/* assemble a series of frames into a single image */
static unsigned int assemble(struct aes2550_dev *aesdev, unsigned char *output,
	gboolean reverse, unsigned int *errors_sum)
{
	uint8_t *assembled = output;
	int frame;
	uint32_t image_height = FRAME_HEIGHT;
	unsigned int min_error;
	size_t num_strips = aesdev->strips_len;
	GSList *list_entry = aesdev->strips;
	*errors_sum = 0;

	if (reverse)
		output += (num_strips - 1) * FRAME_SIZE;
	for (frame = 0; frame < num_strips; frame++) {
		aes_assemble_image(list_entry->data, FRAME_WIDTH, FRAME_HEIGHT, output);

		if (reverse)
		    output -= FRAME_SIZE;
		else
		    output += FRAME_SIZE;
		list_entry = g_slist_next(list_entry);
	}

	/* Detecting where frames overlaped */
	output = assembled;
	for (frame = 1; frame < num_strips; frame++) {
		int not_overlapped;

		output += FRAME_SIZE;
		not_overlapped = find_overlap(assembled, output, &min_error);
		*errors_sum += min_error;
		image_height += not_overlapped;
		assembled += FRAME_WIDTH * not_overlapped;
		memcpy(assembled, output, FRAME_SIZE); 
	}
	return image_height;
}

static void assemble_and_submit_image(struct fp_img_dev *dev)
{
	struct aes2550_dev *aesdev = dev->priv;
	size_t final_size;
	struct fp_img *img;
	unsigned int errors_sum, r_errors_sum;

	BUG_ON(aesdev->strips_len == 0);

	/* reverse list */
	aesdev->strips = g_slist_reverse(aesdev->strips);

	/* create buffer big enough for max image */
	img = fpi_img_new(aesdev->strips_len * FRAME_SIZE);

	img->flags = FP_IMG_COLORS_INVERTED;
	img->height = assemble(aesdev, img->data, FALSE, &errors_sum);
	img->height = assemble(aesdev, img->data, TRUE, &r_errors_sum);

	if (r_errors_sum > errors_sum) {
	    img->height = assemble(aesdev, img->data, FALSE, &errors_sum);
		img->flags |= FP_IMG_V_FLIPPED | FP_IMG_H_FLIPPED;
		fp_dbg("normal scan direction");
	} else {
		fp_dbg("reversed scan direction");
	}

	/* now that overlap has been removed, resize output image buffer */
	final_size = img->height * FRAME_WIDTH;
	img = fpi_img_resize(img, final_size);
	fpi_imgdev_image_captured(dev, img);

	/* free strips and strip list */
	g_slist_foreach(aesdev->strips, (GFunc) g_free, NULL);
	g_slist_free(aesdev->strips);
	aesdev->strips = NULL;
}


/****** FINGER PRESENCE DETECTION ******/

/* FIXME: remove magic constants */
static unsigned char finger_det_reqs[] = {
	0x80, 0x01,
	0x95, 0x18,
	0xa1, 0x00,
	0x8a, 0x07,
	0xad, 0x00,
	0xbd, 0x00,
	0xbe, 0x00,
	0xcf, 0x01,
	0xdd, 0x00,
	0xac, 0x01, /* Errata */
	0x70, 0x00, 0x01, 0x00, /* Heart beat off */
	0x01,
};

static void start_finger_detection(struct fp_img_dev *dev);

static void finger_det_data_cb(struct libusb_transfer *transfer)
{
	struct fp_img_dev *dev = transfer->user_data;
	unsigned char *data = transfer->buffer;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fp_dbg("data transfer status %d\n", transfer->status);
		fpi_imgdev_session_error(dev, -EIO);
		goto out;
	} else if (transfer->length != transfer->actual_length) {
		fp_dbg("expected %d, got %d bytes", transfer->length, transfer->actual_length);
		fpi_imgdev_session_error(dev, -EPROTO);
		goto out;
	}

	/* FIXME: remove magic constants */
	if (data[1] & 0x80) {
		/* finger present, start capturing */
		fpi_imgdev_report_finger_status(dev, TRUE);
		start_capture(dev);
	} else {
		/* no finger, poll for a new histogram */
		start_finger_detection(dev);
	}
out:
	g_free(data);
	libusb_free_transfer(transfer);
}

static void finger_det_reqs_cb(struct libusb_transfer *t)
{
	struct libusb_transfer *transfer;
	unsigned char *data;
	int r;
	struct fp_img_dev *dev = t->user_data;

	if (t->status != LIBUSB_TRANSFER_COMPLETED) {
		fp_dbg("req transfer status %d\n", t->status);
		fpi_imgdev_session_error(dev, -EIO);
		goto exit_free_transfer;
	} else if (t->length != t->actual_length) {
		fp_dbg("expected %d, got %d bytes", t->length, t->actual_length);
		fpi_imgdev_session_error(dev, -EPROTO);
		goto exit_free_transfer;
	}

	transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		fpi_imgdev_session_error(dev, -ENOMEM);
		goto exit_free_transfer;
	}

	/* 2 bytes of result */
	data = g_malloc(2);
	libusb_fill_bulk_transfer(transfer, dev->udev, EP_IN, data, 2,
		finger_det_data_cb, dev, BULK_TIMEOUT);

	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		g_free(data);
		libusb_free_transfer(transfer);
		fpi_imgdev_session_error(dev, r);
	}
exit_free_transfer:
	libusb_free_transfer(t);
}

static void start_finger_detection(struct fp_img_dev *dev)
{
	int r;
	struct aes2550_dev *aesdev = dev->priv;
	struct libusb_transfer *transfer;
	fp_dbg("");

	if (aesdev->deactivating) {
		complete_deactivation(dev);
		return;
	}

	transfer = libusb_alloc_transfer(0);
	if (!transfer) {
		fpi_imgdev_session_error(dev, -ENOMEM);
		return;
	}
	libusb_fill_bulk_transfer(transfer, dev->udev, EP_OUT, finger_det_reqs,
		sizeof(finger_det_reqs), finger_det_reqs_cb, dev, BULK_TIMEOUT);
	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		libusb_free_transfer(transfer);
		fpi_imgdev_session_error(dev, r);
	}
}

/****** CAPTURE ******/

/* FIXME: remove magic constants */
static unsigned char capture_reqs[] = {
	0x80, 0x01,
	0x80, 0x18,
	0x85, 0x00,
	0x8f, 0x0c,
	0x9c, 0xbf,
	0x9d, 0x46,
	0x9e, 0x71,
	0x9f, 0x23,
	0xa2, 0x00,
	0xb1, 0x00,
	0xbf, 0x0b,
	0xcf, 0x32,
	0xdc, 0x01,
	0xdd, 0x00,
	0xac, 0x01, /* Errata */
	0x70, 0x00, 0x01, 0x03, /* Heart beat cmd, 3 * 16 cycles without sending image */
	0x02,
};

static unsigned char capture_set_idle_reqs[] = {
	0x80, 0x01,
	0x70, 0x00, 0x01, 0x00, /* Heart beat off */
	0x00,
};

enum capture_states {
	CAPTURE_WRITE_REQS,
	CAPTURE_READ_DATA,
	CAPTURE_SET_IDLE,
	CAPTURE_NUM_STATES,
};

/* Returns number of processed bytes */
static int process_strip_data(struct fpi_ssm *ssm, unsigned char *data, int buf_size, int *last_found)
{
	unsigned char *stripdata;
	struct fp_img_dev *dev = ssm->priv;
	struct aes2550_dev *aesdev = dev->priv;
	int len, total_len = 0;

	/* FIXME: remove magic constants */
	switch (data[0]) {
	case 0xe0:
		break;
	case 0xdb:
		/* Finger was removed */
		/* assemble image and submit it to library */
		fp_dbg("Got heartbeat => last frame");
		*last_found = 1;
		fpi_ssm_next_state(ssm);
		goto out;
		break;
	default:
		fp_dbg("Invalid magic: %.2x\n", (int)data[0]);
		return -EINVAL;
		break;
	}

	if (buf_size < STRIP_SIZE)
		return 0;

	len = data[1] * 256 + data[2];
	stripdata = g_malloc(FRAME_WIDTH * FRAME_HEIGHT / 2); /* 4 bits per pixel */
	if (!stripdata) {
		fpi_ssm_mark_aborted(ssm, -ENOMEM);
		return -ENOMEM;
	}
	memcpy(stripdata, data + 33, FRAME_WIDTH * FRAME_HEIGHT / 2);
	aesdev->strips = g_slist_prepend(aesdev->strips, stripdata);
	aesdev->strips_len++;

	total_len += (len + 3);
	data += (len + 3);
out:
	return total_len;
}

static void capture_reqs_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;

	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) &&
		(transfer->length == transfer->actual_length)) {
		fpi_ssm_next_state(ssm);
	} else {
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
	libusb_free_transfer(transfer);
}

static void capture_set_idle_reqs_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;
	struct fp_img_dev *dev = ssm->priv;

	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) &&
		(transfer->length == transfer->actual_length)) {
		assemble_and_submit_image(dev);
		fpi_imgdev_report_finger_status(dev, FALSE);
		/* marking machine complete will re-trigger finger detection loop */
		fpi_ssm_mark_completed(ssm);
	} else {
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
	libusb_free_transfer(transfer);
}

static void capture_read_data_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;
	struct fp_img_dev *dev = ssm->priv;
	struct aes2550_dev *aesdev = dev->priv;
	unsigned char *data = NULL;
	int len = transfer->actual_length, processed = 0, last_found = 0;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fp_dbg("status not completed, %d", transfer->status);
		fpi_ssm_mark_aborted(ssm, -EIO);
		goto out;
	}
	if (transfer->actual_length != transfer->length) {
		fp_dbg("Short frame, appear to be a last one?");
	}

	data = transfer->buffer;

	if (aesdev->data_len) {
		processed = MIN(STRIP_SIZE - aesdev->data_len, transfer->actual_length);
		memcpy(aesdev->data_buf + aesdev->data_len, data, processed);
		aesdev->data_len += processed;
		len -= processed;
		data += processed;
	}

	if (aesdev->data_len == STRIP_SIZE) {
		fp_dbg("Processing rest from last transfer");
		process_strip_data(ssm, data, aesdev->data_len, &last_found);
		aesdev->data_len = 0;
	}

	while (len && !last_found) {
		processed = process_strip_data(ssm, data, len, &last_found);
		fp_dbg("Processed frame, result %d\n", processed);
		if (!processed)
			break;
		if (processed < 0) {
			fpi_ssm_mark_aborted(ssm, processed);
			goto out;
		}
		data += processed;
		len -= processed;
	}

	if (!last_found) {
		fpi_ssm_jump_to_state(ssm, CAPTURE_READ_DATA);
		if (len) {
			memcpy(aesdev->data_buf, data, len);
			aesdev->data_len = len;
		}
	}
out:
	g_free(transfer->buffer);
	libusb_free_transfer(transfer);
}

static void capture_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	int r;

	switch (ssm->cur_state) {
	case CAPTURE_WRITE_REQS:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_OUT, capture_reqs,
			sizeof(capture_reqs), capture_reqs_cb, ssm, BULK_TIMEOUT);
		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
		}
	}
	break;
	case CAPTURE_READ_DATA:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		unsigned char *data;

		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			break;
		}

		data = g_malloc(8192);
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_IN, data, 8192,
			capture_read_data_cb, ssm, BULK_TIMEOUT);

		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			g_free(data);
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, r);
		}
	}
	break;
	case CAPTURE_SET_IDLE:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_OUT, capture_set_idle_reqs,
			sizeof(capture_set_idle_reqs), capture_set_idle_reqs_cb, ssm, BULK_TIMEOUT);
		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
		}
	}
	break;
	};
}

static void capture_sm_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	struct aes2550_dev *aesdev = dev->priv;

	fp_dbg("Capture completed");
	if (aesdev->deactivating)
		complete_deactivation(dev);
	else if (ssm->error)
		fpi_imgdev_session_error(dev, ssm->error);
	else
		start_finger_detection(dev);
	fpi_ssm_free(ssm);
}

static void start_capture(struct fp_img_dev *dev)
{
	struct aes2550_dev *aesdev = dev->priv;
	struct fpi_ssm *ssm;

	if (aesdev->deactivating) {
		complete_deactivation(dev);
		return;
	}

	ssm = fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
	fp_dbg("");
	ssm->priv = dev;
	fpi_ssm_start(ssm, capture_sm_complete);
}

/****** INITIALIZATION/DEINITIALIZATION ******/

/* FIXME: remove magic constants */
static unsigned char init_reqs[] = {
	0x80, 0x01, /* Master reset */
	0xa1, 0x00,
	0x80, 0x12,
	0x85, 0x80,
	0xa8, 0x10,
	0xb1, 0x20,
	0x81, 0x04,
};

/* FIXME: remove magic constants */
static unsigned char calibrate_reqs[] = {
	0x80, 0x01, /* Master reset */
	0xdd, 0x00, /* Debug Off */
	0x06, /* Run calibration */
	0x10, /* Read calibration table */
};

enum activate_states {
	WRITE_INIT,
	READ_DATA,
	CALIBRATE,
	READ_CALIB_TABLE,
	ACTIVATE_NUM_STATES,
};

static void init_reqs_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;

	if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) &&
		(transfer->length == transfer->actual_length)) {
		fpi_ssm_next_state(ssm);
	} else {
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
	libusb_free_transfer(transfer);
}

static void init_read_data_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		fpi_ssm_next_state(ssm);
	} else {
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
	g_free(transfer->buffer);
	libusb_free_transfer(transfer);
}

/* TODO: use calibration table */
static void calibrate_read_data_cb(struct libusb_transfer *transfer)
{
	struct fpi_ssm *ssm = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		fpi_ssm_next_state(ssm);
	} else {
		fpi_ssm_mark_aborted(ssm, -EIO);
	}
	g_free(transfer->buffer);
	libusb_free_transfer(transfer);
}

static void activate_run_state(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	int r;

	switch (ssm->cur_state) {
	case WRITE_INIT:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_OUT, init_reqs,
			sizeof(init_reqs), init_reqs_cb, ssm, BULK_TIMEOUT);
		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
		}
	}
	break;
	case READ_DATA:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		unsigned char *data;

		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			break;
		}

		data = g_malloc(8192);
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_IN, data, 8192,
			init_read_data_cb, ssm, BULK_TIMEOUT);

		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			g_free(data);
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, r);
		}
	}
	break;
	case CALIBRATE:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			return;
		}
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_OUT, calibrate_reqs,
			sizeof(calibrate_reqs), init_reqs_cb, ssm, BULK_TIMEOUT);
		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
		}
	}
	break;
	case READ_CALIB_TABLE:
	{
		struct libusb_transfer *transfer = libusb_alloc_transfer(0);
		unsigned char *data;

		if (!transfer) {
			fpi_ssm_mark_aborted(ssm, -ENOMEM);
			break;
		}

		data = g_malloc(8192);
		libusb_fill_bulk_transfer(transfer, dev->udev, EP_IN, data, 8192,
			calibrate_read_data_cb, ssm, BULK_TIMEOUT);

		r = libusb_submit_transfer(transfer);
		if (r < 0) {
			g_free(data);
			libusb_free_transfer(transfer);
			fpi_ssm_mark_aborted(ssm, r);
		}
	}
	break;
	}
}

static void activate_sm_complete(struct fpi_ssm *ssm)
{
	struct fp_img_dev *dev = ssm->priv;
	fp_dbg("status %d", ssm->error);
	fpi_imgdev_activate_complete(dev, ssm->error);

	if (!ssm->error)
		start_finger_detection(dev);
	fpi_ssm_free(ssm);
}

static int dev_activate(struct fp_img_dev *dev, enum fp_imgdev_state state)
{
	struct aes2550_dev *aesdev = dev->priv;
	struct fpi_ssm *ssm = fpi_ssm_new(dev->dev, activate_run_state,
		ACTIVATE_NUM_STATES);
	ssm->priv = dev;
	aesdev->data_len = 0;
	fpi_ssm_start(ssm, activate_sm_complete);
	return 0;
}

static void dev_deactivate(struct fp_img_dev *dev)
{
	struct aes2550_dev *aesdev = dev->priv;

	aesdev->deactivating = TRUE;
}

static void complete_deactivation(struct fp_img_dev *dev)
{
	struct aes2550_dev *aesdev = dev->priv;
	fp_dbg("");

	aesdev->deactivating = FALSE;
	g_slist_free(aesdev->strips);
	aesdev->strips = NULL;
	aesdev->strips_len = 0;
	fpi_imgdev_deactivate_complete(dev);
}

static int dev_init(struct fp_img_dev *dev, unsigned long driver_data)
{
	/* FIXME check endpoints */
	int r;

	r = libusb_claim_interface(dev->udev, 0);
	if (r < 0) {
		fp_err("could not claim interface 0");
		return r;
	}

	dev->priv = g_malloc0(sizeof(struct aes2550_dev));
	fpi_imgdev_open_complete(dev, 0);
	return 0;
}

static void dev_deinit(struct fp_img_dev *dev)
{
	g_free(dev->priv);
	libusb_release_interface(dev->udev, 0);
	fpi_imgdev_close_complete(dev);
}

static const struct usb_id id_table[] = {
	{ .vendor = 0x08ff, .product = 0x2550 }, /* AES2550 */
	{ .vendor = 0x08ff, .product = 0x2810 }, /* AES2810 */
	{ 0, 0, 0, },
};

struct fp_img_driver aes2550_driver = {
	.driver = {
		.id = 4,
		.name = FP_COMPONENT,
		.full_name = "AuthenTec AES2550/AES2810",
		.id_table = id_table,
		.scan_type = FP_SCAN_TYPE_SWIPE,
	},
	.flags = 0,
	.img_height = -1,
	.img_width = 192,

	.open = dev_init,
	.close = dev_deinit,
	.activate = dev_activate,
	.deactivate = dev_deactivate,
};
