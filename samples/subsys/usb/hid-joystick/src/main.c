/*
 * Copyright (c) 2026 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/class/usbd_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define HID_JOYSTICK_REPORT_DESC(bcnt) \
{ \
	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP), \
	HID_USAGE(HID_USAGE_GEN_DESKTOP_JOYSTICK), \
	HID_COLLECTION(HID_COLLECTION_APPLICATION), \
		HID_USAGE_PAGE(HID_USAGE_GEN_BUTTON), \
		HID_USAGE_MIN8(1), \
		HID_USAGE_MAX8(bcnt), \
		HID_LOGICAL_MIN8(0), \
		HID_LOGICAL_MAX8(1), \
		HID_REPORT_SIZE(1), \
		HID_REPORT_COUNT(bcnt), \
		HID_INPUT(0x02), \
		HID_REPORT_SIZE(8 - bcnt), \
		HID_REPORT_COUNT(1), \
		HID_INPUT(0x03), \
		HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP), \
		HID_USAGE(HID_USAGE_GEN_DESKTOP_X), \
		HID_USAGE(HID_USAGE_GEN_DESKTOP_Y), \
		HID_LOGICAL_MIN8(-127), \
		HID_LOGICAL_MAX8(127), \
		HID_REPORT_SIZE(8), \
		HID_REPORT_COUNT(2), \
		HID_INPUT(0x02), \
	HID_END_COLLECTION \
}

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const uint8_t hid_report_desc[] = HID_JOYSTICK_REPORT_DESC(1);
static enum usb_dc_status_code usb_status;
static atomic_t hid_iface_ready;

enum joystick_report_idx {
	JOYSTICK_BUTTON = 0,
	JOYSTICK_X = 1,
	JOYSTICK_Y = 2,
	JOYSTICK_REPORT_COUNT = 3,
};

K_MSGQ_DEFINE(joystick_msgq, JOYSTICK_REPORT_COUNT, 2, 1);
static K_SEM_DEFINE(ep_write_sem, 0, 1);

static inline void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

static ALWAYS_INLINE void rwup_if_suspended(void)
{
	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}
}

static void input_cb(struct input_event *evt, void *user_data)
{
	static uint8_t tmp[JOYSTICK_REPORT_COUNT];

	ARG_UNUSED(user_data);

	/* Keep previous report state so multiple directional keys/buttons can be combined. */
	switch (evt->code) {

	case INPUT_KEY_RIGHT:     /* Right */
		rwup_if_suspended();
		tmp[JOYSTICK_X] = evt->value ? 20 : 0;
		break;

	case INPUT_KEY_LEFT:     /* Left */
		rwup_if_suspended();
		tmp[JOYSTICK_X] = evt->value ? -20 : 0;
		break;

	case INPUT_KEY_UP:     /* Up */
		rwup_if_suspended();
		tmp[JOYSTICK_Y] = evt->value ? -20 : 0;
		break;

	case INPUT_KEY_DOWN:     /* Down */
		rwup_if_suspended();
		tmp[JOYSTICK_Y] = evt->value ? 20 : 0;
		break;

	case INPUT_KEY_ENTER:     /* Center Button */
		rwup_if_suspended();
		WRITE_BIT(tmp[JOYSTICK_BUTTON], 0, evt->value);
		break;

	default:
		LOG_INF("Unrecognized input code %u value %d",
			evt->code, evt->value);
		return;
	}

	if (k_msgq_put(&joystick_msgq, tmp, K_NO_WAIT) != 0) {
		LOG_ERR("Failed to put new input event");
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static int enable_usb_device_next(void)
{
	struct usbd_context *sample_usbd;
	int err;

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* defined(CONFIG_USB_DEVICE_STACK_NEXT) */

#if !defined(CONFIG_USB_DEVICE_STACK_NEXT)
static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&ep_write_sem);
}
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
static void iface_ready_cb(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);
	atomic_set(&hid_iface_ready, ready ? 1 : 0);
	LOG_INF("HID iface ready=%d", ready ? 1 : 0);
	if (ready) {
		/*
		 * Allow first HID report transmission
		 */
		k_sem_give(&ep_write_sem);
	}
}

static void input_report_done_cb(const struct device *dev,
				 const uint8_t *const report)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(report);
	k_sem_give(&ep_write_sem);
}

static int hid_get_report(const struct device *dev, const uint8_t type, const uint8_t id,
			  const uint16_t len, uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(id);
	ARG_UNUSED(len);
	ARG_UNUSED(buf);

	return 0;
}

static const struct hid_device_ops hid_ops = {
	.iface_ready = iface_ready_cb,
	.get_report = hid_get_report,
	.input_report_done = input_report_done_cb,
};
#endif

#if !defined(CONFIG_USB_DEVICE_STACK_NEXT)
static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
};
#endif

UDC_STATIC_BUF_DEFINE(hid_joystick_report, JOYSTICK_REPORT_COUNT);

int main(void)
{
	const struct device *hid_dev;
	int ret;

	if (!gpio_is_ready_dt(&led0)) {
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
#else
	hid_dev = device_get_binding("HID_0");
#endif
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	ret = hid_device_register(hid_dev, hid_report_desc, sizeof(hid_report_desc), &hid_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID device, %d", ret);
		return 0;
	}
	ret = enable_usb_device_next();
#else
	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				&ops);
	usb_hid_init(hid_dev);
	ret = usb_enable(status_cb);
#endif
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	while (true) {
		k_msgq_get(&joystick_msgq, hid_joystick_report, K_FOREVER);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		if (!atomic_get(&hid_iface_ready)) {
			while (!atomic_get(&hid_iface_ready)) {
				k_sleep(K_MSEC(10));
			}
		}
		k_sem_take(&ep_write_sem, K_FOREVER);
		ret = hid_device_submit_report(hid_dev,
					       JOYSTICK_REPORT_COUNT,
					       hid_joystick_report);
		if (ret != 0) {
			LOG_ERR("hid_device_submit_report failed (%d)", ret);

			/*
			 * Give semaphore back if submit failed
			 */
			k_sem_give(&ep_write_sem);
		}
#else
		ret = hid_int_ep_write(hid_dev,
				       hid_joystick_report,
				       JOYSTICK_REPORT_COUNT,
				       NULL);

		if (ret == 0) {
			k_sem_take(&ep_write_sem, K_FOREVER);
		}
#endif

		if (ret) {
			LOG_ERR("HID write failed (%d)", ret);
		} else {
			gpio_pin_toggle_dt(&led0);
		}
	}
}
