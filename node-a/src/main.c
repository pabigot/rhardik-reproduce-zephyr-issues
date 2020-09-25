/* main.c - Application main entry point */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <settings/settings.h>
#include <sys/byteorder.h>
#include <nrf.h>
#include <device.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>
#include "app_prov.h"

#define BUTTON_DEBOUNCE_DELAY_MS 250

#define OP_DUMMY_SET BT_MESH_MODEL_OP_2(0x82, 0x2f)

void provisioner_thread(void);
K_THREAD_DEFINE(provisioner_tid, 1024, provisioner_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

static struct gpio_callback button_cb;
static struct device *sw_device;

void dummy_report_set(u8_t cmd);

static void button_pressed(struct device *dev, struct gpio_callback *cb, u32_t pin_pos);


void init_led(const char *port, u32_t pin_num)
{
        struct device *led_device = device_get_binding(port);
        gpio_pin_configure(led_device,
                           pin_num, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
        gpio_pin_write(led_device, pin_num, 1);
}

void main(void)
{

	printk("AAAAAAAAAAAAAAAAAAAAAAAAA\n");
	sw_device = device_get_binding(DT_GPIO_KEYS_SW0_GPIOS_CONTROLLER);
	gpio_pin_configure(sw_device, DT_GPIO_KEYS_SW0_GPIOS_PIN,
			(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			 GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
#if !CONFIG_BOARD_NRF52840_PCA10059
	gpio_pin_configure(sw_device, DT_GPIO_KEYS_SW1_GPIOS_PIN,
			(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			 GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
	gpio_pin_configure(sw_device, DT_GPIO_KEYS_SW2_GPIOS_PIN,
			(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			 GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
	gpio_pin_configure(sw_device, DT_GPIO_KEYS_SW3_GPIOS_PIN,
			(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			 GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
#endif
	gpio_init_callback(&button_cb, button_pressed,
			BIT(DT_GPIO_KEYS_SW0_GPIOS_PIN)
#if !CONFIG_BOARD_NRF52840_PCA10059
			| BIT(DT_GPIO_KEYS_SW1_GPIOS_PIN)
			| BIT(DT_GPIO_KEYS_SW2_GPIOS_PIN)
			| BIT(DT_GPIO_KEYS_SW3_GPIOS_PIN)
#endif
		);

	gpio_add_callback(sw_device, &button_cb);
	gpio_pin_enable_callback(sw_device, DT_GPIO_KEYS_SW0_GPIOS_PIN);
#if !CONFIG_BOARD_NRF52840_PCA10059 
	gpio_pin_enable_callback(sw_device, DT_GPIO_KEYS_SW1_GPIOS_PIN);
	gpio_pin_enable_callback(sw_device, DT_GPIO_KEYS_SW2_GPIOS_PIN);
	gpio_pin_enable_callback(sw_device, DT_GPIO_KEYS_SW3_GPIOS_PIN);
#endif

#if CONFIG_BOARD_NRF52840_PCA10059
	init_led(DT_ALIAS_LED2_GPIOS_CONTROLLER, DT_ALIAS_LED2_GPIOS_PIN);
#else
	init_led(DT_ALIAS_LED1_GPIOS_CONTROLLER, DT_ALIAS_LED1_GPIOS_PIN);
#endif
	printk("BBBBBBBBBBBBBBBBBBBBB\n");
	provisioner();
	printk("CCCCCCCCCCCCCCCCCCCCC\n");

}

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
	case BIT(DT_GPIO_KEYS_SW0_GPIOS_PIN): return 0;
#if !CONFIG_BOARD_NRF52840_PCA10059 
	case BIT(DT_GPIO_KEYS_SW1_GPIOS_PIN): return 1;
//	case BIT(DT_GPIO_KEYS_SW2_GPIOS_PIN): return 2;
//	case BIT(DT_GPIO_KEYS_SW3_GPIOS_PIN): return 3;
#endif

	default:
		printk("Invalid pin_pos\n");
	}

	printk("No match for GPIO pin 0x%08x\n", pin_pos);

	return 0;
}
static u32_t time, last_time;
static void button_pressed(struct device *dev, struct gpio_callback *cb,
		u32_t pin_pos)
{
	u8_t button;

	time = k_uptime_get_32();

	/* debounce the switch */
	if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
		last_time = time;
		return;
	}

	last_time = time;

	button = pin_to_sw(pin_pos);

	dummy_report_set(button);
}

void dummy_report_set(u8_t cmd)
{
	int ret;

	bt_mesh_model_msg_init(nw_range_mod_pub->msg, OP_DUMMY_SET);
	
	cmd = cmd ? 1 : 0;
	net_buf_simple_add_u8(nw_range_mod_pub->msg, cmd); //cmd = 0 to stop and 1 to start the dummy report.


	ret = bt_mesh_model_publish(nw_range_mod_cli);

	if (ret) {
		printk("\nmsg publish failed, ERR = %d\n", ret);
		return;
	}

	if (cmd)
		printk("GW:Start dummy report\n");
	else
		printk("GW:Stop dummy report\n");

}

void provisioner_thread_set(bool onoff)
{
        if (onoff)
                k_thread_resume(provisioner_tid);
        else
                k_thread_suspend(provisioner_tid);
}

