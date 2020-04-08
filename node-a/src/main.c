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
#include "board.h"

#define APP_NAME "NODE-A"

#define BUTTON_DEBOUNCE_DELAY_MS 250

#define OP_ENVI_REPORT_SET              BT_MESH_MODEL_OP_2(0x82, 0x00)
#define OP_ENVI_REPORT                  BT_MESH_MODEL_OP_2(0x82, 0x01)

static struct bt_mesh_model *mod_cli = NULL;
static struct bt_mesh_model_pub *mod_pub = NULL;

static struct gpio_callback button_cb;
static struct device *sw_device;

static void button_pressed(struct device *dev, struct gpio_callback *cb, u32_t pin_pos);

static u16_t addr;

static struct {
        u16_t local;
        u16_t dst;
        u16_t net_idx;
        u16_t app_idx;
} net = {
        .local = BT_MESH_ADDR_UNASSIGNED,
        .dst = BT_MESH_ADDR_UNASSIGNED,
};

static struct bt_mesh_cfg_srv cfg_srv = {
        .relay = BT_MESH_RELAY_DISABLED,
        .beacon = BT_MESH_BEACON_DISABLED,
#if defined(CONFIG_BT_MESH_FRIEND)
        .frnd = BT_MESH_FRIEND_ENABLED,
#else
        .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
        .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
        .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
        .default_ttl = 7,

        /* 3 transmissions with 20ms interval */
        .net_transmit = BT_MESH_TRANSMIT(2, 20),
        .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
//      .hb_sub.func = heartbeat,
};

#if 0
static struct bt_mesh_cfg_cli cfg_cli = {
};
#endif

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 9/*max msg size*/ + 2/*opcode side*/);


static void envi_report(struct bt_mesh_model *model,
		struct bt_mesh_msg_ctx *ctx,
		struct net_buf_simple *buf)
{
        s8_t temperature = net_buf_simple_pull_u8(buf);
        u16_t airp = net_buf_simple_pull_le16(buf);
        u16_t humidity = net_buf_simple_pull_le16(buf);

       printk("------------------------\n");
       printk("-->[0x%04x] ", ctx->addr);
       printk("temperature:%hhd airp:%hu humidity:%hu\n", temperature, airp, humidity);
       printk("-------------------------\n");
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{  OP_ENVI_REPORT, 0, envi_report},
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	//BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, &gen_onoff_pub_cli, NULL),

};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void prov_complete(u16_t net_idx, u16_t addr)
{
	struct bt_mesh_elem *ele = &elements[0];

	printk(APP_NAME"%s\n", __func__);
	
	net.local = addr;
        net.net_idx = net_idx,
        net.dst = addr;
	
	board_prov_complete();
	addr = ele->addr;
	printk("GW_ADDR = 0x%04x\n", addr);
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const u8_t dev_uuid[16] = { 0xdb, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.complete = prov_complete,
	.reset = prov_reset,
};

static void bt_ready(int err)
{
	struct bt_mesh_elem *ele;;

	if (err) {
		printk(APP_NAME"%s: Bluetooth init failed (err %d)\n", __func__, err);
		return;
	}

	printk(APP_NAME"%s:Bluetooth initialized\n", __func__);

	board_init();

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk(APP_NAME"%s: Initializing mesh failed (err %d)\n", __func__, err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
        err = bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	if (err == -EALREADY) {
		printk("\nUsing stored settings\n");
		ele = &elements[0];
		addr = ele->addr;
		printk("GW_ADDR = 0x%04x\n", addr);
	} else {
		printk("\n\nPlease provision the node\n\n");
	}

	printk(APP_NAME"%s: Mesh initialized\n", __func__);
	mod_cli = &root_models[ARRAY_SIZE(root_models)-1];
	mod_pub = mod_cli->pub;
	//mod_pub->send_rel = 1;
	printk("\n....NODE-A....\n");

	err = bt_set_name("Node-A");
	if(err)
		printk("bt_set_name failed, err=%d\n", err);

}

void init_led(const char *port, u32_t pin_num)
{
        struct device *led_device = device_get_binding(port);
        gpio_pin_configure(led_device,
                           pin_num, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
        gpio_pin_write(led_device, pin_num, 1);
}

void main(void)
{
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

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

}

static uint8_t pin_to_sw(uint32_t pin_pos)
{
	switch (pin_pos) {
	case BIT(DT_GPIO_KEYS_SW0_GPIOS_PIN): return 0;
#if !CONFIG_BOARD_NRF52840_PCA10059 
	case BIT(DT_GPIO_KEYS_SW1_GPIOS_PIN): return 1;
	case BIT(DT_GPIO_KEYS_SW2_GPIOS_PIN): return 2;
	case BIT(DT_GPIO_KEYS_SW3_GPIOS_PIN): return 3;
#endif

	default:
		printk("Invalid pin_pos\n");
	}

	printk("No match for GPIO pin 0x%08x\n", pin_pos);

	return 0;
}
static void envi_report_set(u8_t cmd);
static u32_t time, last_time;
static void button_pressed(struct device *dev, struct gpio_callback *cb,
		u32_t pin_pos)
{
	u8_t button;

	printk(APP_NAME"%s\n", __func__);

	time = k_uptime_get_32();

	/* debounce the switch */
	if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
		last_time = time;
		return;
	}

	last_time = time;

	button = pin_to_sw(pin_pos);

	envi_report_set(button);
}

static void envi_report_set(u8_t cmd)
{
	int ret;

	bt_mesh_model_msg_init(mod_pub->msg, OP_ENVI_REPORT_SET);
	
	cmd = cmd ? 1 : 0;
	net_buf_simple_add_u8(mod_pub->msg, cmd); //cmd = 0 to stop and 1 to start the envi report.


	ret = bt_mesh_model_publish(mod_cli);

	if (ret)	
		printk("\nmsg publish failed, ERR = %d\n", ret);

}

