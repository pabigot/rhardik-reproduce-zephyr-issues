//* main.c - Application main entry point */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
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


#include <nrfx/hal/nrf_temp.h>
#include <drivers/flash.h>


#define APP_NAME "Node"

#define OP_DUMMY_SET BT_MESH_MODEL_OP_2(0x82, 0x2f)

static struct bt_mesh_model *mod_cli = NULL;
static struct bt_mesh_model_pub *mod_pub = NULL;

static s8_t die_temperature(void);

static struct bt_mesh_cfg_srv cfg_srv = {
        .relay = BT_MESH_RELAY_ENABLED,
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
        .net_transmit = BT_MESH_TRANSMIT(0, 20),
        .relay_retransmit = BT_MESH_TRANSMIT(0, 20),
};

static u8_t dummy_flg;
static void dummy_thread(void);
K_THREAD_DEFINE(dummy_tid, 2048, dummy_thread, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
static void dummy_thread(void)
{
	while (1) {
			k_sleep(K_MSEC(8000));
			printk("flag = %d\n",dummy_flg);
			printf("%s\n", __func__);
			//k_thread_suspend(dummy_tid);
	}
}

static void dummy_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	dummy_flg = net_buf_simple_pull_u8(buf);

	if (dummy_flg) {
		printk("WAKEUP:NODE:Start DUMMY report\n");
		k_wakeup(dummy_tid);
		printk("RESUME:NODE:Start DUMMY report\n");
		k_thread_resume(dummy_tid);
	} else {
		printk("SUSPEND:NODE:Stop DUMMY report\n");
		k_thread_suspend(dummy_tid);
	}
}

static const struct bt_mesh_model_op gen_level_srv_op[] = {
        { OP_DUMMY_SET, 0, dummy_set },
        BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, gen_level_srv_op,
			NULL, NULL),

};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static u16_t node_addr;
static void prov_complete(u16_t net_idx, u16_t addr)
{
	struct bt_mesh_elem *ele = &elements[0];

	printk(APP_NAME"%s\n", __func__);
	//board_prov_complete();
	node_addr = ele->addr;
	printk("NODE_ADDR = 0x%04x\n", node_addr);

}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}
static const u8_t dev_uuid[16] = { 0xdd, 0xda };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.complete = prov_complete,
	.reset = prov_reset,
};

static void bt_ready(int err)
{
	struct bt_mesh_elem *ele = &elements[0];
	if (err) {
		printk(APP_NAME"%s: Bluetooth init failed (err %d)\n", __func__, err);
		return;
	}

	printk(APP_NAME"%s:Bluetooth initialized\n", __func__);

	//board_init();

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk(APP_NAME"%s: Initializing mesh failed (err %d)\n", __func__, err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	mod_cli = &root_models[ARRAY_SIZE(root_models)-1];
	mod_pub = mod_cli->pub;
	/* This will be a no-op if settings_load() loaded provisioning info */
	err = bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	if (err == -EALREADY) {
		printk("\nUsing stored settings\n");
		ele = &elements[0];
		node_addr = ele->addr;
		printk("NODE_ADDR = 0x%04x\n", node_addr);
	} else {
		printk("\n\nPlease provision the node\n\n");
	}
	
	printk(APP_NAME"%s: Mesh initialized\n", __func__);
	printk("\n....Node....\n");
	err = bt_set_name("Node");
	if(err)
		printk("bt_set_name failed, err=%d\n", err);

}

void main(void)
{       
        
	int err;
	
	k_thread_suspend(dummy_tid);

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	
	printk("Bluetooth initialized\n");
}

static s8_t die_temperature(void)
{
        int32_t temp;

        NRF_TEMP->TASKS_START = 1;
        k_sleep(K_MSEC(100));
        temp = (nrf_temp_result_get(NRF_TEMP) / 4);
        NRF_TEMP->TASKS_STOP = 1;

        return temp;
}
