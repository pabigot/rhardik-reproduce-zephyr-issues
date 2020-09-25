#ifndef _APP_PROV_H_
#define _APP_PROV_H_

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
#include <drivers/counter.h>

#define CONFIG_BT_MESH_ZEPHYR_PROVISION true
#define CONFIG_BT_MESH_SELF_PROVISION false

#if CONFIG_BT_MESH_ZEPHYR_PROVISION
	#define CONFIG_BT_MESH_SELF_PROVISION false
	#define CONFIG_BT_MESH_APP_PROVISION false
#elif CONFIG_BT_MESH_SELF_PROVISION
	#define CONFIG_BT_MESH_APP_PROVISION false
#else
	#define CONFIG_BT_MESH_APP_PROVISION true
#endif

extern u8_t dev_uuid[16];
extern const struct bt_mesh_prov prov;
extern u16_t local_node_addr;
extern bool node_provisioned;

struct net_st {
        u16_t local;
        u16_t dst;
        u16_t net_idx;
        u16_t app_idx;
};
extern struct net_st net;

extern struct bt_mesh_model *nw_range_mod_cli;
extern struct bt_mesh_model_pub *nw_range_mod_pub;

int config_node_relay(u16_t node_addr, u8_t new_relay, u8_t new_transmit, u8_t *status, u8_t *transmit);

#endif /* _APP_PROV_H_ */
