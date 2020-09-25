#include "app_prov.h"
#include <sys/printk.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#define APP_NAME "GW"

static const u16_t net_idx;
static u16_t app_idx;

u8_t dev_uuid[16] = {0xaa, 0xcc};
const struct bt_mesh_prov prov;

bool node_provisioned;

struct bt_mesh_model *nw_range_mod_cli = NULL;
struct bt_mesh_model_pub *nw_range_mod_pub = NULL;
struct bt_mesh_model *cfg_mod_cli = NULL;
struct bt_mesh_model_pub *cfg_mod_pub = NULL;

u16_t self_addr = 0x0001;
static u16_t node_addr;
static u8_t node_uuid[16];


K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);

static void prov_complete(u16_t net_idx, u16_t addr);
static  void prov_reset(void);
static void unprovisioned_beacon(u8_t uuid[16],
				 bt_mesh_prov_oob_info_t oob_info,
				 u32_t *uri_hash);
static void node_added(u16_t net_idx, u8_t uuid[16], u16_t addr, u8_t num_elem);

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
	//.hb_sub.func = heartbeat,
};
static struct bt_mesh_cfg_cli cfg_cli = {
};

BT_MESH_MODEL_PUB_DEFINE(gen_level_pub_cli, NULL, 9/*max msg size*/ + 2/*opcode side*/);

void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
				  u8_t test_id, u16_t cid, u8_t *faults,
				  size_t fault_count);

struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_CLI, NULL, &gen_level_pub_cli, NULL),
};

struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

struct net_st net = {
        .local = BT_MESH_ADDR_UNASSIGNED,
        .dst = BT_MESH_ADDR_UNASSIGNED,
};

const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.complete = prov_complete,
	.reset = prov_reset,
        .unprovisioned_beacon = unprovisioned_beacon,
        .node_added = node_added,
};

const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

void bt_ready(int err)
{

	if (err) {
		printk(APP_NAME"%s: Bluetooth init failed (err %d)\n", __func__, err);
		return;
	}

	printk(APP_NAME"%s:Bluetooth initialized\n", __func__);


	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk(APP_NAME"%s: Initializing mesh failed (err %d)\n", __func__, err);
		return;
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_load();
	}

	nw_range_mod_cli = &root_models[ARRAY_SIZE(root_models)-1];
	nw_range_mod_pub = nw_range_mod_cli->pub;
	///////////////////////////////////////////////////////////////////////////
	nw_range_mod_pub->mod = nw_range_mod_cli;
	nw_range_mod_pub->addr = 0x0002;
	nw_range_mod_pub->key = 0; 
	nw_range_mod_pub->cred = false;
	nw_range_mod_pub->ttl = 7;
	nw_range_mod_pub->period = 0;
	nw_range_mod_pub->retransmit = 0;
	/////////////////////////////////////////////////////////////////////////////

	provisioner_thread_set(1);

	err = bt_set_name("Node-A");
	if(err)
		printk("bt_set_name failed, err=%d\n", err);

	printk(APP_NAME"%s: Mesh initialized\n", __func__);
}


static void setup_cdb(void)
{
	struct bt_mesh_cdb_app_key *key;

	key = bt_mesh_cdb_app_key_alloc(net_idx, app_idx);
	if (key == NULL) {
		printk("Failed to allocate app-key 0x%04x\n", app_idx);
		return;
	}

	bt_rand(key->keys[0].app_key, 16);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_app_key_store(key);
	}
}

static void configure_self(struct bt_mesh_cdb_node *self)
{
	struct bt_mesh_cdb_app_key *key;
	int err;
	u8_t status;

	printk("Configuring self...\n");

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printk("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(self->net_idx, self->addr, self->net_idx,
				      app_idx, key->keys[0].app_key, NULL);
	if (err < 0) {
		printk("Failed to add app-key (err %d)\n", err);
		return;
	}

	err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr, app_idx, \
			BT_MESH_MODEL_ID_GEN_LEVEL_CLI, NULL);
	if (err < 0) {
		printk("%s:bt_mesh_cfg_mod_app_bind ERR (%d)\n", __func__, err);
		return;
	}
#if 0 	
	struct bt_mesh_cfg_mod_pub pub;
        pub.addr = 0xc001;
        pub.app_idx = key->app_idx;
        pub.cred_flag = false;
        pub.ttl = 7;
        //pub.period = BT_MESH_PUB_PERIOD_10SEC(1);
        pub.period = 0;
        pub.transmit = 0;

        err = bt_mesh_cfg_mod_pub_set(net_idx, self->addr, self->addr,
                                      BT_MESH_MODEL_ID_GEN_LEVEL_CLI, &pub,
                                      &status);
        if (err < 0) {
                printk("mod_pub_set.. %d, %d\n", err, status);
                return;
        }
#endif
	
	atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

        if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
                bt_mesh_cdb_node_store(self);
        }

	printk("Configuration complete\n");
}

static void configure_node(struct bt_mesh_cdb_node *node)
{
	struct bt_mesh_cdb_app_key *key;
	int err;

	printk("Configuring node 0x%04x...\n", node->addr);

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printk("No app-key 0x%04x\n", app_idx);
		return;
	}

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx,
				      key->keys[0].app_key, NULL);
	if (err < 0) {
		printk("Failed to add app-key (err %d)\n", err);
		return;
	}

	err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr, node->addr, app_idx, \
			BT_MESH_MODEL_ID_GEN_LEVEL_SRV, NULL);
	if (err < 0) {
		printk("%s:bt_mesh_cfg_mod_app_bind ERR (%d)\n", __func__, err);
		return;
	}
	
	 bt_mesh_cfg_mod_sub_add(net_idx, node->addr, node->addr, 0xc001, \
                        BT_MESH_MODEL_ID_GEN_LEVEL_SRV, NULL);


	atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

        if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
                bt_mesh_cdb_node_store(node);
        }

	printk("Configuration complete\n");
}

void unprovisioned_beacon(u8_t uuid[16],
				 bt_mesh_prov_oob_info_t oob_info,
				 u32_t *uri_hash)
{
	memcpy(node_uuid, uuid, 16);
	k_sem_give(&sem_unprov_beacon);
}

void node_added(u16_t net_idx, u8_t uuid[16], u16_t addr, u8_t num_elem)
{

	node_addr = addr;

	k_sem_give(&sem_node_added);
}

static int zephyr_provisioner_init(void)
{
	u8_t net_key[16], dev_key[16];
	int err;

	printk("FFFFFFFFFFFFFFFFFFFFF\n");
	bt_rand(net_key, 16);
	printk("GGGGGGGGGGGGGGGGGGGGG\n");
	err = bt_mesh_cdb_create(net_key);
	printk("HHHHHHHHHHHHHHHHHHHHHH\n");
	if (err == -EALREADY) {
		printk("Using stored CDB\n");
	} else if (err) {
		printk("Failed to create CDB (err %d)\n", err);
		return err;
	} else {
		printk("Created CDB\n");
		setup_cdb();
	}

	printk("IIIIIIIIIIIIIIIIIIIIIII\n");
	bt_rand(dev_key, 16);

	err = bt_mesh_provision(net_key, BT_MESH_NET_PRIMARY, 0, 0, self_addr,
				dev_key);
	if (err == -EALREADY) {
		printk("Using stored settings\n");
	} else if (err) {
		printk("%s:Provisioning failed (err %d)\n", __func__, err);
		return err;
	} else {
		printk("Provisioning completed\n");
	}
	printk("JJJJJJJJJJJJJJJJJJJJJJJJ\n");

	return 0;
}

static u8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
	if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED)) {
		if (node->addr == self_addr) {
			configure_self(node);
		} else {
			configure_node(node);
		}
	}

	return BT_MESH_CDB_ITER_CONTINUE;
}


void provisioner(void)
{
	char uuid_hex_str[32 + 1];
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	printk("DDDDDDDDDDDDDDDDDDDDDDDDDD\n");
	zephyr_provisioner_init();
	printk("EEEEEEEEEEEEEEEEEEEEEEEEEEE\n");
	while (1) {
		k_sem_reset(&sem_unprov_beacon);
		k_sem_reset(&sem_node_added);
		bt_mesh_cdb_node_foreach(check_unconfigured, NULL);
		#if 0
		bt_mesh_cdb_node_foreach(list_nodes, NULL);
		#endif
		printk("Waiting for unprovisioned beacon...\n");
		err = k_sem_take(&sem_unprov_beacon, K_SECONDS(10));
		if (err == -EAGAIN) {
			continue;
		}

		bin2hex(node_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

		printk("Provisioning %s\n", uuid_hex_str);
		err = bt_mesh_provision_adv(node_uuid, net_idx, 0, 0);
		if (err < 0) {
			printk("%s:Provisioning failed (err %d)\n", __func__, err);
			continue;
		}

		printk("Waiting for node to be added...\n");
		err = k_sem_take(&sem_node_added, K_SECONDS(10));
		if (err == -EAGAIN) {
			printk("Timeout waiting for node to be added\n");
			continue;
		}

		printk("Added node 0x%04x\n", node_addr);
	}
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
	struct bt_mesh_elem *ele = &elements[0];

	printk(APP_NAME"%s\n", __func__);
	
	net.local = addr;
        net.net_idx = net_idx,
        net.dst = addr;
	
	self_addr = ele->addr;
	printk("GW_ADDR = 0x%04x\n", addr);
	node_provisioned = 1;
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

u16_t get_element_addr(u8_t ele_num)
{
	struct bt_mesh_elem *ele = &elements[0];

	return ele->addr;
}

void provisioner(void);
void provisioner_thread(void)
{
	provisioner_thread_set(0);
	self_addr = 0x0001;
	provisioner();
}
