#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gradient_srv.h"
#include "model_handler.h"
#include "led_indication.h"
#include "reverse_routing.h"
#include "heartbeat.h"

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_DBG);

static const struct shell *chat_shell;

// Counter cho data payload
static uint16_t data_counter = 0;

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/

static void attention_on(const struct bt_mesh_model *mod)
{
	led_indicate_attention(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	led_indicate_attention(false);
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/******************************************************************************/
/***************************** Button handler setup ***************************/
/******************************************************************************/

// Button callback handler
static void button_handler(uint32_t button_state, uint32_t has_changed);

/******************************************************************************/
/***************************** Gradient model setup *******************************/
/******************************************************************************/

/**
 * Returns true if the node is new
 */
static void print_client_status(void);

static void handle_chat_start(struct bt_mesh_gradient_srv *Gradient)
{
	print_client_status();
	
	/* Start heartbeat after mesh is ready */
	heartbeat_start(Gradient);
}

static void handle_data_received(struct bt_mesh_gradient_srv *srv, uint16_t data)
{
	LOG_INF("[App] BACKPROP data received: %d", data);
	/* Add application-specific handling here */
}

static const struct bt_mesh_gradient_srv_handlers chat_handlers = {
	.start = handle_chat_start,
	.data_received = handle_data_received,
};

/* .. include_startingpoint_model_handler_rst_1 */
/* Non-static to allow access from shell_commands.c */
struct bt_mesh_gradient_srv gradient_srv = {
	.handlers = &chat_handlers,
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(
		1,
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub)),
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_GRADIENT_SRV(&gradient_srv))),
};
/* .. include_endpoint_model_handler_rst_1 */

static void print_client_status(void)
{
	if (!bt_mesh_is_provisioned()) {
		shell_print(chat_shell,
			    "The mesh node is not provisioned. Please provision the mesh node before using the chat.");
	} else {
		shell_print(chat_shell,
			    "The mesh node is provisioned. The client address is 0x%04x.",
			    bt_mesh_model_elem(gradient_srv.model)->rt->addr);
	}
}

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/******************************************************************************/
/******************************** Public API **********************************/
/******************************************************************************/
const struct bt_mesh_comp *model_handler_init(void)
{
    chat_shell = shell_backend_uart_get_ptr();

    // Khởi tạo forwarding table cho TẤT CẢ nodes
    for(int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++){
        gradient_srv.forwarding_table[i].addr = BT_MESH_ADDR_UNASSIGNED;
        gradient_srv.forwarding_table[i].rssi = INT8_MIN;
        gradient_srv.forwarding_table[i].gradient = UINT8_MAX;
        gradient_srv.forwarding_table[i].last_seen = 0;
        gradient_srv.forwarding_table[i].backprop_dest = NULL;
    }

    // Khởi tạo reverse routing table
    rrt_init(gradient_srv.forwarding_table, 
             CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    // Khởi tạo heartbeat module
    heartbeat_init();

#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
	
    // Node này là sink (cấu hình qua Kconfig)
    gradient_srv.gradient = 0;
    LOG_INF("Initialized as SINK node (gradient = 0)\n");
#else
    // Node thông thường
    gradient_srv.gradient = UINT8_MAX;
    LOG_INF("Initialized as regular node (gradient = 255)\n");
#endif

    // Khởi tạo buttons
    int err = dk_buttons_init(button_handler);
    if (err) {
        LOG_INF("Failed to initialize buttons (err %d)\n", err);
    } else {
        LOG_INF("Button 0 initialized - Press to send data to sink\n");
    }

    return &comp;
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    /* Button 1: Send DATA (regular node only) */
    if (has_changed & button_state & DK_BTN1_MSK) 
    {
        LOG_INF("\n=== Button 1 Pressed ===");
        
        if (!bt_mesh_is_provisioned()) {
            LOG_INF("ERROR: Node not provisioned yet");
            return;
        }
        
        uint16_t my_addr = bt_mesh_model_elem(gradient_srv.model)->rt->addr;
        LOG_INF("My address: 0x%04x, gradient: %d", my_addr, gradient_srv.gradient);
        
        /* Gateway should use shell command: mesh backprop <addr> */
        if (gradient_srv.gradient == 0) {
            LOG_INF("Gateway mode: Use shell command 'mesh backprop <addr>' to send BACKPROP");
            LOG_INF("========================\n");
            return;
        }
        
        /* Regular node: Send DATA to sink */
        bool has_route = false;
        for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
            if (gradient_srv.forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
                has_route = true;
                break;
            }
        }
        
        if (!has_route) {
            LOG_INF("ERROR: No route available, waiting for gradient messages...");
            LOG_INF("========================\n");
            return;
        }
        
        data_counter++;
        uint16_t dest_addr = gradient_srv.forwarding_table[0].addr;

        LOG_INF("Sending data %d to 0x%04x...", data_counter, dest_addr);

        bt_mesh_gradient_srv_data_send(&gradient_srv, dest_addr, data_counter);
       
        LOG_INF("========================\n");
    }
    
    /* Button 2: Hint to use shell commands */
    if (has_changed & button_state & DK_BTN2_MSK)
    {
        LOG_INF("\n=== Debug commands available via shell ===");
        LOG_INF("  mesh info        - Show node info");
        LOG_INF("  mesh fwd         - Show forwarding table");
        LOG_INF("  mesh rrt         - Show reverse routing table");
        LOG_INF("  mesh backprop    - Send BACKPROP (Gateway only)");
        LOG_INF("  mesh destinations - List known destinations");
        LOG_INF("==========================================\n");
    }
}
