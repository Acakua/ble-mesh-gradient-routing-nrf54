#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/drivers/uart.h>

#include "gradient_srv.h"
#include "model_handler.h"

static const struct shell *chat_shell;

// Counter cho data payload
static uint16_t data_counter = 0;

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/
/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
		BIT(0) | BIT(1),
		BIT(1) | BIT(2),
		BIT(2) | BIT(3),
		BIT(3) | BIT(0),
	};

	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(const struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
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
}

static const struct bt_mesh_gradient_srv_handlers chat_handlers = {
	.start = handle_chat_start,
};

/* .. include_startingpoint_model_handler_rst_1 */
static struct bt_mesh_gradient_srv gradient_srv = {
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
    k_work_init_delayable(&attention_blink_work, attention_blink);

    chat_shell = shell_backend_uart_get_ptr();

    // Khởi tạo forwarding table cho TẤT CẢ nodes
    for(int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++){
        gradient_srv.forwarding_table[i].addr = BT_MESH_ADDR_UNASSIGNED;
        gradient_srv.forwarding_table[i].rssi = INT8_MIN;
        gradient_srv.forwarding_table[i].gradient = UINT8_MAX;
    }

#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
	
    // Node này là sink (cấu hình qua Kconfig)
    gradient_srv.gradient = 0;
    printk("Initialized as SINK node (gradient = 0)\n");
#else
    // Node thông thường
    gradient_srv.gradient = UINT8_MAX;
    printk("Initialized as regular node (gradient = 255)\n");
#endif

    // Khởi tạo buttons
    int err = dk_buttons_init(button_handler);
    if (err) {
        printk("Failed to initialize buttons (err %d)\n", err);
    } else {
        printk("Button 0 initialized - Press to send data to sink\n");
    }

    return &comp;
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	// Kiểm tra nếu button 0 được nhấn 
	if (has_changed & button_state & DK_BTN1_MSK) {
		printk("\n=== Button 0 Pressed ===\n");
		
		// Kiểm tra nếu node chưa được provision
		if (!bt_mesh_is_provisioned()) {
			printk("ERROR: Node not provisioned yet\n");
			return;
		}
		
		// In thông tin node hiện tại
		uint16_t my_addr = bt_mesh_model_elem(gradient_srv.model)->rt->addr;
		printk("My address: 0x%04x, gradient: %d\n", 
			   my_addr, gradient_srv.gradient);
		
		// Nếu node này là sink, không gửi data
		if (gradient_srv.gradient == 0) {
			printk("This is sink node, cannot send data\n");
			return;
		}
		
		// In forwarding table
		printk("Forwarding table:\n");
		bool has_route = false;
		for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
			if (gradient_srv.forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
				printk("  [%d] addr=0x%04x, gradient=%d, rssi=%d\n",
					   i,
					   gradient_srv.forwarding_table[i].addr,
					   gradient_srv.forwarding_table[i].gradient,
					   gradient_srv.forwarding_table[i].rssi);
				has_route = true;
			}
		}
		
		if (!has_route) {
			printk("ERROR: No route available, waiting for gradient messages...\n");
			return;
		}
		
		// Gửi data đến parent tốt nhất (index 0)
		uint16_t dest_addr = gradient_srv.forwarding_table[0].addr;
		data_counter++;
		
		printk("Sending data %d to 0x%04x...\n", data_counter, dest_addr);
		
		int err = bt_mesh_gradient_srv_data_send(&gradient_srv, 
												  dest_addr, 
												  data_counter);
		
		if (err) {
			printk("ERROR: Send failed with error %d\n", err);
		} else {
			printk("SUCCESS: Data %d sent to 0x%04x\n", data_counter, dest_addr);
		}
		
		printk("========================\n\n");
	}
}
