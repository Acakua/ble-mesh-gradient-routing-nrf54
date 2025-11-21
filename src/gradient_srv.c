#include <zephyr/bluetooth/mesh.h>
#include "gradient_srv.h"
#include "mesh/net.h"
#include <string.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/drivers/uart.h>
#include <stdio.h>

/* Define UART device */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Helper function to print via UART */
static void uart_print(const char *buf)
{
    if (!device_is_ready(uart_dev)) {
        return;
    }
    
    int msg_len = strlen(buf);
    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

/* Macro to replace printk */
#define UART_PRINT(fmt, ...) \
    do { \
        char _buf[256]; \
        snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
        uart_print(_buf); \
    } while (0)

/* Define gradient publish interval (milliseconds) */
#define UPDATE_GRADIENT_INTERVAL 5000  

BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT,
				   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
		    BT_MESH_RX_SDU_MAX,
	     "The message must fit inside an application SDU.");
BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT,
				   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
		    BT_MESH_TX_SDU_MAX,
	     "The message must fit inside an application SDU.");

/* Work item for LED 2 blink */
static struct k_work_delayable led2_blink_work;
static int blink_count2 = 0;

/* Work item for LED 1 blink */
static struct k_work_delayable led1_blink_work;
static int blink_count21 = 0;

// Khai báo work item cho periodic gradient publish
static struct k_work_delayable gradient_publish_work;

// Biến global để lưu gradient_srv pointer
static struct bt_mesh_gradient_srv *g_gradient_srv = NULL;

// Work handler để nháy LED 2
static void led2_blink_handler(struct k_work *work)
{
    static bool led_state = false;
    
    if (blink_count2 < 6) {  // 3 lần bật/tắt = 6 transitions
        if (led_state) {
            dk_set_led_on(DK_LED3);
        } else {
            dk_set_led_off(DK_LED3);
        }
        led_state = !led_state;
        blink_count2++;
        
        // Schedule lại sau 100ms
        k_work_schedule(&led2_blink_work, K_MSEC(100));
    } else {
        blink_count2 = 0;
        dk_set_led_off(DK_LED3);  // Đảm bảo LED tắt
    }
}

// Work handler cho LED 1
static void led1_blink_handler(struct k_work *work)
{
    static bool led_state = false;
    
    if (blink_count21 < 6) {  // 3 lần bật/tắt
        if (led_state) {
            dk_set_led_on(DK_LED2);  // LED khác
        } else {
            dk_set_led_off(DK_LED2);
        }
        led_state = !led_state;
        blink_count21++;
        
        k_work_schedule(&led1_blink_work, K_MSEC(100));
    } else {
        blink_count21 = 0;
        dk_set_led_off(DK_LED2);
    }
}

// Work handler để publish gradient định kỳ
static void gradient_publish_handler(struct k_work *work)
{
    if (g_gradient_srv != NULL) {
        // Publish gradient message
        int err = bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
        
        if (err) {
            UART_PRINT("Failed to publish gradient: %d\n", err);
        } else {
            UART_PRINT("Published gradient: %d\n", g_gradient_srv->gradient);
        }
    }
    
    // Reschedule
    k_work_reschedule(&gradient_publish_work, K_MSEC(UPDATE_GRADIENT_INTERVAL));
}

static int handle_gradient_mesage(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
              struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint8_t msg;
    int8_t rssi = ctx->recv_rssi;
    uint16_t sender_addr = ctx->addr;

    if(sender_addr == bt_mesh_model_elem(gradient_srv->model)->rt->addr){
        // Tin nhắn từ chính nó, bỏ qua
        return 0;
    }

    blink_count2 = 0;
    k_work_schedule(&led2_blink_work, K_NO_WAIT);

    msg = net_buf_simple_pull_u8(buf);
    
    UART_PRINT("Received gradient %d from 0x%04x (RSSI: %d)\n", 
               msg, sender_addr, rssi);

    int insert_pos = -1;
        
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            break;
        }

        if (gradient_srv->forwarding_table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            insert_pos = i;
            break;
        }

        if (msg < gradient_srv->forwarding_table[i].gradient) {
            insert_pos = i;
            break;
        }else if (msg == gradient_srv->forwarding_table[i].gradient && 
                rssi > gradient_srv->forwarding_table[i].rssi) {
            insert_pos = i;
            break;
        }
    }
        
    if (insert_pos != -1) {
        for (int i = CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE - 1; i > insert_pos; i--) {
            gradient_srv->forwarding_table[i] = gradient_srv->forwarding_table[i - 1];
        }
    
        gradient_srv->forwarding_table[insert_pos].addr = sender_addr;
        gradient_srv->forwarding_table[insert_pos].rssi = rssi;
        gradient_srv->forwarding_table[insert_pos].gradient = msg;
    }

    // Cập nhật gradient của node hiện tại dựa trên node tốt nhất
    if (gradient_srv->forwarding_table[0].addr != BT_MESH_ADDR_UNASSIGNED) {
        uint8_t best_parent_gradient = gradient_srv->forwarding_table[0].gradient;
            
        if (gradient_srv->gradient > best_parent_gradient + 1) {
            gradient_srv->gradient = best_parent_gradient + 1;		
            bt_mesh_gradient_srv_gradient_send(gradient_srv);
        }
    }
    return 0;
}

/* .. include_startingpoint_gradient_srv_rst_1 */
static int handle_data_message(const struct bt_mesh_model *model, 
                                  struct bt_mesh_msg_ctx *ctx,
                  struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    
    uint16_t received_data = net_buf_simple_pull_le16(buf);

    if (gradient_srv->gradient == 0) {
        static bool led0_state = false;
        led0_state = !led0_state;
        
        dk_set_led(DK_LED1, led0_state); 
        return 0;
    }else{
		// Trigger LED2 blink (non-blocking)
		blink_count21 = 0;
		k_work_schedule(&led1_blink_work, K_NO_WAIT);
		for(int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++){
            int err;
            err = bt_mesh_gradient_srv_data_send(gradient_srv,
											gradient_srv->forwarding_table[i].addr,
											received_data);
			if(!err){
				break;
			}
		}
	}
    return 0;
}
/* .. include_endpoint_gradient_srv_rst_1 */

/* .. include_startingpoint_gradient_srv_rst_2 */
const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[] = {
	{
		BT_MESH_GRADIENT_SRV_OP_GRADIENT,
		BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
		handle_gradient_mesage
	},
	{
		BT_MESH_GRADIENT_SRV_OP_PRIVATE_MESSAGE,
		BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
		handle_data_message
	},
	BT_MESH_MODEL_OP_END,
};
/* .. include_endpoint_gradient_srv_rst_2 */

static int bt_mesh_gradient_srv_update_handler(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    // Publish gradient message
    struct net_buf_simple *buf = model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT);
    
    net_buf_simple_add_u8(buf, gradient_srv->gradient);

    UART_PRINT("Auto-published gradient: %d\n", gradient_srv->gradient);

    return 0;
}

/* .. include_startingpoint_gradient_srv_rst_3 */
#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_gradient_srv_settings_set(const struct bt_mesh_model *model,
					 const char *name,
					 size_t len_rd,
					 settings_read_cb read_cb,
					 void *cb_arg)
{
	if (name) {
		return -ENOENT;
	}

	return 0;
}
#endif
/* .. include_endpoint_gradient_srv_rst_3 */

/* .. include_startingpoint_gradient_srv_rst_4 */
static int bt_mesh_gradient_srv_init(const struct bt_mesh_model *model)
{
	struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

	gradient_srv->model = model;

	net_buf_simple_init_with_data(&gradient_srv->pub_msg, gradient_srv->buf,
				      sizeof(gradient_srv->buf));
	gradient_srv->pub.msg = &gradient_srv->pub_msg;
	gradient_srv->pub.update = bt_mesh_gradient_srv_update_handler;

	// Khởi tạo LED blink work
    k_work_init_delayable(&led2_blink_work, led2_blink_handler);

	// Khởi tạo LED2 blink work
    k_work_init_delayable(&led1_blink_work, led1_blink_handler);

	// Khởi tạo gradient publish work
    k_work_init_delayable(&gradient_publish_work, gradient_publish_handler);

	return 0;
}
/* .. include_endpoint_gradient_srv_rst_4 */

/* .. include_startingpoint_gradient_srv_rst_5 */
static int bt_mesh_gradient_srv_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    if (gradient_srv->handlers->start) {
        gradient_srv->handlers->start(gradient_srv);
    }

    g_gradient_srv = gradient_srv;
    
    // Auto-configure publication
    if (model->pub) {
        model->pub->addr = BT_MESH_ADDR_ALL_NODES;
        model->pub->ttl = 0;
        
        UART_PRINT("Auto-configured publication: addr=0x%04x\n", 
               model->pub->addr);
    }
    
    k_work_schedule(&gradient_publish_work, K_MSEC(UPDATE_GRADIENT_INTERVAL));
    
    return 0;
}
/* .. include_endpoint_gradient_srv_rst_5 */

/* .. include_startingpoint_gradient_srv_rst_6 */
static void bt_mesh_gradient_srv_reset(const struct bt_mesh_model *model)
{
	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		(void) bt_mesh_model_data_store(model, true, NULL, NULL, 0);
	}
}
/* .. include_endpoint_gradient_srv_rst_6 */

/* .. include_startingpoint_gradient_srv_rst_7 */
const struct bt_mesh_model_cb _bt_mesh_gradient_srv_cb = {
	.init = bt_mesh_gradient_srv_init,
	.start = bt_mesh_gradient_srv_start,
#ifdef CONFIG_BT_SETTINGS
	.settings_set = bt_mesh_gradient_srv_settings_set,
#endif
	.reset = bt_mesh_gradient_srv_reset,
};
/* .. include_endpoint_gradient_srv_rst_7 */

int bt_mesh_gradient_srv_gradient_send(struct bt_mesh_gradient_srv *gradient_srv)
{
    struct net_buf_simple *buf = gradient_srv->model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT);

    net_buf_simple_add_u8(buf, gradient_srv->gradient);

    return bt_mesh_model_publish(gradient_srv->model);
}

/* .. include_startingpoint_gradient_srv_rst_9 */
int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
                                    uint16_t addr,
                                    uint16_t data)
{
    struct bt_mesh_msg_ctx ctx = {
        .addr = addr,
        .app_idx = gradient_srv->model->keys[0],
        .send_ttl = 0,
        .send_rel = true,
    };

    BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_GRADIENT_SRV_OP_PRIVATE_MESSAGE, 2);
    bt_mesh_model_msg_init(&buf, BT_MESH_GRADIENT_SRV_OP_PRIVATE_MESSAGE);

    net_buf_simple_add_le16(&buf, data);
    
    return bt_mesh_model_send(gradient_srv->model, &ctx, &buf, NULL, NULL);
}
/* .. include_endpoint_gradient_srv_rst_9 */
