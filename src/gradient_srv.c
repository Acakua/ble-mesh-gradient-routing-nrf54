#include <zephyr/bluetooth/mesh.h>
#include "gradient_srv.h"
#include "mesh/net.h"
#include <string.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gradient_srv, LOG_LEVEL_DBG);

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
static int blink_count1 = 0;

static struct bt_mesh_gradient_srv *g_gradient_srv = NULL;

static struct k_work_delayable initial_publish_work;
static struct k_work_delayable gradient_process_work;

/* Work item for data send retry */
static struct k_work_delayable data_retry_work;

/* Work item for cleanup expired nodes */
static struct k_work_delayable cleanup_work;

/* Context để lưu data gradient processing */
struct gradient_context {
    struct bt_mesh_gradient_srv *gradient_srv;
    uint8_t gradient_msg;
    uint16_t sender_addr;
    int8_t rssi;
};

static struct gradient_context gradient_ctx = {0};

/* Context cho data send với retry */
struct data_send_context {
    struct bt_mesh_gradient_srv *gradient_srv;
    uint16_t data;
    uint16_t sender_addr;      // Địa chỉ node gửi ban đầu (để skip)
    int current_index;         // Index hiện tại trong forwarding table
    bool active;               // Đang có pending send
};

static struct data_send_context data_send_ctx = {0};

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
    
    if (blink_count1 < 6) {  // 3 lần bật/tắt
        if (led_state) {
            dk_set_led_on(DK_LED2);  // LED khác
        } else {
            dk_set_led_off(DK_LED2);
        }
        led_state = !led_state;
        blink_count1++;
        
        k_work_schedule(&led1_blink_work, K_MSEC(100));
    } else {
        blink_count1 = 0;
        dk_set_led_off(DK_LED2);
    }
}

// Work handler cho initial publish
static void initial_publish_handler(struct k_work *work)
{
    if (g_gradient_srv != NULL) {
        int err = bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
        
        if (err) {
            LOG_INF("Initial publish failed: %d\n", err);
        } else {
            LOG_INF("Initial gradient published: %d\n", 
                   g_gradient_srv->gradient);
        }
    }
}

/* Hàm xóa entry khỏi forwarding table */
static void remove_forwarding_entry(struct bt_mesh_gradient_srv *gradient_srv, int index)
{
    if (index < 0 || index >= CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        return;
    }
    
    uint16_t removed_addr = gradient_srv->forwarding_table[index].addr;
    
    // Shift các entries lên
    for (int i = index; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE - 1; i++) {
        gradient_srv->forwarding_table[i] = gradient_srv->forwarding_table[i + 1];
    }
    
    // Clear entry cuối cùng
    int last = CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE - 1;
    gradient_srv->forwarding_table[last].addr = BT_MESH_ADDR_UNASSIGNED;
    gradient_srv->forwarding_table[last].rssi = INT8_MIN;
    gradient_srv->forwarding_table[last].gradient = UINT8_MAX;
    gradient_srv->forwarding_table[last].last_seen = 0;  
    
    LOG_INF("[Forwarding] Removed 0x%04x from index %d", removed_addr, index);
}

/* Work handler để cleanup expired nodes */
static void cleanup_handler(struct k_work *work)
{
    if (g_gradient_srv == NULL) {
        return;
    }
    
    int64_t current_time = k_uptime_get();
    bool table_changed = false;
    
    LOG_DBG("[Cleanup] Running cleanup check...");
    
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (g_gradient_srv->forwarding_table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }
        
        int64_t time_diff = current_time - g_gradient_srv->forwarding_table[i].last_seen;
        
        if (time_diff > CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS) {
            LOG_WRN("[Cleanup] Node 0x%04x expired (last seen %lld ms ago)",
                    g_gradient_srv->forwarding_table[i].addr, time_diff);
            
            // Xóa entry expired
            remove_forwarding_entry(g_gradient_srv, i);
            
            table_changed = true;
            i--;  // Recheck current index after shift
        }
    }
    
    // ✅ Cập nhật lại gradient nếu bảng thay đổi
    if (table_changed) {
        if (g_gradient_srv->forwarding_table[0].addr != BT_MESH_ADDR_UNASSIGNED) {
            uint8_t best_parent_gradient = g_gradient_srv->forwarding_table[0].gradient;
            
            // Recalculate gradient based on best available parent
            uint8_t new_gradient = best_parent_gradient + 1;
            
            if (g_gradient_srv->gradient != new_gradient) {
                uint8_t old_gradient = g_gradient_srv->gradient;
                g_gradient_srv->gradient = new_gradient;
                
                LOG_INF("[Cleanup] Gradient recalculated: %d -> %d", 
                        old_gradient, g_gradient_srv->gradient);
                
                bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
            }
        } else {
            // Không còn parent nào → reset gradient về max (nếu không phải sink)
            if (g_gradient_srv->gradient != UINT8_MAX && g_gradient_srv->gradient != 0) {
                LOG_WRN("[Cleanup] No parents available, resetting gradient to 255");
                g_gradient_srv->gradient = UINT8_MAX;
                bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
            }
        }
        
        // In forwarding table sau cleanup
        LOG_INF("[Cleanup] Forwarding table after cleanup:");
        for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
            if (g_gradient_srv->forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
                LOG_INF("  [%d] addr=0x%04x, gradient=%d, rssi=%d",
                        i,
                        g_gradient_srv->forwarding_table[i].addr,
                        g_gradient_srv->forwarding_table[i].gradient,
                        g_gradient_srv->forwarding_table[i].rssi);
            }
        }
    }
    
    // ✅ Reschedule cleanup (chạy mỗi 10 giây)
    k_work_schedule(&cleanup_work, K_MSEC(10000));
}

/* Forward declaration */
static int bt_mesh_gradient_srv_data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                                                    uint16_t addr,
                                                    uint16_t data);

/* Work handler cho data retry */
static void data_retry_handler(struct k_work *work)
{
    struct data_send_context *ctx = &data_send_ctx;
    
    if (!ctx->active || !ctx->gradient_srv) {
        ctx->active = false;
        return;
    }
    
    struct bt_mesh_gradient_srv *gradient_srv = ctx->gradient_srv;
    
    // Tìm next valid entry
    while (ctx->current_index < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        uint16_t next_addr = gradient_srv->forwarding_table[ctx->current_index].addr;
        
        // Hết entry valid
        if (next_addr == BT_MESH_ADDR_UNASSIGNED) {
            LOG_ERR("[Retry] No more valid entries, data lost!");
            ctx->active = false;
            return;
        }
        
        // Skip sender (tránh gửi ngược lại)
        if (next_addr == ctx->sender_addr) {
            LOG_DBG("[Retry] Skipping sender 0x%04x", next_addr);
            ctx->current_index++;
            continue;
        }
        
        // Thử gửi đến entry này
        LOG_INF("[Retry] Trying index %d: addr=0x%04x", ctx->current_index, next_addr);
        
        int err = bt_mesh_gradient_srv_data_send_internal(gradient_srv, next_addr, ctx->data);
        
        if (err) {
            LOG_ERR("[Retry] Failed to queue send to 0x%04x, err=%d", next_addr, err);
            // Xóa entry này và thử tiếp
            remove_forwarding_entry(gradient_srv, ctx->current_index);
            // Không tăng current_index vì entries đã shift
            continue;
        }
        
        // Đã queue thành công, chờ callback
        return;
    }
    
    // Hết entries
    LOG_ERR("[Retry] All entries exhausted, data lost!");
    ctx->active = false;
}

/*Callback khi message được gửi xong */
static void data_send_end_cb(int err, void *user_data)
{
    uint16_t dest_addr = (uint16_t)(uintptr_t)user_data;
    struct data_send_context *ctx = &data_send_ctx;
    
    if (err) {
        LOG_ERR("[TX Complete] FAILED to send to 0x%04x, err=%d", dest_addr, err);
        
        if (ctx->active && ctx->gradient_srv) {
            // Xóa entry thất bại
            LOG_INF("[TX Complete] Removing failed node 0x%04x from forwarding table", dest_addr);
            remove_forwarding_entry(ctx->gradient_srv, ctx->current_index);
            
            //Schedule retry (không tăng index vì entries đã shift)
            k_work_schedule(&data_retry_work, K_MSEC(100));
        }
    } else {
        LOG_INF("[TX Complete] SUCCESS sent to 0x%04x", dest_addr);
        ctx->active = false;  // Hoàn thành
    }
}

//Callback structure
static const struct bt_mesh_send_cb data_send_cb = {
    .start = NULL,  // Optional: called when transmission starts
    .end = data_send_end_cb,  // Called when transmission completes
};

/* Internal send function (không update context) */
static int bt_mesh_gradient_srv_data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
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
    
    // Truyền callback để biết kết quả thực sự
    return bt_mesh_model_send(gradient_srv->model, &ctx, &buf, 
                              &data_send_cb, 
                              (void *)(uintptr_t)addr);  
}

// Work handler để xử lý forwarding table và cập nhật gradient 
static void gradient_process_handler(struct k_work *work)
{
    struct gradient_context *ctx = &gradient_ctx;
    struct bt_mesh_gradient_srv *gradient_srv = ctx->gradient_srv;
    uint8_t msg = ctx->gradient_msg;
    int8_t rssi = ctx->rssi;
    uint16_t sender_addr = ctx->sender_addr;
    
    if (!gradient_srv) {
        return;
    }

    LOG_INF("Received gradient %d from 0x%04x (RSSI: %d)\n", 
           msg, sender_addr, rssi);
    
    int64_t current_time = k_uptime_get();
    
    //Xử lý forwarding table
    int insert_pos = -1;
    int sender_pos = -1;
        
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) 
    {
        /** Find position to insert */
        if (gradient_srv->forwarding_table[i].addr == BT_MESH_ADDR_UNASSIGNED) 
        {
            insert_pos = i;
            break;
        }

        if (msg < gradient_srv->forwarding_table[i].gradient && insert_pos == -1) 
        {
            insert_pos = i;
        } else if (msg == gradient_srv->forwarding_table[i].gradient && 
                   rssi > gradient_srv->forwarding_table[i].rssi && insert_pos == -1) 
        {
            insert_pos = i;
        }

        /** Check if sender already exists */
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            sender_pos = i;
            if(sender_pos == insert_pos){
                insert_pos = -1;
            }


            gradient_srv->forwarding_table[sender_pos].rssi = rssi;
            gradient_srv->forwarding_table[sender_pos].gradient = msg;
            gradient_srv->forwarding_table[sender_pos].last_seen = current_time;
            
            LOG_INF("[Process] Updated existing entry at index [%d]: addr = [0x%04x], gradient = [%d]\n", 
                sender_pos, sender_addr, msg);
            break;
        }
    }
        
    if (insert_pos != -1) {
        if (sender_pos == -1){
            // Shift entries
            for (int i = CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE - 1; i > insert_pos; i--) 
            {
                gradient_srv->forwarding_table[i] = gradient_srv->forwarding_table[i - 1];
            }
        
            gradient_srv->forwarding_table[insert_pos].addr = sender_addr;
            gradient_srv->forwarding_table[insert_pos].rssi = rssi;
            gradient_srv->forwarding_table[insert_pos].gradient = msg;
            gradient_srv->forwarding_table[insert_pos].last_seen = current_time;
            
            LOG_INF("[Process] Inserted at index [%d]: addr=0x%04x, gradient=%d\n", 
                insert_pos, sender_addr, msg);
        }else{
            for (int i = sender_pos; i > insert_pos; i--) 
            {
                gradient_srv->forwarding_table[i] = gradient_srv->forwarding_table[i - 1];
            }
            gradient_srv->forwarding_table[insert_pos].addr = sender_addr;
            gradient_srv->forwarding_table[insert_pos].rssi = rssi;
            gradient_srv->forwarding_table[insert_pos].gradient = msg;
            gradient_srv->forwarding_table[insert_pos].last_seen = current_time;  

            LOG_INF("[Process] Moved entry from index [%d] to index [%d]: addr = [0x%04x], gradient = [%d]\n", 
                sender_pos, insert_pos, sender_addr, msg);
        }
    }

    /* Cập nhật gradient */
    if (gradient_srv->forwarding_table[0].addr != BT_MESH_ADDR_UNASSIGNED) {
        uint8_t best_parent_gradient = gradient_srv->forwarding_table[0].gradient;
            
        if (gradient_srv->gradient > best_parent_gradient + 1) {
            uint8_t old_gradient = gradient_srv->gradient;
            gradient_srv->gradient = best_parent_gradient + 1;
            
            LOG_INF("[Process] Gradient updated: [%d] -> [%d]\n", 
                   old_gradient, gradient_srv->gradient);
            
            // Publish ngay
            bt_mesh_gradient_srv_gradient_send(gradient_srv);
        }
    }
    
    LOG_INF("[Process] Gradient handling completed\n");
    gradient_ctx.gradient_srv = NULL;
}

static int handle_gradient_mesage(const struct bt_mesh_model *model, 
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint8_t msg;
    int8_t rssi = ctx->recv_rssi;
    uint16_t sender_addr = ctx->addr;

    //Bỏ qua message từ chính node
    if(sender_addr == bt_mesh_model_elem(gradient_srv->model)->rt->addr){
        return 0;
    }

    if(rssi < -75){
        return 0;
    }

    // Nháy LED khi nhận gradient
    blink_count2 = 0;
    k_work_schedule(&led2_blink_work, K_NO_WAIT);

    msg = net_buf_simple_pull_u8(buf);

    if(msg > gradient_srv->gradient){
        return 0;
    }

    gradient_ctx.gradient_srv = gradient_srv;
    gradient_ctx.gradient_msg = msg;
    gradient_ctx.sender_addr = sender_addr;
    gradient_ctx.rssi = rssi;

    k_work_schedule(&gradient_process_work, K_NO_WAIT);
    
    return 0;
}

/* .. include_startingpoint_gradient_srv_rst_1 */
static int handle_data_message(const struct bt_mesh_model *model, 
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    uint16_t sender_addr = ctx->addr;
    
    uint16_t received_data = net_buf_simple_pull_le16(buf);

    LOG_INF("Received data %d from 0x%04x", received_data, sender_addr);

    if (gradient_srv->gradient == 0) {
        static bool led0_state = false;
        led0_state = !led0_state;
        dk_set_led(DK_LED1, led0_state);
        LOG_INF("[Sink] Data received: %d", received_data);
        return 0;
    }
    
    blink_count1 = 0;
    k_work_schedule(&led1_blink_work, K_NO_WAIT);
    
    // Kiểm tra có route không
    if (gradient_srv->forwarding_table[0].addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("[Forward] No route available!");
        return 0;
    }
    
    // Setup retry context
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = received_data;
    data_send_ctx.sender_addr = sender_addr;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;
    
    while (data_send_ctx.current_index < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        uint16_t dest_addr = gradient_srv->forwarding_table[data_send_ctx.current_index].addr;
        
        if (dest_addr == BT_MESH_ADDR_UNASSIGNED) {
            LOG_ERR("[Forward] No valid destination!");
            data_send_ctx.active = false;
            return 0;
        }
        
        if (dest_addr == sender_addr) {
            data_send_ctx.current_index++;
            continue;
        }
        
        // Gửi đến entry này
        LOG_INF("[Forward] Sending data %d to 0x%04x (index %d)", 
                received_data, dest_addr, data_send_ctx.current_index);
        
        int err = bt_mesh_gradient_srv_data_send_internal(gradient_srv, dest_addr, received_data);
        
        if (err) {
            LOG_ERR("[Forward] Failed to queue, err=%d", err);
            remove_forwarding_entry(gradient_srv, data_send_ctx.current_index);
            continue;
        }
        
        return 0;  // Chờ callback
    }
    
    LOG_ERR("[Forward] No valid destination after filtering!");
    data_send_ctx.active = false;
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

    LOG_INF("Auto-published gradient: %d\n", gradient_srv->gradient);

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

    k_work_init_delayable(&led1_blink_work, led1_blink_handler);

    /* Khởi tạo initial publish work */
    k_work_init_delayable(&initial_publish_work, initial_publish_handler);

    /* Khởi tạo gradient process work */
    k_work_init_delayable(&gradient_process_work, gradient_process_handler);

    // Khởi tạo data retry work
    k_work_init_delayable(&data_retry_work, data_retry_handler);
    
    // Khởi tạo cleanup work
    k_work_init_delayable(&cleanup_work, cleanup_handler);

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
        model->pub->period = BT_MESH_PUB_PERIOD_SEC(5);
        
        LOG_INF("Auto-configured publication: addr=0x%04x\n", 
               model->pub->addr);
        
        k_work_schedule(&initial_publish_work, K_MSEC(500));
    }
    
    //Bắt đầu cleanup timer (chạy sau 10 giây đầu tiên)
    k_work_schedule(&cleanup_work, K_MSEC(15000));
    LOG_INF("Cleanup timer started (timeout: %d ms)", 
            CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS);
    
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

/* Public API - khởi tạo retry context */
int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
                                    uint16_t addr,
                                    uint16_t data)
{
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.sender_addr = BT_MESH_ADDR_UNASSIGNED;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;
    
    return bt_mesh_gradient_srv_data_send_internal(gradient_srv, addr, data);
}
