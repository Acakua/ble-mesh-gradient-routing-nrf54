#include <zephyr/bluetooth/mesh.h>
#include "gradient_srv.h"
#include "mesh/net.h"
#include <string.h>
#include <limits.h>
#include <zephyr/logging/log.h>

#include "neighbor_table.h"
#include "routing_policy.h"
#include "led_indication.h"
#include "data_forward.h"
#include "gradient_work.h"
#include "reverse_routing.h"
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(gradient_srv, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------
 * DEFINES (FIXED: Thêm các định nghĩa thiếu)
 * ------------------------------------------------------------------------- */
#ifndef BT_MESH_GRADIENT_SRV_DEFAULT_TTL
#define BT_MESH_GRADIENT_SRV_DEFAULT_TTL 10
#endif

#ifndef BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER 0xFFFF
#endif

BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
            BT_MESH_RX_SDU_MAX,
         "The message must fit inside an application SDU.");
BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
            BT_MESH_TX_SDU_MAX,
         "The message must fit inside an application SDU.");

/* -------------------------------------------------------------------------
 * HELPER FUNCTIONS (FIXED: Đặt lên đầu để tránh lỗi implicit declaration)
 * ------------------------------------------------------------------------- */

/**
 * @brief Hàm in log định dạng CSV ra UART cho Sink Node
 */
static void log_csv_data(uint16_t src_addr, uint16_t sender_addr, 
                         uint16_t data, int8_t rssi, uint8_t ttl_received)
{
    uint8_t hop_count = 0;
    
    // Tính toán số bước nhảy (Hop Count)
    if (ttl_received > 0 && ttl_received <= BT_MESH_GRADIENT_SRV_DEFAULT_TTL) {
        hop_count = BT_MESH_GRADIENT_SRV_DEFAULT_TTL - ttl_received;
    } else {
        // Trường hợp TTL lạ hoặc bằng 0
        hop_count = 0; 
    }

    // Format: CSV_LOG,Source,Sender,Data,RSSI,Hops
    printk("CSV_LOG,%u,%u,%u,%d,%u\n", 
           src_addr, sender_addr, data, rssi, hop_count);
}

/******************************************************************************/
/* Message Handlers                                                           */
/******************************************************************************/

static int handle_gradient_mesage(const struct bt_mesh_model *model, 
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint8_t msg;
    int8_t rssi = ctx->recv_rssi;
    uint16_t sender_addr = ctx->addr;

    /* Skip messages from self */
    if (sender_addr == bt_mesh_model_elem(gradient_srv->model)->rt->addr) {
        return 0;
    }

    /* Check RSSI threshold using routing_policy */
    if (!rp_is_candidate_acceptable(rssi)) {
        return 0;
    }

    /* LED indication */
    led_indicate_gradient_received();

    msg = net_buf_simple_pull_u8(buf);

    // [THÊM ĐÁNH NHÃN CONTROL]
    LOG_INF("[CONTROL - Gradient Beacon] Received from: 0x%04x, Gradient: %d", sender_addr, msg);

    /* Check if gradient should be processed using routing_policy */
    if (!rp_should_process_gradient(msg, gradient_srv->gradient)) {
        return 0;
    }

    /* Schedule processing in work context */
    gradient_work_schedule_process(gradient_srv, msg, sender_addr, rssi);
    
    return 0;
}

static int handle_data_message(const struct bt_mesh_model *model, 
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint16_t sender_addr = ctx->addr;
    int8_t rssi = ctx->recv_rssi;
    uint8_t ttl_received = ctx->recv_ttl; // Lấy TTL để tính hops
    
    uint16_t original_source = net_buf_simple_pull_le16(buf);
    uint16_t received_data = net_buf_simple_pull_le16(buf);

    // [THÊM LOGIC PHÂN LOẠI DATA / HEARTBEAT]
    if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
        LOG_INF("[CONTROL - Heartbeat] Recv from original: 0x%04x (via 0x%04x)", original_source, sender_addr);
    } else {
        LOG_INF("[DATA - Sensor] Recv from original: 0x%04x (via 0x%04x), Data: %d", original_source, sender_addr, received_data);
    }
    
    int64_t now = k_uptime_get();
    
    k_mutex_lock(&gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    /* ════════════════════════════════════════════════════════════════════
     * FIX RACE CONDITION: Beacon vs Data
     * ════════════════════════════════════════════════════════════════════ */
    
    bool sender_found = false;
    int sender_index = -1;
    
    /* Tìm sender trong forwarding table */
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            sender_found = true;
            sender_index = i;
            break;
        }
    }
    
    if (sender_found) {
        /* CASE 1: Node đã tồn tại -> Refresh timestamp */
        gradient_srv->forwarding_table[sender_index].last_seen = now;
        gradient_srv->forwarding_table[sender_index].rssi = rssi;
        
        LOG_DBG("[Data RX] Refreshed 0x%04x: grad=%d (unchanged), rssi=%d", 
                sender_addr, 
                gradient_srv->forwarding_table[sender_index].gradient,
                rssi);
    } else {
        /* CASE 2: Node mới -> Chỉ log, đợi beacon */
        LOG_INF("[Data RX] Unknown sender 0x%04x (not in fwd table yet)", sender_addr);
    }
    
    /* ════════════════════════════════════════════════════════════════════
     * Reverse Route Learning (RRT)
     * ════════════════════════════════════════════════════════════════════ */
    int err = rrt_add_dest(gradient_srv->forwarding_table,
                           CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                           sender_addr,      /* nexthop */
                           original_source,  /* destination */
                           now);
    
    if (err == 0) {
        LOG_DBG("[RRT] Learned: dest=0x%04x via nexthop=0x%04x", 
                original_source, sender_addr);
    }
    
    k_mutex_unlock(&gradient_srv->forwarding_table_mutex);

    /* ════════════════════════════════════════════════════════════════════
     * Forwarding Logic & LOGGING
     * ════════════════════════════════════════════════════════════════════ */
    
    /* Gateway (Sink): Nhận data, không forward */
    if (gradient_srv->gradient == 0) {
        led_indicate_sink_received();
        
        /* GỌI HÀM LOG CSV CHO SINK (Chỉ log data thực, không log heartbeat vào CSV) */
        if (received_data != BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
            log_csv_data(original_source, sender_addr, received_data, rssi, ttl_received);
        }
        
        /* Check if this is heartbeat */
        if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
            LOG_INF("[CONTROL - Heartbeat] Arrived at Sink from 0x%04x", original_source);
        } else {
            /* Normal data - callback to application */
            if (gradient_srv->handlers->data_received) {
                gradient_srv->handlers->data_received(gradient_srv, received_data);
            }
        }
        return 0;
    }
    
    /* Regular Node: Forward data (Bao gồm cả Heartbeat và Data) */
    err = data_forward_send(gradient_srv, received_data, original_source, sender_addr);
    
    if (err == 0) {
        LOG_DBG("[Forward] Queued packet from src=0x%04x", original_source);
    } else {
        LOG_ERR("[Forward] Failed to forward, err=%d", err);
    }
    
    return 0;
}


/**
 * @brief Handle BACKPROP_DATA message (Gateway → Node downlink)
 */
static int handle_backprop_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    uint16_t sender_addr = ctx->addr;
    
    /* Parse packet: [final_dest:2] + [ttl:1] + [payload:2] */
    uint16_t final_dest = net_buf_simple_pull_le16(buf);
    uint8_t ttl = net_buf_simple_pull_u8(buf);
    uint16_t payload = net_buf_simple_pull_le16(buf);
    
    // [THÊM ĐÁNH NHÃN CONTROL]
    LOG_INF("[CONTROL - Backprop] Received: dest=0x%04x, payload=%d, from=0x%04x", final_dest, payload, sender_addr);
    
    /* Check if we are the final destination */
    if (final_dest == my_addr) {
        LOG_INF("[CONTROL - Backprop] I am the destination! Payload: %d", payload);
        led_indicate_backprop_received();
        
        if (gradient_srv->handlers->data_received) {
            gradient_srv->handlers->data_received(gradient_srv, payload);
        }
        return 0;
    }
    
    /* Check TTL */
    if (ttl <= 1) {
        LOG_WRN("[CONTROL - Backprop] TTL expired, dropping packet");
        return 0;
    }
    
    /* Find nexthop from Reverse Routing Table */
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        final_dest);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("[CONTROL - Backprop] No route to dest=0x%04x, dropping", final_dest);
        return 0;
    }
    
    /* Forwarding logic */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);
    net_buf_simple_add_le16(&msg, final_dest);
    net_buf_simple_add_u8(&msg, ttl - 1);
    net_buf_simple_add_le16(&msg, payload);
    
    struct bt_mesh_msg_ctx tx_ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = nexthop,
        .send_ttl = 0,
    };
    
    int err = bt_mesh_model_send(gradient_srv->model, &tx_ctx, &msg, NULL, NULL);
    if (err) {
        LOG_ERR("[CONTROL - Backprop] Forward failed, err=%d", err);
    }
    
    return 0;
}

/******************************************************************************/
/* Model Operations                                                           */
/******************************************************************************/

const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[] = {
    {
        BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
        BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
        handle_gradient_mesage
    },
    {
        BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE,
        BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
        handle_data_message
    },
    {
        BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA,
        BT_MESH_LEN_EXACT(5),
        handle_backprop_message
    },
    BT_MESH_MODEL_OP_END,
};

/******************************************************************************/
/* Model Callbacks                                                            */
/******************************************************************************/

static int bt_mesh_gradient_srv_update_handler(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    struct net_buf_simple *buf = model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
    net_buf_simple_add_u8(buf, gradient_srv->gradient);

    LOG_INF("[CONTROL - Gradient Beacon] Auto-publishing gradient: %d", gradient_srv->gradient);

    return 0;
}

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

static int bt_mesh_gradient_srv_init(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    gradient_srv->model = model;

    net_buf_simple_init_with_data(&gradient_srv->pub_msg, gradient_srv->buf,
                      sizeof(gradient_srv->buf));
    gradient_srv->pub.msg = &gradient_srv->pub_msg;
    gradient_srv->pub.update = bt_mesh_gradient_srv_update_handler;

    /* Initialize mutex for forwarding table protection */
    k_mutex_init(&gradient_srv->forwarding_table_mutex);

    /* Initialize all sub-modules */
    led_indication_init();
    data_forward_init();
    gradient_work_init();

    return 0;
}

static int bt_mesh_gradient_srv_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    if (gradient_srv->handlers->start) {
        gradient_srv->handlers->start(gradient_srv);
    }

    /* Set global reference for work handlers */
    gradient_work_set_srv(gradient_srv);
    
    /* Auto-configure publication */
    if (model->pub) {
        model->pub->addr = BT_MESH_ADDR_ALL_NODES;
        model->pub->ttl = 0;
        model->pub->period = BT_MESH_PUB_PERIOD_SEC(5);
        
        LOG_INF("Auto-configured publication: addr=0x%04x", model->pub->addr);
        
        gradient_work_schedule_initial_publish();
    }
    
    /* Start cleanup timer */
    gradient_work_start_cleanup();
    
    return 0;
}

static void bt_mesh_gradient_srv_reset(const struct bt_mesh_model *model)
{
    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        (void) bt_mesh_model_data_store(model, true, NULL, NULL, 0);
    }
}

const struct bt_mesh_model_cb _bt_mesh_gradient_srv_cb = {
    .init = bt_mesh_gradient_srv_init,
    .start = bt_mesh_gradient_srv_start,
#ifdef CONFIG_BT_SETTINGS
    .settings_set = bt_mesh_gradient_srv_settings_set,
#endif
    .reset = bt_mesh_gradient_srv_reset,
};

/******************************************************************************/
/* Public APIs                                                                */
/******************************************************************************/

int bt_mesh_gradient_srv_gradient_send(struct bt_mesh_gradient_srv *gradient_srv)
{
    struct net_buf_simple *buf = gradient_srv->model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
    net_buf_simple_add_u8(buf, gradient_srv->gradient);

    LOG_INF("[CONTROL - Gradient Beacon] Manually sending gradient status");
    return bt_mesh_model_publish(gradient_srv->model);
}

int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
                                   uint16_t addr,
                                   uint16_t data)
{
    return data_forward_send_direct(gradient_srv, addr, data);
}

int bt_mesh_gradient_srv_backprop_send(struct bt_mesh_gradient_srv *gradient_srv,
                                       uint16_t dest_addr,
                                       uint16_t payload)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    
    /* Cannot send to self */
    if (dest_addr == my_addr) {
        LOG_ERR("[CONTROL - Backprop Send] Cannot send to self");
        return -EINVAL;
    }
    
    /* Find nexthop from Reverse Routing Table */
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        dest_addr);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("[CONTROL - Backprop Send] No route to dest=0x%04x", dest_addr);
        return -ENETUNREACH;
    }
    
    LOG_INF("[CONTROL - Backprop Send] Target: 0x%04x via Nexthop: 0x%04x, Data: %d",
            dest_addr, nexthop, payload);
    
    /* Prepare BACKPROP_DATA message */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);
    net_buf_simple_add_le16(&msg, dest_addr);
    net_buf_simple_add_u8(&msg, BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL);
    net_buf_simple_add_le16(&msg, payload);
    
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = nexthop,
        .send_ttl = 0,
    };
    
    return bt_mesh_model_send(gradient_srv->model, &ctx, &msg, NULL, NULL);
}