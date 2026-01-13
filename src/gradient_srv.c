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

LOG_MODULE_REGISTER(gradient_srv, LOG_LEVEL_DBG);

BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
            BT_MESH_RX_SDU_MAX,
         "The message must fit inside an application SDU.");
BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
            BT_MESH_TX_SDU_MAX,
         "The message must fit inside an application SDU.");

/******************************************************************************/
/*                          Message Handlers                                  */
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
    
    uint16_t original_source = net_buf_simple_pull_le16(buf);
    uint16_t received_data = net_buf_simple_pull_le16(buf);

    LOG_INF("Received DATA: original_src=0x%04x, sender=0x%04x, data=%d", 
            original_source, sender_addr, received_data);
    
    int64_t now = k_uptime_get();
    
    k_mutex_lock(&gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    /* === FIX: CHỐNG GHI ĐÈ GRADIENT === */
    bool sender_exists = false;
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            sender_exists = true;
            /* CHỈ cập nhật thời gian và RSSI, TUYỆT ĐỐI KHÔNG sửa Gradient */
            gradient_srv->forwarding_table[i].last_seen = now;
            gradient_srv->forwarding_table[i].rssi = rssi; 
            /* Cần re-sort lại nếu RSSI thay đổi nhiều, nhưng tạm thời 
               để đơn giản ta chỉ update field. 
               Nếu muốn sort lại, phải gọi nt_update_sorted với gradient CŨ */
            break;
        }
    }

    if (!sender_exists) {
        /* Chỉ đoán Gradient nếu là node hoàn toàn mới */
        uint8_t entry_gradient = (gradient_srv->gradient == 0) ? UINT8_MAX : gradient_srv->gradient + 1;
        LOG_INF("[Route Learn] New neighbor 0x%04x, guessing gradient=%d", sender_addr, entry_gradient);
        nt_update_sorted(gradient_srv->forwarding_table,
                         CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                         sender_addr, entry_gradient, rssi, now);
    }
    /* === END FIX === */
    
    /* Reverse Route Learning logic giữ nguyên... */
    int err = rrt_add_dest(gradient_srv->forwarding_table,
                           CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                           sender_addr,
                           original_source,
                           now);
    
    /* ... (phần còn lại của hàm giữ nguyên) ... */
    
    k_mutex_unlock(&gradient_srv->forwarding_table_mutex);

    /* Forwarding logic... */
    if (gradient_srv->gradient == 0) {
        led_indicate_sink_received();
        /* ... sink logic ... */
        return 0;
    }
    
    /* Sử dụng data_forward_send mới (đã có strict check) */
    err = data_forward_send(gradient_srv, received_data, original_source, sender_addr);
    /* ... */
    
    return 0;
}


/**
 * @brief Handle BACKPROP_DATA message (Gateway → Node downlink)
 *
 * Packet format: [final_dest: 2 bytes] + [ttl: 1 byte] + [payload: 2 bytes]
 * Total: 5 bytes
 *
 * Logic:
 * 1. If final_dest == my address → deliver locally
 * 2. If TTL <= MIN_TTL → drop (prevent loops)
 * 3. Otherwise → find nexthop from RRT, forward with TTL-1
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
    
    LOG_INF("[BACKPROP] Received: dest=0x%04x, ttl=%d, payload=%d, from=0x%04x",
            final_dest, ttl, payload, sender_addr);
    
    /* Check if we are the final destination */
    if (final_dest == my_addr) {
        LOG_INF("[BACKPROP] I am destination! Payload received: %d", payload);
        led_indicate_gradient_received();  /* Visual indication */
        
        /* Callback to application if needed */
        if (gradient_srv->handlers->data_received) {
            gradient_srv->handlers->data_received(gradient_srv, payload);
        }
        return 0;
    }
    
    /* FIX: Strict check to ensure we have enough TTL to forward */
    /* If current TTL is equal to or less than MIN+1, we stop here. */
    if (ttl <= BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL + 1) {
        LOG_WRN("[BACKPROP] TTL expired (%d <= %d), dropping packet for dest=0x%04x", 
                ttl, BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL + 1, final_dest);
        return 0;
    }
    
    /* Find nexthop from Reverse Routing Table */
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        final_dest);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("[BACKPROP] No route to dest=0x%04x, dropping", final_dest);
        return 0;
    }
    
    LOG_INF("[BACKPROP] Forwarding to dest=0x%04x via nexthop=0x%04x, ttl=%d->%d",
            final_dest, nexthop, ttl, ttl - 1);
    
    /* Prepare and send BACKPROP message to nexthop */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);
    net_buf_simple_add_le16(&msg, final_dest);
    net_buf_simple_add_u8(&msg, ttl - 1);  /* Decrement TTL */
    net_buf_simple_add_le16(&msg, payload);
    
    struct bt_mesh_msg_ctx tx_ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = nexthop,
        .send_ttl = 0,  /* Single hop - no BLE Mesh TTL */
    };
    
    int err = bt_mesh_model_send(gradient_srv->model, &tx_ctx, &msg, NULL, NULL);
    if (err) {
        LOG_ERR("[BACKPROP] Failed to forward, err=%d", err);
    }
    
    return 0;
}

/******************************************************************************/
/*                          Model Operations                                  */
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
        BT_MESH_LEN_EXACT(5),  /* [final_dest:2] + [ttl:1] + [payload:2] */
        handle_backprop_message
    },
    BT_MESH_MODEL_OP_END,
};

/******************************************************************************/
/*                          Model Callbacks                                   */
/******************************************************************************/

static int bt_mesh_gradient_srv_update_handler(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    struct net_buf_simple *buf = model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
    net_buf_simple_add_u8(buf, gradient_srv->gradient);

    LOG_INF("Auto-published gradient: %d", gradient_srv->gradient);

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
/*                          Public APIs                                       */
/******************************************************************************/

int bt_mesh_gradient_srv_gradient_send(struct bt_mesh_gradient_srv *gradient_srv)
{
    struct net_buf_simple *buf = gradient_srv->model->pub->msg;

    bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
    net_buf_simple_add_u8(buf, gradient_srv->gradient);

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
        LOG_ERR("[BACKPROP Send] Cannot send to self");
        return -EINVAL;
    }
    
    /* Find nexthop from Reverse Routing Table */
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        dest_addr);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("[BACKPROP Send] No route to dest=0x%04x", dest_addr);
        return -ENETUNREACH;
    }
    
    LOG_INF("[BACKPROP Send] Sending to dest=0x%04x via nexthop=0x%04x, ttl=%d, payload=%d",
            dest_addr, nexthop, BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL, payload);
    
    /* Prepare BACKPROP_DATA message */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);
    net_buf_simple_add_le16(&msg, dest_addr);
    net_buf_simple_add_u8(&msg, BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL);
    net_buf_simple_add_le16(&msg, payload);
    
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = nexthop,
        .send_ttl = 0,  /* Single hop - no BLE Mesh TTL */
    };
    
    return bt_mesh_model_send(gradient_srv->model, &ctx, &msg, NULL, NULL);
}

