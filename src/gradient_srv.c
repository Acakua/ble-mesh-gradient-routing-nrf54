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
#include "packet_stats.h"
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(gradient_srv, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * DEFINES
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
 * STATIC VARIABLES (SEQUENCE NUMBERS & DEDUP)
 * ------------------------------------------------------------------------- */
/* 1. Cho lệnh TEST START */
static uint8_t last_processed_test_id = 0xFF; 
static uint8_t current_tx_test_id = 0;

/* 2. Cho lệnh REPORT REQ (STOP COMMAND) */
static uint8_t last_processed_req_id = 0xFF;
static uint8_t current_tx_req_id = 0;

/* [RELATIVE LATENCY] External reference to test start time */
extern uint32_t g_test_start_time;

/* -------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * ------------------------------------------------------------------------- */

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

    LOG_DBG("[CONTROL - Gradient Beacon] Received from: 0x%04x, Gradient: %d", sender_addr, msg);

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
    
    /* Bóc tách các trường dữ liệu theo đúng thứ tự đóng gói */
    uint16_t original_source = net_buf_simple_pull_le16(buf);
    uint16_t received_data = net_buf_simple_pull_le16(buf);
    
    /* Bóc tách Hop Count (1 byte) */
    uint8_t hop_count = net_buf_simple_pull_u8(buf); 
    
    /* Bóc tách Timestamp (4 bytes) */
    uint32_t timestamp = 0;
    if (buf->len >= 4) {
        timestamp = net_buf_simple_pull_le32(buf);
    }

    /* [NEW] Bóc tách Min RSSI (1 byte) */
    int8_t path_min_rssi = (int8_t)net_buf_simple_pull_u8(buf);

    uint32_t delay_ms = 0;
    if (timestamp > 0) {
        /* [RELATIVE LATENCY]
         * now_rel = Uptime hiện tại - Uptime lúc bắt đầu test (tại Sink)
         * timestamp = Uptime lúc gửi - Uptime lúc bắt đầu test (tại Node)
         */
        uint32_t now_rel = k_uptime_get_32() - (g_test_start_time > 0 ? g_test_start_time : k_uptime_get_32());
        if (now_rel >= timestamp) {
            delay_ms = now_rel - timestamp;
        } else {
            delay_ms = 0; 
        }
    }

    /* Logging Logic */
    if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
        LOG_DBG("[CONTROL - Heartbeat] Recv from 0x%04x (via 0x%04x), Hops: %d, Delay: %u ms", 
                original_source, sender_addr, hop_count, delay_ms);
    } else {
        LOG_DBG("[DATA - Sensor] Recv from 0x%04x (via 0x%04x), Seq: %d, Hops: %d, Delay: %u ms, PathMinRSSI: %d", 
                original_source, sender_addr, received_data, hop_count, delay_ms, path_min_rssi);
    }
    
    int64_t now = k_uptime_get();
    
    k_mutex_lock(&gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    /* 1. Update Neighbor Table (Refresh timestamp) */
    bool sender_found = false;
    int sender_index = -1;
    
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            sender_found = true;
            sender_index = i;
            break;
        }
    }
    
    if (sender_found) {
        gradient_srv->forwarding_table[sender_index].last_seen = now;
        gradient_srv->forwarding_table[sender_index].rssi = rssi;
    }
    
    /* 2. Reverse Route Learning (RRT) */
    rrt_add_dest(gradient_srv->forwarding_table,
                 CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                 sender_addr,      /* nexthop */
                 original_source,  /* destination */
                 now);
    
    k_mutex_unlock(&gradient_srv->forwarding_table_mutex);

    /* ════════════════════════════════════════════════════════════════════
     * Forwarding Logic & LOGGING
     * ════════════════════════════════════════════════════════════════════ */
    
    if (gradient_srv->gradient == 0) {
        /* I AM THE SINK (Gateway) */
        led_indicate_sink_received();
        
        /* [MODIFIED] Log thêm Delay vào CSV (Chỉ log khi phiên test đang chạy) */
        if (pkt_stats_is_enabled()) {
            if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
                 /* [UPDATED] Format: TYPE, Src, Sender, Hops, DelayMs */
                 printk("CSV_LOG,HEARTBEAT,0x%04x,0x%04x,%d,%u\n", 
                        original_source, sender_addr, hop_count, delay_ms);
            } else {
                 /* [NEW] Cập nhật với RSSI chặng cuối cùng về Sink */
                 if (rssi < path_min_rssi) {
                     path_min_rssi = rssi;
                 }
                 /* [UPDATED] Format: TYPE, Src, Sender, Seq, Hops, DelayMs, PathMinRSSI */
                 printk("CSV_LOG,DATA,0x%04x,0x%04x,%d,%d,%u,%d\n", 
                        original_source, sender_addr, received_data, hop_count, delay_ms, path_min_rssi);
                if (gradient_srv->handlers->data_received) {
                    gradient_srv->handlers->data_received(gradient_srv, received_data);
                }
            }
        }
    
    } else {
        /* I AM A RELAY NODE */

        /* [MODIFIED] Check if we should forward? (Heartbeat -> NO, DATA -> YES) */
        if (received_data != BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
             /* [NEW] Cập nhật Path Min RSSI trước khi tiếp tục */
             if (rssi < path_min_rssi) {
                 path_min_rssi = rssi;
             }

             /* Forwarding Packet with timestamp and updated Min RSSI */
             int err = data_forward_send(gradient_srv, received_data, 
                                         original_source, sender_addr, 
                                         hop_count, timestamp, path_min_rssi);
             if (err) {
                 LOG_WRN("[Forward] Failed to forward seq=%d, err=%d", received_data, err);
             }
        }
    }
    
    return 0;
}

/**
 * @brief Handle BACKPROP_DATA message (Gateway -> Node downlink)
 */
static int handle_backprop_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    uint16_t sender_addr = ctx->addr;
    
    uint16_t final_dest = net_buf_simple_pull_le16(buf);
    uint8_t ttl = net_buf_simple_pull_u8(buf);
    uint16_t payload = net_buf_simple_pull_le16(buf);
    
    LOG_INF("[CONTROL - Backprop] Recv: dest=0x%04x, payload=%d, from=0x%04x", final_dest, payload, sender_addr);
    
    if (final_dest == my_addr) {
        LOG_INF("[CONTROL - Backprop] Destination Reached! Payload: %d", payload);
        led_indicate_backprop_received();
        if (gradient_srv->handlers->data_received) {
            gradient_srv->handlers->data_received(gradient_srv, payload);
        }
        return 0;
    }
    
    if (ttl <= 1) {
        return 0;
    }
    
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        final_dest);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("[CONTROL - Backprop] No route to dest=0x%04x", final_dest);
        return 0;
    }
    
    /* Forwarding */
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
    
    bt_mesh_model_send(gradient_srv->model, &tx_ctx, &msg, NULL, NULL);
    return 0;
}

/**
 * @brief [UPDATED] Handle REPORT REQUEST (Broadcast from Sink)
 * Triggered at Sensor Nodes (STOP command)
 * Logic: Receive -> Dedup -> Action -> Re-broadcast
 */
static int handle_report_req(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *srv = model->rt->user_data;
    
    /* 1. Kiểm tra Payload (1 byte ID) */
    if (buf->len < 1) {
        return -EINVAL;
    }
    uint8_t received_req_id = net_buf_simple_pull_u8(buf);

    /* 2. Duplicate Check */
    if (received_req_id == last_processed_req_id) {
        /* Đã xử lý lệnh này rồi */
        return 0;
    }

    LOG_WRN(">>> RX STOP/REPORT REQ (ID: %d) from 0x%04x <<<", received_req_id, ctx->addr);
    
    /* 3. Update State */
    last_processed_req_id = received_req_id;
    
    /* 4. Action: Notify app to stop sending */
    if (srv->handlers->report_req_received) {
        srv->handlers->report_req_received(srv);
    }
    
    /* 5. Re-broadcast (Controlled Flooding) */
    /* Chỉ re-broadcast nếu TTL còn và tôi không phải là Sink */
    if (srv->gradient != 0 && ctx->recv_ttl > 1) {
        LOG_DBG("Re-broadcasting STOP REQ ID %d...", received_req_id);

        BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, 1);
        bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ);
        net_buf_simple_add_u8(&msg, received_req_id);

        struct bt_mesh_msg_ctx send_ctx = {
            .app_idx = model->keys[0],
            .addr = BT_MESH_ADDR_ALL_NODES,
            .send_ttl = ctx->recv_ttl - 1,
        };

        (void)bt_mesh_model_send(model, &send_ctx, &msg, NULL, NULL);
    }
    
    return 0;
}

/**
 * @brief [UPDATED] Handle TEST START (Broadcast/Flood from Sink)
 * Triggered at Sensor Nodes (START command)
 * Logic: Receive -> Dedup -> Action -> Re-broadcast
 */
static int handle_test_start(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *srv = model->rt->user_data;

    /* 1. Kiểm tra độ dài và lấy Test ID */
    if (buf->len < 1) {
        return -EINVAL;
    }
    uint8_t received_test_id = net_buf_simple_pull_u8(buf);
    
    /* 2. Duplicate Check (Chống lặp) */
    if (received_test_id == last_processed_test_id) {
        /* Đã xử lý lệnh này rồi, bỏ qua */
        return 0;
    }

    LOG_INF(">>> RX TEST START (ID: %d) from 0x%04x <<<", received_test_id, ctx->addr);

    /* 3. Cập nhật trạng thái để không xử lý lại */
    last_processed_test_id = received_test_id;

    /* 4. Notify application to start sending */
    if (srv->handlers->test_start_received) {
        srv->handlers->test_start_received(srv);
    }

    /* 5. Re-broadcast (Controlled Flooding) */
    /* Điều kiện: TTL còn đủ và tôi không phải là Sink */
    if (srv->gradient != 0 && ctx->recv_ttl > 1) {
        
        LOG_DBG("Re-broadcasting TEST START ID %d...", received_test_id);

        BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TEST_START, 1);
        bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TEST_START);
        net_buf_simple_add_u8(&msg, received_test_id);

        struct bt_mesh_msg_ctx send_ctx = {
            .app_idx = model->keys[0],
            .addr = BT_MESH_ADDR_ALL_NODES, // Gửi cho tất cả hàng xóm
            .send_ttl = ctx->recv_ttl - 1,  // Giảm TTL đi 1
        };

        (void)bt_mesh_model_send(model, &send_ctx, &msg, NULL, NULL);
    }
    
    return 0;
}

/**
 * @brief [NEW] Handle REPORT RESPONSE (Unicast from Node)
 * Triggered at Sink Node
 */
/* FORWARD DECLARATION for public API if needed, but we can access srv directly if passed */

static void report_retry_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct bt_mesh_gradient_srv *srv = CONTAINER_OF(dwork, struct bt_mesh_gradient_srv, report_retry_work);

    if (!srv->is_report_pending) return;

    if (srv->report_retry_count >= REPORT_MAX_RETRIES) {
        LOG_ERR("Max retries for REPORT reached. Giving up.");
        srv->is_report_pending = false;
        return;
    }

    srv->report_retry_count++;
    LOG_WRN("Resending REPORT_RSP (Retry %u)...", srv->report_retry_count);
    
    /* Re-send Report */
    struct packet_stats stats;
    pkt_stats_get(&stats);

    /* [MODIFIED] Payload size 12 bytes */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP, 12);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP);
    
    /* Gắn thêm địa chỉ nguồn gốc (Reporter) vào payload để Relay/Sink nhận diện đúng */
    uint16_t my_addr = bt_mesh_model_elem(srv->model)->rt->addr;
    net_buf_simple_add_le16(&msg, my_addr);
    
    net_buf_simple_add_le16(&msg, (uint16_t)stats.data_tx);
    net_buf_simple_add_le16(&msg, (uint16_t)stats.gradient_beacon_tx);
    net_buf_simple_add_le16(&msg, (uint16_t)stats.heartbeat_tx);
    net_buf_simple_add_le16(&msg, (uint16_t)stats.route_change_count);
    net_buf_simple_add_le16(&msg, (uint16_t)stats.data_fwd_tx);

    /* [DYNAMIC PARENT SWITCHING] 
     * Thay đổi Parent dựa trên số lần Retry để tránh bị kẹt tại một Relay chết.
     */
    uint16_t target_parent = BT_MESH_ADDR_UNASSIGNED;
    int parent_idx = 0;

    // Retry 0-2: Dùng Best Parent (Index 0)
    // Retry 3-5: Dùng 2nd Best Parent (Index 1) nếu có
    // Retry 6+:  Quét tìm bất kỳ Parent nào khả thi
    if (srv->report_retry_count >= 6) {
        // Fallback mode: Tìm bất kỳ ai là Parent
        for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
            if (srv->forwarding_table[i].gradient < srv->gradient &&
                srv->forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
                target_parent = srv->forwarding_table[i].addr;
                parent_idx = i;
                // Ưu tiên đổi khác cái cũ một chút bằng cách lấy modulo
                if (i >= (srv->report_retry_count % 3)) break; 
            }
        }
    } else if (srv->report_retry_count >= 3) {
        // Try 2nd best parent
        if (srv->forwarding_table[1].gradient < srv->gradient && 
            srv->forwarding_table[1].addr != BT_MESH_ADDR_UNASSIGNED) {
            target_parent = srv->forwarding_table[1].addr;
            parent_idx = 1;
        }
    }

    // Nếu không tìm được candidate nào khác, quay về Best Parent
    if (target_parent == BT_MESH_ADDR_UNASSIGNED) {
        target_parent = srv->forwarding_table[0].addr;
        parent_idx = 0;
    }

    struct bt_mesh_msg_ctx ctx = {
        .app_idx = srv->model->keys[0],
        .addr = target_parent,
        .send_ttl = BT_MESH_TTL_DEFAULT,
        .send_rel = true, // BẮT BUỘC có ACK
    };
    
    if (target_parent != BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("Resending REPORT_RSP (Retry %u) to Parent [%d]: 0x%04x", 
                srv->report_retry_count, parent_idx, target_parent);
        bt_mesh_model_send(srv->model, &ctx, &msg, NULL, NULL);
    } else {
        LOG_ERR("Retry %u: No parent to send report!", srv->report_retry_count);
    }

    // Schedule next retry with Jitter
    uint32_t jitter = sys_rand32_get() % 1000;
    k_work_schedule(&srv->report_retry_work, K_MSEC(REPORT_RETRY_TIMEOUT_MS + jitter));
}

static int handle_report_ack(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *srv = model->rt->user_data;
    uint16_t target_addr = net_buf_simple_pull_le16(buf); 
    uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

    if (target_addr == my_addr) {
        LOG_INF("Received REPORT_ACK! Stopping retry.");
        k_work_cancel_delayable(&srv->report_retry_work);
        srv->is_report_pending = false;
        return 0;
    }

    /* Forwarding Logic using RRT */
    uint16_t nexthop = rrt_find_nexthop(srv->forwarding_table, 
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        target_addr);
                                        
    if (nexthop != BT_MESH_ADDR_UNASSIGNED) {
        BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, 2);
        bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK);
        net_buf_simple_add_le16(&msg, target_addr);

        struct bt_mesh_msg_ctx new_ctx = {
            .app_idx = model->keys[0],
            .addr = nexthop,
            .send_ttl = BT_MESH_TTL_DEFAULT,
        };
        
        LOG_DBG("Forwarding REPORT_ACK for 0x%04x to Nexthop 0x%04x", target_addr, nexthop);
        bt_mesh_model_send(model, &new_ctx, &msg, NULL, NULL);
    } else {
        LOG_WRN("No route to forward REPORT_ACK for 0x%04x", target_addr);
    }

    return 0;
}

static int handle_report_rsp(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    /* Kiểm tra chiều dài payload (12 bytes: ReporterAddr(2) + 5 stats * 2) */
    if (buf->len < 12) return -EINVAL;

    uint16_t reporter_addr = net_buf_simple_pull_le16(buf);
    uint16_t data_tx = net_buf_simple_pull_le16(buf);
    uint16_t beacon_tx = net_buf_simple_pull_le16(buf);
    uint16_t hb_tx = net_buf_simple_pull_le16(buf);
    uint16_t route_changes = net_buf_simple_pull_le16(buf);
    uint16_t data_fwd = net_buf_simple_pull_le16(buf);
    
    struct bt_mesh_gradient_srv *srv = model->rt->user_data; 
    
    /* [LOGIC PHÂN NHÁNH: SINK vs RELAY] */
    if (srv->gradient == 0) {
        /* [1. I AM SINK] - Log dữ liệu và gửi ACK về nguồn gốc thực sự */
        LOG_INF("SINK received REPORT from 0x%04x (Forwarded by 0x%04x)", 
                reporter_addr, ctx->addr);
        
        // Format log cho Python script (Dùng reporter_addr để định danh đúng node)
        // [NEW] Format: TYPE, Src, Tx, Beacon, HB, RouteChanges, FwdCount
        printk("CSV_LOG,REPORT,0x%04x,%u,%u,%u,%u,%u\n", 
               reporter_addr, data_tx, beacon_tx, hb_tx, route_changes, data_fwd);
               
        /* SEND ACK via Reverse Routing tới Reporter gốc */
        uint16_t nexthop = rrt_find_nexthop(srv->forwarding_table, 
                                            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                            reporter_addr);
                                            
        if (nexthop != BT_MESH_ADDR_UNASSIGNED) {
            BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, 2);
            bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK);
            net_buf_simple_add_le16(&msg, reporter_addr);

            struct bt_mesh_msg_ctx ack_ctx = {
                .app_idx = model->keys[0],
                .addr = nexthop,
                .send_ttl = BT_MESH_TTL_DEFAULT,
            };
            bt_mesh_model_send(model, &ack_ctx, &msg, NULL, NULL);
            LOG_DBG("Sink sent REPORT_ACK for 0x%04x via Nexthop 0x%04x", reporter_addr, nexthop);
        } else {
            LOG_WRN("Sink cannot find RRT route for ACK to 0x%04x (Source: 0x%04x)", 
                    reporter_addr, ctx->addr);
        }
    } else {
        /* [2. I AM RELAY] - Chuyển tiếp bản báo cáo lên CHA */
        const neighbor_entry_t *best_parent = find_strict_upstream_parent(srv, reporter_addr);

        if (best_parent != NULL) {
            LOG_INF("Relaying REPORT from 0x%04x to Parent 0x%04x", reporter_addr, best_parent->addr);

            BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP, 12);
            bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP);
            net_buf_simple_add_le16(&msg, reporter_addr); // RE-PACK ORIGINAL REPORTER
            net_buf_simple_add_le16(&msg, data_tx);
            net_buf_simple_add_le16(&msg, beacon_tx);
            net_buf_simple_add_le16(&msg, hb_tx);
            net_buf_simple_add_le16(&msg, route_changes);
            net_buf_simple_add_le16(&msg, data_fwd);

            struct bt_mesh_msg_ctx fwd_ctx = {
                .app_idx = model->keys[0],
                .addr = best_parent->addr,
                .send_ttl = BT_MESH_TTL_DEFAULT,
                .send_rel = true, 
            };
            
            bt_mesh_model_send(model, &fwd_ctx, &msg, NULL, NULL);
        } else {
            LOG_ERR("Relay 0x%04x has no PARENT to forward report from 0x%04x!", 
                    bt_mesh_model_elem(model)->rt->addr, reporter_addr);
        }
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
    /* [UPDATED] Report Req expects 1 byte ID now */
    { 
        BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, 
        BT_MESH_LEN_MIN(1), 
        handle_report_req 
    },
    { 
        BT_MESH_GRADIENT_SRV_OP_REPORT_RSP, 
        BT_MESH_LEN_EXACT(12), 
        handle_report_rsp 
    },
    { 
        BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, 
        BT_MESH_LEN_EXACT(2), 
        handle_report_ack 
    },
    { 
        BT_MESH_GRADIENT_SRV_OP_TEST_START, 
        BT_MESH_LEN_MIN(1), /* Expect 1 byte payload (Test ID) */
        handle_test_start 
    },
    BT_MESH_MODEL_OP_END,
};

/******************************************************************************/
/* Model Callbacks & Init                                                     */
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
                      const char *name, size_t len_rd,
                      settings_read_cb read_cb, void *cb_arg)
{
    return 0;
}
#endif

static int bt_mesh_gradient_srv_init(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
    gradient_srv->model = model;
    
    /* Init Reliable Report Context */
    k_work_init_delayable(&gradient_srv->report_retry_work, report_retry_handler);
    gradient_srv->is_report_pending = false;
    gradient_srv->report_retry_count = 0;
    
    net_buf_simple_init_with_data(&gradient_srv->pub_msg, gradient_srv->buf,
                      sizeof(gradient_srv->buf));
    gradient_srv->pub.msg = &gradient_srv->pub_msg;
    gradient_srv->pub.update = bt_mesh_gradient_srv_update_handler;

    k_mutex_init(&gradient_srv->forwarding_table_mutex);

    led_indication_init();
    data_forward_init();
    gradient_work_init();
    pkt_stats_init();

    return 0;
}

static int bt_mesh_gradient_srv_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    if (gradient_srv->handlers->start) {
        gradient_srv->handlers->start(gradient_srv);
    }
    
    gradient_work_set_srv(gradient_srv);
    
    if (model->pub) {
        model->pub->addr = BT_MESH_ADDR_ALL_NODES;
        model->pub->ttl = 0;
        model->pub->period = BT_MESH_PUB_PERIOD_SEC(5);
        gradient_work_schedule_initial_publish();
    }
    
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
    return bt_mesh_model_publish(gradient_srv->model);
}

int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
                                   uint16_t addr, uint16_t data, uint32_t timestamp, int8_t initial_rssi)
{
    return data_forward_send_direct(gradient_srv, addr, data, timestamp, initial_rssi);
}

int bt_mesh_gradient_srv_backprop_send(struct bt_mesh_gradient_srv *gradient_srv,
                                       uint16_t dest_addr, uint16_t payload)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    
    if (dest_addr == my_addr) return -EINVAL;
    
    uint16_t nexthop = rrt_find_nexthop(gradient_srv->forwarding_table,
                                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                                        dest_addr);
    
    if (nexthop == BT_MESH_ADDR_UNASSIGNED) return -ENETUNREACH;
    
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

/**
 * @brief [UPDATED] Gửi lệnh yêu cầu báo cáo xuống toàn mạng (Controlled Flooding)
 */
int bt_mesh_gradient_srv_send_report_req(struct bt_mesh_gradient_srv *gradient_srv, bool force_new_id)
{
    /* 1. Tăng ID cho lần gửi mới nếu được yêu cầu */
    if (force_new_id) {
        current_tx_req_id++;
    }

    /* 2. Đóng gói ID (1 byte) vào payload */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, 1);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ);
    net_buf_simple_add_u8(&msg, current_tx_req_id);

    /* 3. Gửi Broadcast với TTL mặc định */
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    LOG_INF(">>> BROADCASTING REPORT REQUEST (ID: %d) <<<", current_tx_req_id);
    return bt_mesh_model_send(gradient_srv->model, &ctx, &msg, NULL, NULL);
}

/**
 * @brief [UPDATED] Gửi lệnh TEST START xuống toàn mạng (Controlled Flooding)
 */
int bt_mesh_gradient_srv_send_test_start(struct bt_mesh_gradient_srv *gradient_srv, bool force_new_id)
{
    /* 1. Tăng ID cho lần gửi mới nếu được yêu cầu */
    if (force_new_id) {
        current_tx_test_id++;
    }

    /* 2. Tạo gói tin có payload là Test ID (1 byte) */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TEST_START, 1);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TEST_START);
    net_buf_simple_add_u8(&msg, current_tx_test_id);

    /* 3. Gửi Broadcast với TTL mặc định */
    struct bt_mesh_msg_ctx ctx = {
        .app_idx = gradient_srv->model->keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    LOG_INF(">>> BROADCASTING TEST START (ID: %d) <<<", current_tx_test_id);
    return bt_mesh_model_send(gradient_srv->model, &ctx, &msg, NULL, NULL);
}

/**
 * @brief [NEW] Gửi phản hồi báo cáo về Sink (Unicast)
 */
int bt_mesh_gradient_srv_report_rsp_send(struct bt_mesh_gradient_srv *gradient_srv)
{
    LOG_INF("Starting Reliable Report Sequence...");
    
    gradient_srv->is_report_pending = true;
    gradient_srv->report_retry_count = 0;
    
    /* [FIX CONGESTION] Thêm Delay ngẫu nhiên ban đầu (0-5s) cho cấu hình 40 node 
     * để tránh việc hàng chục node cùng ập vào Sink một lúc gây nghẽn.
     */
    uint32_t initial_jitter = sys_rand32_get() % 5000;
    k_work_schedule(&gradient_srv->report_retry_work, K_MSEC(initial_jitter));
    
    return 0;
}