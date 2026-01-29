/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "data_forward.h"
#include "neighbor_table.h"
#include "led_indication.h"
#include "packet_stats.h"
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_forward, LOG_LEVEL_DBG);

/* Timing constants */
#define DATA_RETRY_DELAY_MS  100  /* 100ms */

/* Work item for data send retry */
static struct k_work_delayable data_retry_work;

/* Context for data send */
struct data_send_context {
    struct bt_mesh_gradient_srv *gradient_srv;
    uint16_t data;
    uint16_t original_source;
    uint16_t target_addr;     /* The specific chosen parent address */
    bool active;
};

static struct data_send_context data_send_ctx = {0};
static uint16_t last_parent_addr = BT_MESH_ADDR_UNASSIGNED;

/* Forward declaration */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, 
                              uint16_t data, uint8_t hop_count,
                              uint32_t timestamp);

/**
 * @brief Find the BEST Parent strictly for Uplink Routing
 * * [CHANGED]: Removed 'static' keyword so gradient_srv.c can use this function
 * to route REPORT_RSP packets.
 * * Scans the entire table to find a neighbor with:
 * 1. Gradient < My Gradient (CRITICAL CONDITION)
 * 2. Best Gradient among valid candidates
 * 3. Best RSSI among ties
 * * @param srv Pointer to gradient server
 * @param exclude_addr Address to exclude (e.g., the sender)
 * @return Pointer to best entry, or NULL if no VALID PARENT found.
 */
const neighbor_entry_t *find_strict_upstream_parent(
    struct bt_mesh_gradient_srv *srv, uint16_t exclude_addr)
{
    const neighbor_entry_t *best_candidate = NULL;
    uint8_t my_gradient = srv->gradient;

    /* If I am uninitialized, I cannot route properly */
    if (my_gradient == UINT8_MAX) {
        return NULL;
    }

    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            i);

        if (!entry) continue;
        if (entry->addr == exclude_addr) continue;

        /* --- THE LAW: STRICT UPLINK RULE --- */
        /* Only consider neighbors strictly closer to Gateway */
        if (entry->gradient >= my_gradient) {
            continue; /* Skip children, siblings, and bad paths */
        }

        /* Logic to pick the best among valid parents */
        if (best_candidate == NULL) {
            best_candidate = entry;
        } else {
            /* Compare with current best candidate */
            /* Prioritize Lower Gradient */
            if (entry->gradient < best_candidate->gradient) {
                best_candidate = entry;
            }
            /* Tie-break with RSSI */
            else if (entry->gradient == best_candidate->gradient) {
                if (entry->rssi > best_candidate->rssi) {
                    best_candidate = entry;
                }
            }
        }
    }

    return best_candidate;
}

static void data_send_end_cb(int err, void *user_data)
{
    uint16_t dest_addr = (uint16_t)(uintptr_t)user_data;
    struct data_send_context *ctx = &data_send_ctx;
    
    if (err) {
        LOG_ERR("[TX Complete] FAILED to send to 0x%04x, err=%d", dest_addr, err);
        /* In this strict version, we do NOT blindly retry other nodes.
           Retrying blindly causes loops. We just fail. 
           Reliability is handled by upper layers or next periodic send. */
    } else {
        pkt_stats_inc_data_tx();
        LOG_INF("[TX Complete] SUCCESS sent to 0x%04x", dest_addr);
    }
    
    /* Always clear active flag */
    ctx->active = false;
}

static const struct bt_mesh_send_cb data_send_cb = {
    .start = NULL,
    .end = data_send_end_cb,
};

/* Forward declaration */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, 
                              uint16_t data, uint8_t hop_count,
                              uint32_t timestamp);
/* ... */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, 
                              uint16_t data, uint8_t hop_count,
                              uint32_t timestamp)
{
    struct bt_mesh_msg_ctx ctx = {
        .addr = addr,
        .app_idx = gradient_srv->model->keys[0],
        .send_ttl = 0,
        .send_rel = true,
    };

    /* [MODIFIED] Tăng kích thước buffer từ 5 lên 9 bytes (thêm 4 byte timestamp) */
    BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE, 9);
    bt_mesh_model_msg_init(&buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE);
    
    net_buf_simple_add_le16(&buf, original_source);
    net_buf_simple_add_le16(&buf, data);
    net_buf_simple_add_u8(&buf, hop_count);
    
    /* [RELATIVE TIMstamp] Gửi thời gian tương đối tính từ lúc bắt đầu test */
    if (timestamp == 0) {
        extern uint32_t g_test_start_time;
        timestamp = k_uptime_get_32() - (g_test_start_time > 0 ? g_test_start_time : k_uptime_get_32());
    }
    net_buf_simple_add_le32(&buf, timestamp);
    
    LOG_DBG("[TX] To 0x%04x: Src=0x%04x, Seq=%d, Hops=%d, TS=%u", 
            addr, original_source, data, hop_count, timestamp);
    
    return bt_mesh_model_send(gradient_srv->model, &ctx, &buf, 
                              &data_send_cb, 
                              (void *)(uintptr_t)addr);
}

/* Handler Unused in Strict Mode but kept for compilation compatibility */
static void data_retry_handler(struct k_work *work)
{
    data_send_ctx.active = false;
}

void data_forward_init(void)
{
    k_work_init_delayable(&data_retry_work, data_retry_handler);
}

/**
 * @brief Forward data to the best upstream parent and increment hop count.
 * * @param hop_count_received The hop count value pulled from the incoming packet.
 */
int data_forward_send(struct bt_mesh_gradient_srv *gradient_srv,
                      uint16_t data, uint16_t original_source, 
                      uint16_t sender_addr, uint8_t hop_count_received,
                      uint32_t timestamp)
{
    /* FIX: Busy check */
    if (data_send_ctx.active) {
        LOG_WRN("[Forward] System busy, dropping packet %d", data);
        return -EBUSY;
    }

    /* Logic: Tìm cha tốt nhất theo hướng Uplink (về Sink) */
    const neighbor_entry_t *best_parent = find_strict_upstream_parent(gradient_srv, sender_addr);

    if (best_parent == NULL) {
        LOG_ERR("[Forward] DROP! No valid PARENT found (neighbors have >= gradient %d)", 
                gradient_srv->gradient);
        return -ENETUNREACH;
    }

    /* Logic cộng dồn Hop Count */
    uint8_t next_hop_count = hop_count_received + 1;

    led_indicate_data_forwarded();
    
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = original_source;
    data_send_ctx.target_addr = best_parent->addr;
    data_send_ctx.active = true;
    
    LOG_INF("[Forward] Relay via 0x%04x (Grad: %d) Seq: %d, Hops: %d -> %d", 
            best_parent->addr, best_parent->gradient, data, 
            hop_count_received, next_hop_count);
    
    /* [NEW] Route Change Detection */
    if (last_parent_addr != BT_MESH_ADDR_UNASSIGNED && 
        last_parent_addr != best_parent->addr) {
        pkt_stats_inc_route_change();
        LOG_INF("[METRIC] Route Changed: 0x%04x -> 0x%04x", last_parent_addr, best_parent->addr);
    }
    last_parent_addr = best_parent->addr;
    
    /* Gửi đi với giá trị hop_count mới và timestamp gốc */
    int err = data_send_internal(gradient_srv, best_parent->addr, original_source, 
                                 data, next_hop_count, timestamp);

    if (err) {
        data_send_ctx.active = false;
        LOG_ERR("[Forward] TX failed start, err=%d", err);
        return err;
    }

    return 0;
}

/**
 * @brief Send data directly from this node (Source Node) to the best parent.
 * Initializes Hop Count to 1.
 */
int data_forward_send_direct(struct bt_mesh_gradient_srv *gradient_srv,
                             uint16_t addr, uint16_t data, uint32_t timestamp)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    
    /* FIX: Busy check */
    if (data_send_ctx.active) {
        LOG_WRN("[Direct] System busy, dropping packet %d", data);
        return -EBUSY;
    }

    /* FIX: Even for direct send (Heartbeat/Data), strictly use Upstream Parent */
    /* Ignore 'addr' parameter as this is for Uplink Data */
    const neighbor_entry_t *best_parent = find_strict_upstream_parent(gradient_srv, BT_MESH_ADDR_UNASSIGNED);
    
    if (best_parent == NULL) {
        LOG_WRN("[Direct] No Uplink Route! (Gradient %d, no lower neighbor)", 
                gradient_srv->gradient);
        return -ENETUNREACH;
    }
    
    uint16_t nexthop = best_parent->addr;
    
    /* [NEW] Khởi tạo Hop Count = 1 cho gói tin gốc */
    uint8_t initial_hop_count = 1;
    
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = my_addr;
    data_send_ctx.target_addr = nexthop;
    data_send_ctx.active = true;
    
    LOG_INF("[Direct] Sending Seq: %d to PARENT 0x%04x (Hops: 1)", 
            data, nexthop);
    
    /* [NEW] Route Change Detection */
    if (last_parent_addr != BT_MESH_ADDR_UNASSIGNED && 
        last_parent_addr != nexthop) {
        pkt_stats_inc_route_change();
        LOG_INF("[METRIC] Route Changed: 0x%04x -> 0x%04x", last_parent_addr, nexthop);
    }
    last_parent_addr = nexthop;
    
    /* [MODIFIED] Truyền thêm tham số initial_hop_count vào hàm gửi nội bộ */
    /* [MODIFIED] Pass the provided timestamp to data_send_internal */
    int err = data_send_internal(gradient_srv, nexthop, my_addr, data, initial_hop_count, timestamp);
    
    if (err) {
        data_send_ctx.active = false;
        LOG_ERR("[Direct] TX failed start, err=%d", err);
        return err;
    }
    
    return 0;
}