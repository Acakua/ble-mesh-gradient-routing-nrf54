/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "data_forward.h"
#include "neighbor_table.h"
#include "led_indication.h"
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

/* Forward declaration */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, uint16_t data);

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
        LOG_INF("[TX Complete] SUCCESS sent to 0x%04x", dest_addr);
    }
    
    /* Always clear active flag */
    ctx->active = false;
}

static const struct bt_mesh_send_cb data_send_cb = {
    .start = NULL,
    .end = data_send_end_cb,
};

static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, uint16_t data)
{
    struct bt_mesh_msg_ctx ctx = {
        .addr = addr,
        .app_idx = gradient_srv->model->keys[0],
        .send_ttl = 0,
        .send_rel = true,
    };

    BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE, 4);
    bt_mesh_model_msg_init(&buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE);
    net_buf_simple_add_le16(&buf, original_source);
    net_buf_simple_add_le16(&buf, data);
    
    LOG_DBG("[TX] Sending to 0x%04x: original_src=0x%04x, Seq/Data=%d", 
            addr, original_source, data);
    
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

int data_forward_send(struct bt_mesh_gradient_srv *gradient_srv,
                      uint16_t data, uint16_t original_source, uint16_t sender_addr)
{
    /* FIX: Busy check */
    if (data_send_ctx.active) {
        LOG_WRN("[Forward] System busy, dropping packet %d", data);
        return -EBUSY;
    }

    /* FIX: Use Strict Parent Search instead of blind nt_best/iteration */
    const neighbor_entry_t *best_parent = find_strict_upstream_parent(gradient_srv, sender_addr);

    if (best_parent == NULL) {
        LOG_ERR("[Forward] DROP! No valid PARENT found (neighbors have >= gradient %d)", 
                gradient_srv->gradient);
        /* Returning error stops the loop. Packet dies here. */
        return -ENETUNREACH;
    }

    led_indicate_data_forwarded();
    
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = original_source;
    data_send_ctx.target_addr = best_parent->addr;
    data_send_ctx.active = true;
    
    LOG_INF("[Forward] Selected PARENT 0x%04x (Grad: %d) for forwarding Seq: %d", 
            best_parent->addr, best_parent->gradient, data);
    
    int err = data_send_internal(gradient_srv, best_parent->addr, original_source, data);
    if (err) {
        data_send_ctx.active = false;
        LOG_ERR("[Forward] TX failed start, err=%d", err);
        return err;
    }
    return 0;
}

int data_forward_send_direct(struct bt_mesh_gradient_srv *gradient_srv,
                             uint16_t addr, uint16_t data)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    
    /* FIX: Busy check */
    if (data_send_ctx.active) {
        LOG_WRN("[Direct] System busy, dropping packet %d", data);
        return -EBUSY;
    }

    /* FIX: Even for direct send (Heartbeat), strictly use Upstream Parent */
    /* Ignore 'addr' parameter as this is for Uplink Data */
    const neighbor_entry_t *best_parent = find_strict_upstream_parent(gradient_srv, BT_MESH_ADDR_UNASSIGNED);
    
    if (best_parent == NULL) {
        LOG_WRN("[Direct] No Uplink Route! (Gradient %d, no lower neighbor)", 
                gradient_srv->gradient);
        return -ENETUNREACH;
    }
    
    uint16_t nexthop = best_parent->addr;
    
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = my_addr;
    data_send_ctx.target_addr = nexthop;
    data_send_ctx.active = true;
    
    LOG_INF("[Direct] Sending Seq: %d to PARENT 0x%04x (Grad: %d)", 
            data, nexthop, best_parent->gradient);
    
    int err = data_send_internal(gradient_srv, nexthop, my_addr, data);
    if (err) {
        data_send_ctx.active = false;
        return err;
    }
    return 0;
}