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
#define DATA_RETRY_DELAY_MS  100  /* 100ms - delay before retrying next entry */

/* Work item for data send retry */
static struct k_work_delayable data_retry_work;

/* Context for data send with retry */
struct data_send_context {
    struct bt_mesh_gradient_srv *gradient_srv;
    uint16_t data;
    uint16_t original_source;  /* Original source address (node that created packet) */
    uint16_t sender_addr;      /* Immediate sender address (to skip when forwarding) */
    int current_index;         /* Current index in forwarding table */
    bool active;               /* Has pending send */
};

static struct data_send_context data_send_ctx = {0};

/* Forward declaration */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, uint16_t data);

/**
 * @brief Callback when data transmission completes
 *
 * Handles TX result and schedules retry if failed.
 *
 * @param err Error code (0 = success)
 * @param user_data Destination address cast to pointer
 */
static void data_send_end_cb(int err, void *user_data)
{
    uint16_t dest_addr = (uint16_t)(uintptr_t)user_data;
    struct data_send_context *ctx = &data_send_ctx;
    
    if (err) {
        LOG_ERR("[TX Complete] FAILED to send to 0x%04x, err=%d", dest_addr, err);
        
        if (ctx->active && ctx->gradient_srv) {
            LOG_WRN("[TX Complete] Send to 0x%04x failed, trying next entry...", dest_addr);
            k_work_schedule(&data_retry_work, K_MSEC(DATA_RETRY_DELAY_MS));
        }
    } else {
        LOG_INF("[TX Complete] SUCCESS sent to 0x%04x", dest_addr);
        ctx->active = false;
    }
}

static const struct bt_mesh_send_cb data_send_cb = {
    .start = NULL,
    .end = data_send_end_cb,
};

/**
 * @brief Internal function to send data message
 *
 * Sends data packet to specified address with TX callback.
 * Packet format: [original_source: 2 bytes] + [data: 2 bytes]
 *
 * @param gradient_srv Pointer to gradient server instance
 * @param addr Destination mesh address
 * @param original_source Address of node that originally created the packet
 * @param data Data payload to send
 *
 * @return 0 on success, negative error code on failure
 */
static int data_send_internal(struct bt_mesh_gradient_srv *gradient_srv,
                              uint16_t addr, uint16_t original_source, uint16_t data)
{
    struct bt_mesh_msg_ctx ctx = {
        .addr = addr,
        .app_idx = gradient_srv->model->keys[0],
        .send_ttl = 0,
        .send_rel = true,
    };

    /* Packet format: [original_source: 2 bytes] + [data: 2 bytes] = 4 bytes */
    BT_MESH_MODEL_BUF_DEFINE(buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE, 4);
    bt_mesh_model_msg_init(&buf, BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE);
    net_buf_simple_add_le16(&buf, original_source);  /* Original source first */
    net_buf_simple_add_le16(&buf, data);              /* Then data */
    
    LOG_DBG("[TX] Sending to 0x%04x: original_src=0x%04x, data=%d", 
            addr, original_source, data);
    
    return bt_mesh_model_send(gradient_srv->model, &ctx, &buf, 
                              &data_send_cb, 
                              (void *)(uintptr_t)addr);
}

/**
 * @brief Work handler for data send retry
 *
 * Attempts to send data to next entry in forwarding table after failure.
 *
 * @param work Pointer to work item (unused)
 */
static void data_retry_handler(struct k_work *work)
{
    struct data_send_context *ctx = &data_send_ctx;
    
    if (!ctx->active || !ctx->gradient_srv) {
        ctx->active = false;
        return;
    }
    
    struct bt_mesh_gradient_srv *gradient_srv = ctx->gradient_srv;
    
    ctx->current_index++;
    
    while (ctx->current_index < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            ctx->current_index);
        
        if (entry == NULL) {
            LOG_ERR("[Retry] No more valid entries, data lost!");
            ctx->active = false;
            return;
        }
        
        uint16_t next_addr = entry->addr;
        
        if (next_addr == ctx->sender_addr) {
            LOG_DBG("[Retry] Skipping sender 0x%04x", next_addr);
            ctx->current_index++;
            continue;
        }
        
        LOG_INF("[Retry] Trying index %d: addr=0x%04x", ctx->current_index, next_addr);
        
        int err = data_send_internal(gradient_srv, next_addr, ctx->original_source, ctx->data);
        
        if (err) {
            LOG_ERR("[Retry] Failed to queue send to 0x%04x, err=%d", next_addr, err);
            ctx->current_index++;
            continue;
        }
        
        return;
    }
    
    LOG_ERR("[Retry] All entries exhausted, data lost!");
    ctx->active = false;
}

void data_forward_init(void)
{
    k_work_init_delayable(&data_retry_work, data_retry_handler);
}

int data_forward_send(struct bt_mesh_gradient_srv *gradient_srv,
                      uint16_t data, uint16_t original_source, uint16_t sender_addr)
{
    /* Check if route is available */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)gradient_srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);
    
    if (best == NULL) {
        LOG_ERR("[Forward] No route available!");
        return -ENETUNREACH;
    }
    
    /* Indicate data forwarding */
    led_indicate_data_forwarded();
    
    /* Setup context for retry */
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = original_source;
    data_send_ctx.sender_addr = sender_addr;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;
    
    /* Find first valid destination (skip sender) */
    while (data_send_ctx.current_index < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            data_send_ctx.current_index);
        
        if (entry == NULL) {
            LOG_ERR("[Forward] No valid destination!");
            data_send_ctx.active = false;
            return -ENETUNREACH;
        }
        
        uint16_t dest_addr = entry->addr;
        
        if (dest_addr == sender_addr) {
            data_send_ctx.current_index++;
            continue;
        }
        
        LOG_INF("[Forward] Forwarding: original_src=0x%04x, data=%d, to=0x%04x (index %d)", 
                original_source, data, dest_addr, data_send_ctx.current_index);
        
        int err = data_send_internal(gradient_srv, dest_addr, original_source, data);
        
        if (err) {
            LOG_ERR("[Forward] Failed to queue, err=%d", err);
            data_send_ctx.current_index++;
            continue;
        }
        
        return 0;
    }
    
    LOG_ERR("[Forward] No valid destination after filtering!");
    data_send_ctx.active = false;
    return -ENETUNREACH;
}

int data_forward_send_direct(struct bt_mesh_gradient_srv *gradient_srv,
                             uint16_t addr, uint16_t data)
{
    /* Get my own address as original_source (I am creating this packet) */
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
    
    data_send_ctx.gradient_srv = gradient_srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = my_addr;
    data_send_ctx.sender_addr = BT_MESH_ADDR_UNASSIGNED;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;
    
    LOG_INF("[Direct] Sending data %d with original_src=0x%04x to 0x%04x", 
            data, my_addr, addr);
    
    return data_send_internal(gradient_srv, addr, my_addr, data);
}
