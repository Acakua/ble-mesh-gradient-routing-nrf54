/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "gradient_work.h"
#include "neighbor_table.h"
#include "routing_policy.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gradient_work, LOG_LEVEL_DBG);

/* Timing constants */
#define CLEANUP_INTERVAL_MS       15000  /* 15s - interval between cleanup checks */
#define INITIAL_PUBLISH_DELAY_MS  500    /* 500ms - delay before first gradient publish */

/* Global gradient server reference */
static struct bt_mesh_gradient_srv *g_gradient_srv = NULL;

/* Work items */
static struct k_work_delayable initial_publish_work;
static struct k_work_delayable gradient_process_work;
static struct k_work_delayable cleanup_work;

/* Context for gradient processing */
struct gradient_context {
    struct bt_mesh_gradient_srv *gradient_srv;
    uint8_t gradient_msg;
    uint16_t sender_addr;
    int8_t rssi;
};

static struct gradient_context gradient_ctx = {0};

/* Forward declaration */
int bt_mesh_gradient_srv_gradient_send(struct bt_mesh_gradient_srv *gradient_srv);

/**
 * @brief Work handler for initial gradient publish
 *
 * Publishes the first gradient beacon after node startup.
 *
 * @param work Pointer to work item (unused)
 */
static void initial_publish_handler(struct k_work *work)
{
    if (g_gradient_srv != NULL) {
        int err = bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
        
        if (err) {
            LOG_INF("Initial publish failed: %d", err);
        } else {
            LOG_INF("Initial gradient published: %d", g_gradient_srv->gradient);
        }
    }
}

/**
 * @brief Work handler for periodic neighbor cleanup
 *
 * Removes expired neighbors from forwarding table and updates gradient if needed.
 *
 * @param work Pointer to work item (unused)
 */
static void cleanup_handler(struct k_work *work)
{
    if (g_gradient_srv == NULL) {
        return;
    }
    
    int64_t current_time = k_uptime_get();
    bool table_changed = false;
    
    LOG_DBG("[Cleanup] Running cleanup check...");
    
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            i);
        
        if (entry == NULL) {
            continue;
        }
        
        if (nt_is_expired(
                (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i, current_time, CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS)) {
            
            LOG_WRN("[Cleanup] Node 0x%04x expired (last seen %lld ms ago)",
                    entry->addr, current_time - entry->last_seen);
            
            uint16_t removed_addr = nt_remove(
                (neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            if (removed_addr != GR_ADDR_UNASSIGNED) {
                LOG_INF("[Forwarding] Removed 0x%04x from index %d", removed_addr, i);
                table_changed = true;
                i--;  /* Re-check this index since entries shifted */
            }
        }
    }

    /* Update gradient based on best remaining parent */
    if (table_changed) {
        const neighbor_entry_t *best = nt_best(
            (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

        if (best != NULL) {
            uint8_t best_parent_gradient = best->gradient;

            if (best_parent_gradient != UINT8_MAX) {
                uint8_t old_gradient = g_gradient_srv->gradient;
                g_gradient_srv->gradient = rp_compute_new_gradient(best_parent_gradient);

                LOG_INF("[Process] Gradient updated: [%d] -> [%d]",
                    old_gradient, g_gradient_srv->gradient);

                bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
            }
        } else {
            LOG_WRN("[Cleanup] No parents available, resetting gradient to 255");
            g_gradient_srv->gradient = UINT8_MAX;
            bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
        }
        
        LOG_INF("[Cleanup] Forwarding table after cleanup:");
        for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
            const neighbor_entry_t *entry = nt_get(
                (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            if (entry != NULL) {
                LOG_INF("  [%d] addr=0x%04x, gradient=%d, rssi=%d",
                        i, entry->addr, entry->gradient, entry->rssi);
            }
        }
    }

    k_work_schedule(&cleanup_work, K_MSEC(CLEANUP_INTERVAL_MS));
}

/**
 * @brief Work handler for gradient message processing
 *
 * Processes received gradient beacon and updates forwarding table.
 *
 * @param work Pointer to work item (unused)
 */
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

    LOG_INF("Received gradient %d from 0x%04x (RSSI: %d)", 
           msg, sender_addr, rssi);
    
    int64_t current_time = k_uptime_get();
    
    /* Update forwarding table using neighbor_table module */
    nt_update_sorted((neighbor_entry_t *)gradient_srv->forwarding_table,
                     CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                     sender_addr, msg, rssi, current_time);

    /* Debug: Print forwarding table after update */
    LOG_DBG("[Process] Forwarding table after update:");
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            i);
        if (entry != NULL) {
            LOG_DBG("  [%d] addr=0x%04x, gradient=%d, rssi=%d",
                    i, entry->addr, entry->gradient, entry->rssi);
        }
    }

    /* Check if gradient should be updated using routing_policy module */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)gradient_srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    if (best != NULL) {
        if (rp_should_update_my_gradient(gradient_srv->gradient, best->gradient)) {
            uint8_t old_gradient = gradient_srv->gradient;
            gradient_srv->gradient = rp_compute_new_gradient(best->gradient);
            
            LOG_INF("[Process] Gradient updated: [%d] -> [%d]", 
                   old_gradient, gradient_srv->gradient);
            
            bt_mesh_gradient_srv_gradient_send(gradient_srv);
        }
    }
    
    LOG_INF("[Process] Gradient handling completed");
    gradient_ctx.gradient_srv = NULL;
}

void gradient_work_init(void)
{
    k_work_init_delayable(&initial_publish_work, initial_publish_handler);
    k_work_init_delayable(&gradient_process_work, gradient_process_handler);
    k_work_init_delayable(&cleanup_work, cleanup_handler);
}

void gradient_work_start_cleanup(void)
{
    k_work_schedule(&cleanup_work, K_MSEC(CLEANUP_INTERVAL_MS));
    LOG_INF("Cleanup timer started (timeout: %d ms)", 
            CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS);
}

void gradient_work_schedule_initial_publish(void)
{
    k_work_schedule(&initial_publish_work, K_MSEC(INITIAL_PUBLISH_DELAY_MS));
}

void gradient_work_schedule_process(struct bt_mesh_gradient_srv *gradient_srv,
                                    uint8_t gradient, uint16_t sender_addr, int8_t rssi)
{
    gradient_ctx.gradient_srv = gradient_srv;
    gradient_ctx.gradient_msg = gradient;
    gradient_ctx.sender_addr = sender_addr;
    gradient_ctx.rssi = rssi;
    
    k_work_schedule(&gradient_process_work, K_NO_WAIT);
}

void gradient_work_set_srv(struct bt_mesh_gradient_srv *gradient_srv)
{
    g_gradient_srv = gradient_srv;
}
