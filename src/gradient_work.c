/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "gradient_work.h"
#include "neighbor_table.h"
#include "routing_policy.h"
#include "heartbeat.h"
#include "reverse_routing.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gradient_work, LOG_LEVEL_DBG);

/* ========================================================================= */
/* Configuration                               */
/* ========================================================================= */

/* Default timeout if not defined in Kconfig */
#ifndef CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS
#define CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS  120000 /* 120s */
#endif

/* Timing constants */
#define CLEANUP_INTERVAL_MS       15000  /* Check for expired neighbors every 15s */
#define INITIAL_PUBLISH_DELAY_MS  500    /* Random jitter start */

/* Periodic Gradient Publish Interval 
 * Should be less than NODE_TIMEOUT_MS to keep neighbors alive.
 * Recommneded: Timeout / 3 
 */
#define GRADIENT_PUBLISH_INTERVAL_MS (CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS / 3)

/* ========================================================================= */
/* Private Data                                */
/* ========================================================================= */

/* Global gradient server reference */
static struct bt_mesh_gradient_srv *g_gradient_srv = NULL;

/* Work items */
static struct k_work_delayable publish_work;
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

/* ========================================================================= */
/* Work Handlers                                 */
/* ========================================================================= */

/**
 * @brief Work handler for periodic gradient publish
 *
 * Publishes Gradient Beacon to advertise presence and metric to neighbors.
 * Keeps the node "alive" in neighbors' tables.
 */
static void publish_handler(struct k_work *work)
{
    if (g_gradient_srv != NULL) {
        /* Only publish if we have a valid gradient (or we are Sink) */
        if (g_gradient_srv->gradient != UINT8_MAX) {
            
            // [THÊM ĐÁNH NHÃN LOG TẠI ĐÂY]
            // Đánh nhãn CONTROL cho gói tin quảng bá định tuyến (Gradient Beacon)
            LOG_INF("[CONTROL] Broadcasting Gradient Beacon: %d", g_gradient_srv->gradient);

            int err = bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
            if (err) {
                LOG_WRN("Gradient publish failed: %d", err);
            } else {
                LOG_DBG("Gradient published: %d", g_gradient_srv->gradient);
            }
        }
        
        /* Reschedule for next period to keep neighbors updated */
        k_work_reschedule(&publish_work, K_MSEC(GRADIENT_PUBLISH_INTERVAL_MS));
    }
}

/**
 * @brief Work handler for periodic neighbor cleanup
 *
 * Removes expired neighbors and updates gradient/heartbeat if needed.
 */
static void cleanup_handler(struct k_work *work)
{
    if (g_gradient_srv == NULL) {
        return;
    }
    
    int64_t current_time = k_uptime_get();
    bool table_changed = false;
    bool should_publish = false;
    bool best_parent_lost = false; /* Flag to trigger heartbeat reset */
    
    LOG_DBG("[Cleanup] Running cleanup check...");
    
    /* Lock forwarding table for thread-safe access */
    k_mutex_lock(&g_gradient_srv->forwarding_table_mutex, K_FOREVER);
    
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
            
            /* Detect if we are losing our Best Parent (Index 0) */
            if (i == 0) {
                best_parent_lost = true;
                LOG_WRN("[Cleanup] BEST PARENT lost! Route instability detected.");
            }
            
            /* Free backprop_dest linked list before removing entry */
            rrt_clear_entry(
                (void *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            uint16_t removed_addr = nt_remove(
                (neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            if (removed_addr != GR_ADDR_UNASSIGNED) {
                table_changed = true;
                i--;  /* Re-check this index since entries shifted */
            }
        }
    }

    /* Cleanup reverse routing table */
    int64_t rrt_timeout_ms = CONFIG_BT_MESH_GRADIENT_SRV_RRT_TIMEOUT_SEC * 1000LL;
    int rrt_removed = rrt_cleanup_expired(
        g_gradient_srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
        current_time,
        rrt_timeout_ms);
    
    if (rrt_removed > 0) {
        LOG_INF("[Cleanup] RRT: Removed %d expired reverse routes", rrt_removed);
    }

    /* Update gradient based on best remaining parent */
    if (table_changed) {
#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
        /* Sink node (Gateway) always has gradient=0 */
        LOG_DBG("[Cleanup] Sink node, gradient fixed at 0");
#else
        const neighbor_entry_t *best = nt_best(
            (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

        if (best != NULL) {
            uint8_t best_parent_gradient = best->gradient;

            if (best_parent_gradient != UINT8_MAX) {
                uint8_t old_gradient = g_gradient_srv->gradient;
                uint8_t new_gradient = rp_compute_new_gradient(best_parent_gradient);
                
                if (new_gradient == 0) new_gradient = 1; /* Safety */
                
                g_gradient_srv->gradient = new_gradient;

                if (old_gradient != new_gradient) {
                     LOG_INF("[Cleanup] Gradient updated: [%d] -> [%d]",
                            old_gradient, g_gradient_srv->gradient);
                     /* Gradient change will trigger Heartbeat Reset internally */
                     heartbeat_update_gradient(g_gradient_srv->gradient);
                     should_publish = true;
                } else if (best_parent_lost) {
                     /* CRITICAL: Gradient didn't change (backup parent has same gradient),
                      * BUT we lost our active path. We MUST trigger reset to 
                      * establish route via backup parent immediately. */
                     LOG_INF("[Cleanup] Gradient same, but parent changed. Triggering Heartbeat Reset.");
                     heartbeat_trigger_reset();
                }
            }
        } else {
            LOG_WRN("[Cleanup] No parents available, resetting gradient to 255");
            g_gradient_srv->gradient = UINT8_MAX;
            heartbeat_update_gradient(g_gradient_srv->gradient);
            should_publish = true;
        }
#endif
    }

    k_mutex_unlock(&g_gradient_srv->forwarding_table_mutex);

    if (should_publish) {
        bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
    }

    k_work_schedule(&cleanup_work, K_MSEC(CLEANUP_INTERVAL_MS));
}

/**
 * @brief Work handler for gradient message processing
 */
static void gradient_process_handler(struct k_work *work)
{
    struct gradient_context *ctx = &gradient_ctx;
    struct bt_mesh_gradient_srv *gradient_srv = ctx->gradient_srv;
    uint8_t msg = ctx->gradient_msg;
    int8_t rssi = ctx->rssi;
    uint16_t sender_addr = ctx->sender_addr;
    
    if (!gradient_srv) return;

    LOG_DBG("Received gradient %d from 0x%04x (RSSI: %d)", msg, sender_addr, rssi);
    
    int64_t current_time = k_uptime_get();
    
    k_mutex_lock(&gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    nt_update_sorted((neighbor_entry_t *)gradient_srv->forwarding_table,
                      CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                      sender_addr, msg, rssi, current_time);

    k_mutex_unlock(&gradient_srv->forwarding_table_mutex);

    /* Check if gradient should be updated */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)gradient_srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    if (best != NULL) {
#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
        /* Sink logic: Do nothing */
#else
        /* Regular node logic */
        if (rp_should_update_my_gradient(gradient_srv->gradient, best->gradient)) {
            uint8_t old_gradient = gradient_srv->gradient;
            uint8_t new_gradient = rp_compute_new_gradient(best->gradient);
            
            if (new_gradient == 0) new_gradient = 1;
            
            gradient_srv->gradient = new_gradient;
            
            LOG_INF("[Process] Gradient updated: [%d] -> [%d] (Parent: 0x%04x)", 
                    old_gradient, gradient_srv->gradient, best->addr);
            
            /* Notify heartbeat - this handles the RESET internally */
            heartbeat_update_gradient(gradient_srv->gradient);
            
            bt_mesh_gradient_srv_gradient_send(gradient_srv);
        }
#endif
    }
    
    gradient_ctx.gradient_srv = NULL;
}

/* ========================================================================= */
/* Public Functions                              */
/* ========================================================================= */

void gradient_work_init(void)
{
    /* Renamed initial_publish_work to publish_work for periodic use */
    k_work_init_delayable(&publish_work, publish_handler);
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
    /* Start the periodic publish cycle */
    k_work_schedule(&publish_work, K_MSEC(INITIAL_PUBLISH_DELAY_MS));
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