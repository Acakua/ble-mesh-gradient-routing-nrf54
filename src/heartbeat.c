/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Heartbeat Module Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include "heartbeat.h"
#include "gradient_srv.h"
#include "data_forward.h"

LOG_MODULE_REGISTER(heartbeat, LOG_LEVEL_DBG);

/*============================================================================*/
/*                              Private Data                                  */
/*============================================================================*/

/** Reference to gradient server for sending packets */
static struct bt_mesh_gradient_srv *heartbeat_srv;

/** Current gradient value (0 = Gateway, don't send heartbeat) */
static uint8_t current_gradient = UINT8_MAX;

/** Flag indicating if heartbeat has been started */
static bool heartbeat_started = false;

/** Delayable work item for periodic heartbeat */
static struct k_work_delayable heartbeat_work;

/*============================================================================*/
/*                           Private Functions                                */
/*============================================================================*/

/**
 * @brief Work handler - sends one heartbeat packet
 *
 * This function is called by the work queue every HEARTBEAT_INTERVAL_SEC.
 * It sends a DATA packet with:
 * - original_source = this node's address (auto-filled by data_forward_send_direct)
 * - data = HEARTBEAT_MARKER (0xFFFF)
 */
static void heartbeat_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    /* Safety checks */
    if (!heartbeat_srv) {
        LOG_ERR("[Heartbeat] No server reference, stopping");
        heartbeat_started = false;
        return;
    }
    
    /* Don't send if we are Gateway (gradient = 0) */
    if (current_gradient == 0) {
        LOG_DBG("[Heartbeat] Gateway does not send heartbeat");
        /* Don't reschedule - will be restarted if gradient changes */
        heartbeat_started = false;
        return;
    }
    
    /* Don't send if gradient is still uninitialized */
    if (current_gradient == UINT8_MAX) {
        LOG_DBG("[Heartbeat] Gradient not yet set, skipping");
        /* Reschedule to check again later */
        k_work_reschedule(&heartbeat_work, 
                          K_SECONDS(CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC));
        return;
    }
    
    /* Send heartbeat DATA packet
     * 
     * data_forward_send_direct() will:
     * - Set original_source = this node's address
     * - Find best nexthop towards Gateway
     * - Send DATA packet
     * 
     * When Gateway receives this, it learns:
     * "To reach this node, go via the neighbor that forwarded this packet"
     */
    uint16_t dummy_addr = 0;  /* Not used for direct send */
    int err = data_forward_send_direct(heartbeat_srv, dummy_addr, 
                                       BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER);
    
    if (err == 0) {
        LOG_INF("[Heartbeat] Sent heartbeat (gradient=%d)", current_gradient);
    } else if (err == -ENETUNREACH) {
        LOG_WRN("[Heartbeat] No route to Gateway yet, will retry");
    } else {
        LOG_ERR("[Heartbeat] Failed to send, err=%d", err);
    }
    
    /* Reschedule next heartbeat */
    k_work_reschedule(&heartbeat_work, 
                      K_SECONDS(CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC));
#endif
}

/**
 * @brief Calculate random initial delay to avoid synchronized heartbeats
 *
 * If all nodes start sending heartbeat at the same time (e.g., after power-on),
 * there will be packet collisions. This adds a random delay 0-10 seconds.
 *
 * @return Random delay in milliseconds
 */
static uint32_t get_random_initial_delay_ms(void)
{
    /* Random delay between 0 and 10 seconds */
    return sys_rand32_get() % 10000;
}

/*============================================================================*/
/*                            Public Functions                                */
/*============================================================================*/

void heartbeat_init(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    k_work_init_delayable(&heartbeat_work, heartbeat_work_handler);
    heartbeat_srv = NULL;
    heartbeat_started = false;
    current_gradient = UINT8_MAX;
    
    LOG_INF("[Heartbeat] Initialized (interval=%d sec)", 
            CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC);
#else
    LOG_INF("[Heartbeat] Disabled by config");
#endif
}

void heartbeat_start(struct bt_mesh_gradient_srv *srv)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    if (!srv) {
        LOG_ERR("[Heartbeat] Cannot start with NULL server");
        return;
    }
    
    if (heartbeat_started) {
        LOG_DBG("[Heartbeat] Already started");
        return;
    }
    
    heartbeat_srv = srv;
    current_gradient = srv->gradient;
    
    /* Don't start if we are Gateway */
    if (current_gradient == 0) {
        LOG_INF("[Heartbeat] Not starting - this is Gateway");
        return;
    }
    
    heartbeat_started = true;
    
    /* Schedule first heartbeat with random delay to avoid collision */
    uint32_t initial_delay_ms = get_random_initial_delay_ms();
    k_work_reschedule(&heartbeat_work, K_MSEC(initial_delay_ms));
    
    LOG_INF("[Heartbeat] Started (first in %u ms, then every %d sec)",
            initial_delay_ms, CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC);
#else
    ARG_UNUSED(srv);
#endif
}

void heartbeat_stop(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    if (!heartbeat_started) {
        return;
    }
    
    k_work_cancel_delayable(&heartbeat_work);
    heartbeat_started = false;
    
    LOG_INF("[Heartbeat] Stopped");
#endif
}

void heartbeat_update_gradient(uint8_t new_gradient)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    uint8_t old_gradient = current_gradient;
    current_gradient = new_gradient;
    
    /* If we just became Gateway, stop heartbeat */
    if (new_gradient == 0 && heartbeat_started) {
        LOG_INF("[Heartbeat] Became Gateway, stopping heartbeat");
        heartbeat_stop();
    }
    /* If we were Gateway and now are not, start heartbeat */
    else if (old_gradient == 0 && new_gradient != 0 && heartbeat_srv && !heartbeat_started) {
        LOG_INF("[Heartbeat] No longer Gateway, starting heartbeat");
        heartbeat_start(heartbeat_srv);
    }
#else
    ARG_UNUSED(new_gradient);
#endif
}

bool heartbeat_is_active(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    return heartbeat_started;
#else
    return false;
#endif
}
