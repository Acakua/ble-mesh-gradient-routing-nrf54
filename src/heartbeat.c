/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 * * Heartbeat Module Implementation
 * Implements Adaptive Heartbeat Mechanism
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include "heartbeat.h"
#include "gradient_srv.h"
#include "data_forward.h"
#include "packet_stats.h"

LOG_MODULE_REGISTER(heartbeat, LOG_LEVEL_DBG);

/*============================================================================*/
/* Configuration                                 */
/*============================================================================*/

/* Adaptive Intervals (in seconds) */
#define HB_INTERVAL_FAST        5   /* Initial / Recovery state */
#define HB_INTERVAL_MEDIUM      10  /* Intermediate state */
#define HB_INTERVAL_SLOW        20  /* Stable state */
/* Note: The "Maintenance" interval comes from Kconfig (default 3600s) */

/* Marker for Heartbeat Data Packet */
#ifndef BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER 0xFFFF
#endif

/*============================================================================*/
/* Private Types                                 */
/*============================================================================*/

enum heartbeat_state {
    HB_STATE_FAST,          /* 5s */
    HB_STATE_MEDIUM,        /* 10s */
    HB_STATE_SLOW,          /* 20s */
    HB_STATE_MAINTENANCE    /* Kconfig Interval (e.g., 1h) */
};

/*============================================================================*/
/* Private Data                                  */
/*============================================================================*/

/** Reference to gradient server for sending packets */
static struct bt_mesh_gradient_srv *heartbeat_srv;

/** Current gradient value (0 = Gateway, don't send heartbeat) */
static uint8_t current_gradient = UINT8_MAX;

/** Flag indicating if heartbeat has been started */
static bool heartbeat_started = false;

/** Current state of the adaptive heartbeat state machine */
static enum heartbeat_state current_hb_state = HB_STATE_FAST;

/** Delayable work item for periodic heartbeat */
static struct k_work_delayable heartbeat_work;

/*============================================================================*/
/* Private Functions                                */
/*============================================================================*/

/**
 * @brief Get the interval in seconds based on current state
 */
static uint32_t get_current_interval_sec(void)
{
    switch (current_hb_state) {
        case HB_STATE_FAST:
            return HB_INTERVAL_FAST;
        case HB_STATE_MEDIUM:
            return HB_INTERVAL_MEDIUM;
        case HB_STATE_SLOW:
            return HB_INTERVAL_SLOW;
        case HB_STATE_MAINTENANCE:
            return CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC;
        default:
            return HB_INTERVAL_FAST;
    }
}

/**
 * @brief Work handler - sends one heartbeat packet
 */
static void heartbeat_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    /* Safety checks */
    if (!heartbeat_srv || !heartbeat_started) {
        return;
    }
    
    /* Don't send if we are Gateway (gradient = 0) */
    if (current_gradient == 0) {
        LOG_DBG("[Heartbeat] Gateway does not send heartbeat");
        heartbeat_started = false;
        return;
    }
    
    /* Don't send if gradient is uninitialized */
    if (current_gradient == UINT8_MAX) {
        LOG_DBG("[Heartbeat] Gradient not yet set, skipping");
        k_work_reschedule(&heartbeat_work, K_SECONDS(HB_INTERVAL_FAST));
        return;
    }
    
    /* * [THÊM ĐÁNH NHÃN LOG TẠI ĐÂY]
     * In log đánh nhãn CONTROL trước khi gửi gói tin Heartbeat 
     */
    LOG_INF("[CONTROL] Sending Heartbeat (0xFFFF)");

    /* * Send heartbeat DATA packet via Unicast to Best Parent.
     * data_forward_send_direct handles looking up the parent.
     */
    uint16_t dummy_addr = 0; 
    int err = data_forward_send_direct(heartbeat_srv, dummy_addr, 
                                     BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER, 0);
    
    if (err == 0) {
        /* SUCCESS: Increment heartbeat counter */
        pkt_stats_inc_heartbeat();
        
        /* Transition towards Maintenance State */
        switch (current_hb_state) {
            case HB_STATE_FAST:
                LOG_DBG("[Heartbeat] FAST -> MEDIUM");
                current_hb_state = HB_STATE_MEDIUM;
                break;
            case HB_STATE_MEDIUM:
                LOG_DBG("[Heartbeat] MEDIUM -> SLOW");
                current_hb_state = HB_STATE_SLOW;
                break;
            case HB_STATE_SLOW:
                LOG_INF("[Heartbeat] SLOW -> MAINTENANCE (Network Stable)");
                current_hb_state = HB_STATE_MAINTENANCE;
                break;
            case HB_STATE_MAINTENANCE:
                LOG_DBG("[Heartbeat] Keeping MAINTENANCE state");
                break;
        }
    } else {
        /* FAILURE: Reset to FAST state to recover connectivity */
        if (current_hb_state != HB_STATE_FAST) {
            LOG_WRN("[Heartbeat] TX Failed (err %d), resetting to FAST state", err);
            current_hb_state = HB_STATE_FAST;
        } else {
            LOG_ERR("[Heartbeat] TX Failed (err %d) in FAST state", err);
        }
    }
    
    /* Reschedule based on new state */
    uint32_t next_interval = get_current_interval_sec();
    k_work_reschedule(&heartbeat_work, K_SECONDS(next_interval));
#endif
}

/**
 * @brief Calculate random initial delay (0-10s)
 */
static uint32_t get_random_initial_delay_ms(void)
{
    return sys_rand32_get() % 10000;
}

/*============================================================================*/
/* Public Functions                                */
/*============================================================================*/

void heartbeat_init(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    k_work_init_delayable(&heartbeat_work, heartbeat_work_handler);
    heartbeat_srv = NULL;
    heartbeat_started = false;
    current_gradient = UINT8_MAX;
    current_hb_state = HB_STATE_FAST;
    
    LOG_INF("[Heartbeat] Initialized (Maintenance Interval=%d sec)", 
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
        return; /* Already started */
    }
    
    heartbeat_srv = srv;
    current_gradient = srv->gradient;
    
    /* Gateway check */
    if (current_gradient == 0) {
        LOG_INF("[Heartbeat] This is Gateway, heartbeat not required");
        return;
    }
    
    heartbeat_started = true;
    current_hb_state = HB_STATE_FAST; /* Always start FAST */
    
    /* Random delay start */
    uint32_t initial_delay_ms = get_random_initial_delay_ms();
    k_work_reschedule(&heartbeat_work, K_MSEC(initial_delay_ms));
    
    LOG_INF("[Heartbeat] Started (Delay: %dms, Mode: FAST)", initial_delay_ms);
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
    
    /* Case 1: Became Gateway -> Stop */
    if (new_gradient == 0 && heartbeat_started) {
        LOG_INF("[Heartbeat] Became Gateway, stopping");
        heartbeat_stop();
    }
    /* Case 2: No longer Gateway -> Start */
    else if (old_gradient == 0 && new_gradient != 0) {
        LOG_INF("[Heartbeat] No longer Gateway, starting");
        heartbeat_start(heartbeat_srv);
    }
    /* Case 3: Gradient changed (Topology changed) -> Reset to FAST */
    else if (heartbeat_started && (old_gradient != new_gradient)) {
        LOG_INF("[Heartbeat] Gradient changed (%d->%d), resetting cycle", 
                old_gradient, new_gradient);
        heartbeat_trigger_reset();
    }
#else
    ARG_UNUSED(new_gradient);
#endif
}

void heartbeat_trigger_reset(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    if (!heartbeat_started) {
        return;
    }

    /* Only reset if not already in FAST state to avoid unnecessary spam */
    if (current_hb_state != HB_STATE_FAST) {
        LOG_INF("[Heartbeat] Triggered RESET to FAST state");
        current_hb_state = HB_STATE_FAST;
        
        /* Schedule immediate run (small delay to avoid race conditions) */
        k_work_reschedule(&heartbeat_work, K_MSEC(100));
    }
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