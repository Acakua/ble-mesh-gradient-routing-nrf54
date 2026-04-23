/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor Data Periodic Transmission Module
 * (Formerly: Heartbeat Module)
 *
 * Implements periodic SENSOR_DATA packet transmission (marker 0xFFFF).
 * Interval is configurable at runtime via OP_SENSOR_INTERVAL from Gateway.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include "heartbeat.h"
#include "gradient_srv.h"
#include "data_forward.h"
#include "packet_stats.h"
#include "sensor_manager.h"

LOG_MODULE_REGISTER(heartbeat, LOG_LEVEL_INF);

/*============================================================================*/
/* Configuration                                                              */
/*============================================================================*/

/** Default transmission interval: 20 seconds */
#define HB_DEFAULT_INTERVAL_MS  20000U

/** Marker value used in DATA packets to identify sensor data frames */
#ifndef BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER 0xFFFF
#endif

/*============================================================================*/
/* Private Data                                                               */
/*============================================================================*/

/** Reference to gradient server for sending packets */
static struct bt_mesh_gradient_srv *heartbeat_srv;

/** Current gradient value (0 = Gateway, do not send) */
static uint8_t current_gradient = UINT8_MAX;

/** Flag indicating whether periodic transmission has been started */
static bool heartbeat_started = false;

/**
 * @brief Runtime-configurable transmission interval (milliseconds).
 *
 * Default: 20000 ms (20 seconds).
 * Updated by heartbeat_set_interval() when Gateway sends OP_SENSOR_INTERVAL.
 * NOT persistent — resets to default on reboot.
 */
static uint32_t g_sensor_interval_ms = HB_DEFAULT_INTERVAL_MS;

/** Delayable work item for periodic sensor data transmission */
static struct k_work_delayable heartbeat_work;

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

/**
 * @brief Calculate a random initial delay (0–10s) to stagger node starts
 */
static uint32_t get_random_initial_delay_ms(void)
{
    return sys_rand32_get() % 10000;
}

/**
 * @brief Work handler — sends one SENSOR_DATA (0xFFFF) packet
 */
static void heartbeat_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    /* Safety checks */
    if (!heartbeat_srv || !heartbeat_started) {
        return;
    }

    /* Gateway (gradient=0) does not send sensor data */
    if (current_gradient == 0) {
        LOG_DBG("[SensorData] Gateway does not send sensor data");
        heartbeat_started = false;
        return;
    }

    /* Gradient not yet assigned — defer */
    if (current_gradient == UINT8_MAX) {
        LOG_DBG("[SensorData] Gradient not yet set, deferring 20s");
        k_work_reschedule(&heartbeat_work, K_SECONDS(20));
        return;
    }

    LOG_DBG("[SensorData] Sending periodic telemetry, interval=%ums",
            g_sensor_interval_ms);

    /* Collect actual sensor data */
    sensor_packet_t pkt;
    build_sensor_packet(&pkt);

    /* Send sensor data packet via best parent */
    int err = bt_mesh_gradient_srv_sensor_data_send(heartbeat_srv, &pkt);

    if (err == 0) {
        pkt_stats_inc_heartbeat();
    } else {
        LOG_ERR("[SensorData] TX Failed (err %d)", err);
    }

    /* Reschedule with configured interval (no jitter - deterministic for verification) */
    k_work_reschedule(&heartbeat_work, K_MSEC(g_sensor_interval_ms));
#endif
}

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void heartbeat_init(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    k_work_init_delayable(&heartbeat_work, heartbeat_work_handler);
    heartbeat_srv       = NULL;
    heartbeat_started   = false;
    current_gradient    = UINT8_MAX;
    g_sensor_interval_ms = HB_DEFAULT_INTERVAL_MS;

    LOG_INF("[SensorData] Initialized (default interval=%u ms)", HB_DEFAULT_INTERVAL_MS);
#else
    LOG_INF("[SensorData] Disabled by config");
#endif
}

void heartbeat_start(struct bt_mesh_gradient_srv *srv)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    if (!srv) {
        LOG_ERR("[SensorData] Cannot start with NULL server");
        return;
    }

    if (heartbeat_started) {
        return; /* Already running */
    }

    heartbeat_srv    = srv;
    current_gradient = srv->gradient;

    if (current_gradient == 0) {
        LOG_INF("[SensorData] Gateway node — periodic TX not required");
        return;
    }

    heartbeat_started = true;

    uint32_t initial_delay_ms = get_random_initial_delay_ms();
    k_work_reschedule(&heartbeat_work, K_MSEC(initial_delay_ms));

    LOG_INF("[SensorData] Started (initial delay: %u ms, interval: %u ms)",
            initial_delay_ms, g_sensor_interval_ms);
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
    LOG_INF("[SensorData] Stopped");
#endif
}

void heartbeat_update_gradient(uint8_t new_gradient)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    uint8_t old_gradient = current_gradient;
    current_gradient = new_gradient;

    /* Case 1: Became Gateway → stop */
    if (new_gradient == 0 && heartbeat_started) {
        LOG_INF("[SensorData] Became Gateway, stopping");
        heartbeat_stop();
    }
    /* Case 2: No longer Gateway → start */
    else if (old_gradient == 0 && new_gradient != 0) {
        LOG_INF("[SensorData] No longer Gateway, starting");
        heartbeat_start(heartbeat_srv);
    }
    /* Case 3: Gradient changed (topology change) → trigger reset */
    else if (heartbeat_started && (old_gradient != new_gradient)) {
        LOG_INF("[SensorData] Gradient changed (%d→%d), triggering reset",
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

    LOG_INF("[SensorData] RESET triggered (topology change)");
    k_work_reschedule(&heartbeat_work, K_MSEC(100));
#endif
}

void heartbeat_set_interval(uint32_t interval_sec)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    if (interval_sec == 0) {
        LOG_WRN("[SensorData] Interval cannot be 0 — ignoring");
        return;
    }

    g_sensor_interval_ms = interval_sec * 1000U;

    LOG_INF("[SensorData] Interval updated → %u sec (%u ms)",
            interval_sec, g_sensor_interval_ms);

    /* Apply immediately with no jitter for deterministic testing */
    if (heartbeat_started) {
        k_work_reschedule(&heartbeat_work, K_MSEC(g_sensor_interval_ms));
    }
#else
    ARG_UNUSED(interval_sec);
#endif
}

uint32_t heartbeat_get_interval_ms(void)
{
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    return g_sensor_interval_ms;
#else
    return 0U;
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