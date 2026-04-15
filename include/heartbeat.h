/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sensor Data Periodic Transmission Module
 * (Formerly: Heartbeat Module)
 *
 * Purpose:
 * - Sends periodic SENSOR_DATA packets (marker 0xFFFF) to maintain reverse routes
 * - Interval configurable at runtime via Gateway command (OP_SENSOR_INTERVAL)
 * - Automatically stops when node becomes Gateway (gradient=0)
 * - Recovers connectivity when topology changes
 */

#ifndef HEARTBEAT_H__
#define HEARTBEAT_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration */
struct bt_mesh_gradient_srv;

/**
 * @brief Initialize sensor data transmission module
 *
 * Sets up the work item and timer.
 * Must be called once during system initialization.
 */
void heartbeat_init(void);

/**
 * @brief Start periodic sensor data transmission
 *
 * Begins periodic sensor data cycle.
 * Includes a random initial delay (0-10s) to avoid collision storm.
 *
 * @param srv Pointer to gradient server instance
 *
 * @note Only starts if node is NOT Gateway and Heartbeat is enabled in Kconfig.
 */
void heartbeat_start(struct bt_mesh_gradient_srv *srv);

/**
 * @brief Stop heartbeat transmission
 *
 * Stops the timer. Call when node becomes Gateway or unprovisioned.
 */
void heartbeat_stop(void);

/**
 * @brief Update gradient value for sensor data logic
 *
 * Called when node's gradient changes.
 * - If new gradient is 0 (Gateway): Stops transmission.
 * - If gradient changes (but not 0): Triggers a reset.
 *
 * @param new_gradient New gradient value of this node
 */
void heartbeat_update_gradient(uint8_t new_gradient);

/**
 * @brief Trigger an immediate reset of the transmission cycle
 *
 * Reschedules the next sensor data packet with a short delay (100ms).
 * Call this when topology changes (gradient update, parent change).
 */
void heartbeat_trigger_reset(void);

/**
 * @brief Set the sensor data transmission interval at runtime
 *
 * Updates the interval and reschedules the next transmission immediately.
 * The new interval takes effect on the NEXT scheduled packet.
 *
 * @param interval_sec New interval in seconds (1 - 65535). 0 is rejected.
 *
 * @note NOT persistent across reboots. Reboot resets to default (20s).
 */
void heartbeat_set_interval(uint32_t interval_sec);

/**
 * @brief Get the current transmission interval in milliseconds
 *
 * @return Current interval in milliseconds (default 20000).
 *         Returns 0 if heartbeat is disabled in config.
 */
uint32_t heartbeat_get_interval_ms(void);

/**
 * @brief Check if heartbeat is currently active
 *
 * @return true if heartbeat timer is running
 */
bool heartbeat_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* HEARTBEAT_H__ */