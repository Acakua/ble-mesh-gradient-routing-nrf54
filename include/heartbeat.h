/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 * * Heartbeat Module for Gradient Routing
 * * Purpose:
 * - Implements Adaptive Heartbeat mechanism (Fast -> Slow -> Maintenance)
 * - Maintains reverse routes in the Gateway and upstream nodes
 * - Automatically recovers connectivity when topology changes
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
 * @brief Initialize heartbeat module
 *
 * Sets up the work item and timer.
 * Must be called once during system initialization.
 */
void heartbeat_init(void);

/**
 * @brief Start heartbeat transmission
 *
 * Begins the adaptive heartbeat cycle starting at FAST state (5s).
 * Includes a random initial delay to avoid collision storm.
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
 * @brief Update gradient value for heartbeat logic
 *
 * Called when node's gradient changes.
 * - If new gradient is 0 (Gateway): Stops heartbeat.
 * - If gradient changes (but not 0): Triggers a reset to FAST state.
 *
 * @param new_gradient New gradient value of this node
 */
void heartbeat_update_gradient(uint8_t new_gradient);

/**
 * @brief Trigger a reset of the heartbeat cycle
 * * Forces the heartbeat state machine back to FAST mode (5s interval)
 * and schedules an immediate heartbeat packet.
 * * Call this function when:
 * - Gradient changes (Topology change)
 * - Best Parent changes
 * - Data transmission fails (Link break detected)
 * - Loop is detected
 */
void heartbeat_trigger_reset(void);

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