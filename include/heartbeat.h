/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Heartbeat Module for Gradient Routing
 * 
 * Purpose:
 * - Periodically send heartbeat packets to maintain reverse routes
 * - Ensures Gateway can always reach any node via BACKPROP_DATA
 * - Prevents route expiration in Reverse Routing Table
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
 * Sets up the work item and timer for periodic heartbeat transmission.
 * Must be called once during system initialization.
 *
 * @note Does not start sending heartbeats immediately.
 *       Call heartbeat_start() after mesh is ready.
 */
void heartbeat_init(void);

/**
 * @brief Start heartbeat transmission
 *
 * Begins periodic heartbeat transmission after a random initial delay
 * to avoid all nodes sending heartbeats simultaneously.
 *
 * @param srv Pointer to gradient server instance (for sending DATA packets)
 *
 * @note Only starts if:
 *       - CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED is set
 *       - Node is not Gateway (gradient != 0)
 *       - Node has been provisioned (has valid address)
 */
void heartbeat_start(struct bt_mesh_gradient_srv *srv);

/**
 * @brief Stop heartbeat transmission
 *
 * Stops the periodic heartbeat timer. Call this when:
 * - Node is reset/unprovisioned
 * - Node becomes Gateway (gradient changes to 0)
 * - System is shutting down
 */
void heartbeat_stop(void);

/**
 * @brief Update gradient value for heartbeat logic
 *
 * Called when node's gradient changes. If gradient becomes 0 (Gateway),
 * heartbeat will be automatically stopped.
 *
 * @param new_gradient New gradient value of this node
 */
void heartbeat_update_gradient(uint8_t new_gradient);

/**
 * @brief Check if heartbeat is currently active
 *
 * @return true if heartbeat timer is running
 * @return false if heartbeat is stopped or disabled
 */
bool heartbeat_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* HEARTBEAT_H__ */
