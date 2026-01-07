/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef GRADIENT_TYPES_H
#define GRADIENT_TYPES_H

#include <stdint.h>

/**
 * @brief Unassigned address constant for gradient routing
 */
#define GR_ADDR_UNASSIGNED 0x0000

/**
 * @brief Neighbor entry structure for gradient routing forwarding table
 * 
 * NOTE: Field order MUST match bt_mesh_gradient_srv_forwarding_ctx in gradient_srv.h
 */
typedef struct {
	uint16_t addr;      /**< Mesh unicast address of the neighbor */
	int8_t rssi;        /**< Received signal strength indicator */
	uint8_t gradient;   /**< Gradient value (distance to sink) */
	int64_t last_seen;  /**< Timestamp of last received message (uptime in ms) */
} neighbor_entry_t;

#endif /* GRADIENT_TYPES_H */
