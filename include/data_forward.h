/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef DATA_FORWARD_H
#define DATA_FORWARD_H

#include "gradient_srv.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize data forwarding module
 *
 * Must be called before using any data forwarding functions.
 */
void data_forward_init(void);

/**
 * @brief Forward data packet to next hop
 *
 * Attempts to send data to the best next hop. If sending fails,
 * automatically retries with the next entry in forwarding table.
 * Packet format: [original_source: 2 bytes] + [data: 2 bytes]
 *
 * @param gradient_srv Pointer to gradient server instance
 * @param data Data payload to forward
 * @param original_source Address of node that originally created the packet
 * @param sender_addr Address of immediate sender (to avoid sending back)
 *
 * @return 0 on success, negative error code on failure
 */
int data_forward_send(struct bt_mesh_gradient_srv *gradient_srv,
                      uint16_t data, uint16_t original_source, 
                      uint16_t sender_addr, uint8_t current_hop_count,
                      int8_t path_min_rssi);

/**
 * @brief Send data packet directly (for button press, no retry on sender skip)
 *
 * @param gradient_srv Pointer to gradient server instance
 * @param addr Destination address
 * @param data Data payload to send
 *
 * @return 0 on success, negative error code on failure
 */
int data_forward_send_direct(struct bt_mesh_gradient_srv *gradient_srv,
                             uint16_t addr, uint16_t data, 
                             int8_t initial_rssi);

const neighbor_entry_t *find_strict_upstream_parent(
    struct bt_mesh_gradient_srv *srv, uint16_t exclude_addr);

#ifdef __cplusplus
}
#endif

#endif /* DATA_FORWARD_H */
