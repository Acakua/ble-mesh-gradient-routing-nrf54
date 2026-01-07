/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef NEIGHBOR_TABLE_H
#define NEIGHBOR_TABLE_H

#include "gradient_types.h"
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize neighbor table entries
 *
 * Sets all entries to unassigned state:
 * - addr = GR_ADDR_UNASSIGNED
 * - rssi = INT8_MIN
 * - gradient = UINT8_MAX
 * - last_seen = 0
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 */
void nt_init(neighbor_entry_t *table, size_t table_size);

/**
 * @brief Update neighbor table with sorted insertion
 *
 * Inserts or updates a neighbor entry while maintaining sort order.
 * Sort priority: smaller gradient is better; on tie, higher RSSI is better.
 *
 * If sender already exists in table:
 * - Updates rssi, gradient, last_seen
 * - Moves entry to correct sorted position if needed
 *
 * If sender is new:
 * - Inserts at correct sorted position
 * - Shifts existing entries to make room
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 * @param sender_addr Mesh address of the sender
 * @param sender_gradient Gradient value of the sender
 * @param sender_rssi RSSI of the received message
 * @param now_ms Current timestamp in milliseconds (from k_uptime_get)
 *
 * @return true if table was modified, false otherwise
 */
bool nt_update_sorted(neighbor_entry_t *table, size_t table_size,
                      uint16_t sender_addr, uint8_t sender_gradient, int8_t sender_rssi,
                      int64_t now_ms);

/**
 * @brief Get the best (first) entry in the neighbor table
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 *
 * @return Pointer to best entry (index 0) if valid, NULL if table is empty
 */
const neighbor_entry_t *nt_best(const neighbor_entry_t *table, size_t table_size);

/**
 * @brief Get entry at specific index
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 * @param idx Index to retrieve
 *
 * @return Pointer to entry if idx valid and addr not UNASSIGNED, NULL otherwise
 */
const neighbor_entry_t *nt_get(const neighbor_entry_t *table, size_t table_size, size_t idx);

/**
 * @brief Remove entry at specific index
 *
 * Removes the entry at the given index and shifts remaining entries up.
 * The last slot is reset to unassigned state.
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 * @param idx Index to remove
 *
 * @return Address of removed entry, or GR_ADDR_UNASSIGNED if invalid
 */
uint16_t nt_remove(neighbor_entry_t *table, size_t table_size, size_t idx);

/**
 * @brief Count valid entries in the table
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 *
 * @return Number of valid (non-unassigned) entries
 */
size_t nt_count(const neighbor_entry_t *table, size_t table_size);

/**
 * @brief Check if entry at index is expired
 *
 * @param table Pointer to neighbor table array
 * @param table_size Number of entries in the table
 * @param idx Index to check
 * @param current_time_ms Current time in milliseconds
 * @param timeout_ms Timeout threshold in milliseconds
 *
 * @return true if entry exists and is expired, false otherwise
 */
bool nt_is_expired(const neighbor_entry_t *table, size_t table_size, size_t idx,
                   int64_t current_time_ms, int64_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* NEIGHBOR_TABLE_H */
