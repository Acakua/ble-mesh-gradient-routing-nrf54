/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef REVERSE_ROUTING_H
#define REVERSE_ROUTING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Timeout for entries in backprop_dest (90 seconds = 3x heartbeat)
 */
#define RRT_ENTRY_TIMEOUT_MS  90000

/**
 * @brief Maximum destinations per nexthop (to prevent memory exhaustion)
 */
#ifdef CONFIG_BT_MESH_GRADIENT_SRV_RRT_MAX_DEST
  #define RRT_MAX_DEST_PER_NEXTHOP CONFIG_BT_MESH_GRADIENT_SRV_RRT_MAX_DEST
#else
  #define RRT_MAX_DEST_PER_NEXTHOP 50 /* Default fallback */
#endif

/**
 * @brief Node in backprop_dest linked list
 * 
 * Each node stores information about a destination reachable via the corresponding nexthop.
 */
typedef struct backprop_node {
    uint16_t addr;              /**< Destination address */
    int64_t last_seen;          /**< Timestamp of last packet received from this dest */
    struct backprop_node *next; /**< Pointer to next node in list */
} backprop_node_t;

/**
 * @brief Initialize reverse routing for a forwarding table
 *
 * Sets all backprop_dest pointers to NULL.
 * Call this when initializing the node.
 *
 * @param table Pointer to forwarding table (cast from bt_mesh_gradient_srv_forwarding_ctx)
 * @param table_size Number of entries in table
 */
void rrt_init(void *table, size_t table_size);

/**
 * @brief Add a destination to backprop_dest of a nexthop
 *
 * Logic:
 * - If dest exists in same nexthop: only update last_seen
 * - If dest exists in DIFFERENT nexthop: remove from old, add to new
 * - If dest doesn't exist: add new node to linked list
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param nexthop_addr Address of nexthop (immediate sender of packet)
 * @param dest_addr Address of destination (original_source of packet)
 * @param timestamp Current time (k_uptime_get())
 *
 * @return 0 on success, -ENOMEM if out of memory, -ENOENT if nexthop not found
 */
int rrt_add_dest(void *table, size_t table_size,
                 uint16_t nexthop_addr, uint16_t dest_addr, int64_t timestamp);

/**
 * @brief Remove a destination from backprop_dest of a nexthop
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param nexthop_addr Address of nexthop
 * @param dest_addr Address of destination to remove
 *
 * @return 0 on success, -ENOENT if not found
 */
int rrt_remove_dest(void *table, size_t table_size,
                    uint16_t nexthop_addr, uint16_t dest_addr);

/**
 * @brief Find nexthop to reach a destination
 *
 * Iterates all entries, searches backprop_dest for dest_addr.
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param dest_addr Address of destination to find
 *
 * @return Nexthop address if found, 0 (BT_MESH_ADDR_UNASSIGNED) if not found
 */
uint16_t rrt_find_nexthop(const void *table, size_t table_size, uint16_t dest_addr);

/**
 * @brief Remove expired entries from backprop_dest
 *
 * Iterates all entries and linked lists, removes nodes with last_seen exceeding timeout.
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param current_time Current timestamp
 * @param timeout_ms Timeout threshold (typically RRT_ENTRY_TIMEOUT_MS)
 *
 * @return Number of entries removed
 */
int rrt_cleanup_expired(void *table, size_t table_size,
                        int64_t current_time, int64_t timeout_ms);

/**
 * @brief Count destinations in backprop_dest of an entry
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param index Index of entry to count
 *
 * @return Number of destinations, 0 if entry invalid
 */
size_t rrt_get_dest_count(const void *table, size_t table_size, size_t index);

/**
 * @brief Print entire reverse routing table (for debugging)
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 */
void rrt_print_table(const void *table, size_t table_size);

/**
 * @brief Free all nodes in backprop_dest of an entry
 *
 * Call this before removing an entry from forwarding table.
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 * @param index Index of entry to clear
 */
void rrt_clear_entry(void *table, size_t table_size, size_t index);

/**
 * @brief Get any known destination from the reverse routing table
 *
 * Useful for testing - finds the first available destination that
 * the Gateway can send BACKPROP to.
 *
 * @param table Pointer to forwarding table
 * @param table_size Number of entries in table
 *
 * @return Address of a reachable destination, or 0 (BT_MESH_ADDR_UNASSIGNED) if none
 */
uint16_t rrt_get_any_destination(const void *table, size_t table_size);

#ifdef __cplusplus
}
#endif

#endif /* REVERSE_ROUTING_H */
