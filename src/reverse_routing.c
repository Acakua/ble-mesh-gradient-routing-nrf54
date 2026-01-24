/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "reverse_routing.h"
#include "gradient_srv.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(reverse_routing, LOG_LEVEL_DBG);

/* * MEMORY OPTIMIZATION: USE MEMORY SLAB INSTEAD OF HEAP
 * * Calculating total required nodes based on Kconfig limits.
 * This prevents heap fragmentation on nRF54L15.
 */
#if defined(CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) && \
    defined(CONFIG_BT_MESH_GRADIENT_SRV_RRT_MAX_DEST)
    #define RRT_TOTAL_NODES (CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE * \
                             CONFIG_BT_MESH_GRADIENT_SRV_RRT_MAX_DEST)
#else
    /* Fallback safe defaults if Kconfig is missing */
    #define RRT_TOTAL_NODES (25 * 50) 
#endif

/* Fallback definition if header doesn't define it */
#ifndef RRT_MAX_DEST_PER_NEXTHOP
    #define RRT_MAX_DEST_PER_NEXTHOP CONFIG_BT_MESH_GRADIENT_SRV_RRT_MAX_DEST
#endif

/* Define the Memory Slab */
K_MEM_SLAB_DEFINE(rrt_mem_slab, sizeof(backprop_node_t), RRT_TOTAL_NODES, 4);


/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Find entry in forwarding table by nexthop address
 */
static bt_mesh_gradient_srv_forwarding_ctx *find_entry_by_addr(
    bt_mesh_gradient_srv_forwarding_ctx *table,
    size_t table_size,
    uint16_t nexthop_addr)
{
    for (size_t i = 0; i < table_size; i++) {
        if (table[i].addr == nexthop_addr) {
            return &table[i];
        }
    }
    return NULL;
}

/**
 * @brief Find destination in a linked list
 * * @return Pointer to node if found, NULL otherwise
 */
static backprop_node_t *find_dest_in_list(backprop_node_t *head, uint16_t dest_addr)
{
    backprop_node_t *current = head;
    while (current != NULL) {
        if (current->addr == dest_addr) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

/**
 * @brief Remove destination from a linked list
 * * @return true if removed, false if not found
 */
static bool remove_dest_from_list(backprop_node_t **head, uint16_t dest_addr)
{
    backprop_node_t *prev = NULL;
    backprop_node_t *current = *head;
    
    while (current != NULL) {
        if (current->addr == dest_addr) {
            if (prev == NULL) {
                *head = current->next;  /* Remove head node */
            } else {
                prev->next = current->next;  /* Remove middle/tail node */
            }
            
            /* CHANGED: Use slab free instead of k_free */
            k_mem_slab_free(&rrt_mem_slab, (void **)&current);
            return true;
        }
        prev = current;
        current = current->next;
    }
    return false;
}

/**
 * @brief Count nodes in a linked list
 */
static size_t count_list(backprop_node_t *head)
{
    size_t count = 0;
    backprop_node_t *current = head;
    while (current != NULL) {
        count++;
        current = current->next;
    }
    return count;
}

/**
 * @brief Find and remove oldest node in linked list (smallest last_seen)
 */
static void remove_oldest_from_list(backprop_node_t **head)
{
    if (*head == NULL) {
        return;
    }
    
    backprop_node_t *oldest = *head;
    backprop_node_t *oldest_prev = NULL;
    backprop_node_t *prev = NULL;
    backprop_node_t *current = *head;
    
    /* Find oldest node */
    while (current != NULL) {
        if (current->last_seen < oldest->last_seen) {
            oldest = current;
            oldest_prev = prev;
        }
        prev = current;
        current = current->next;
    }
    
    /* Remove oldest */
    if (oldest_prev == NULL) {
        *head = oldest->next;
    } else {
        oldest_prev->next = oldest->next;
    }
    
    LOG_DBG("[RRT] Removed oldest dest 0x%04x to make room", oldest->addr);
    
    /* CHANGED: Use slab free instead of k_free */
    k_mem_slab_free(&rrt_mem_slab, (void **)&oldest);
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

void rrt_init(void *table, size_t table_size)
{
    bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    for (size_t i = 0; i < table_size; i++) {
        ft[i].backprop_dest = NULL;
    }
    
    LOG_INF("[RRT] Initialized reverse routing table (%d entries)", table_size);
    LOG_INF("[RRT] Memory Slab: %d blocks available", RRT_TOTAL_NODES);
}

int rrt_add_dest(void *table, size_t table_size,
                 uint16_t nexthop_addr, uint16_t dest_addr, int64_t timestamp)
{
    bt_mesh_gradient_srv_forwarding_ctx *ft = (bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    /* 1. Find target entry first */
    bt_mesh_gradient_srv_forwarding_ctx *target_entry = find_entry_by_addr(ft, table_size, nexthop_addr);
    if (target_entry == NULL) {
        LOG_WRN("[RRT] Nexthop 0x%04x not found in forwarding table", nexthop_addr);
        return -ENOENT;
    }

    /* 2. Check if it's already on the correct neighbor */
    backprop_node_t *existing = find_dest_in_list(target_entry->backprop_dest, dest_addr);
    if (existing != NULL) {
        existing->last_seen = timestamp; /* Just update time */
        
        /* Ensure it doesn't exist elsewhere (duplicate cleanup) */
        for (size_t i = 0; i < table_size; i++) {
             if (&ft[i] != target_entry && ft[i].addr != 0) {
                 if (remove_dest_from_list(&ft[i].backprop_dest, dest_addr)) {
                     LOG_WRN("[RRT] Fixed duplicate dest 0x%04x (removed from 0x%04x)", dest_addr, ft[i].addr);
                 }
             }
        }
        return 0;
    }

    /* 3. It's a Move or New Add. ALLOCATE FROM SLAB FIRST */
    backprop_node_t *new_node;
    if (k_mem_slab_alloc(&rrt_mem_slab, (void **)&new_node, K_NO_WAIT) != 0) {
        LOG_ERR("[RRT] Memory Slab Full (%d nodes). Dropping route to 0x%04x", 
                RRT_TOTAL_NODES, dest_addr);
        return -ENOMEM; /* Abort cleanly */
    }

    /* 4. Now safe to remove from old location (Move operation) */
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr != nexthop_addr && ft[i].addr != 0) {
            if (remove_dest_from_list(&ft[i].backprop_dest, dest_addr)) {
                LOG_INF("[RRT] Dest 0x%04x moved from nexthop 0x%04x to 0x%04x",
                        dest_addr, ft[i].addr, nexthop_addr);
                break; /* Can only exist in one place */
            }
        }
    }

    /* 5. Check limits and insert */
    size_t count = count_list(target_entry->backprop_dest);
    if (count >= RRT_MAX_DEST_PER_NEXTHOP) {
        LOG_WRN("[RRT] Max destinations reached for nexthop 0x%04x, removing oldest", nexthop_addr);
        remove_oldest_from_list(&target_entry->backprop_dest);
    }

    new_node->addr = dest_addr;
    new_node->last_seen = timestamp;
    new_node->next = target_entry->backprop_dest;
    target_entry->backprop_dest = new_node;
    
    LOG_INF("[RRT] Added dest 0x%04x via nexthop 0x%04x", dest_addr, nexthop_addr);
    return 0;
}

int rrt_remove_dest(void *table, size_t table_size,
                    uint16_t nexthop_addr, uint16_t dest_addr)
{
    bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    bt_mesh_gradient_srv_forwarding_ctx *entry = find_entry_by_addr(ft, table_size, nexthop_addr);
    if (entry == NULL) {
        return -ENOENT;
    }
    
    if (remove_dest_from_list(&entry->backprop_dest, dest_addr)) {
        LOG_INF("[RRT] Removed dest 0x%04x from nexthop 0x%04x", dest_addr, nexthop_addr);
        return 0;
    }
    
    return -ENOENT;
}

uint16_t rrt_find_nexthop(const void *table, size_t table_size, uint16_t dest_addr)
{
    const bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (const bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    // LOG_DBG("[RRT_SEARCH] Looking for dest 0x%04x in table size %d", dest_addr, table_size);

    for (size_t i = 0; i < table_size; i++) {
        /* Bỏ qua các entry trống */
        if (ft[i].addr == GR_ADDR_UNASSIGNED || ft[i].addr == 0) {
            continue;
        }

        /* Kiểm tra trực tiếp: Nếu đích đến chính là hàng xóm này */
        if (ft[i].addr == dest_addr) {
            LOG_DBG("[RRT] Found direct neighbor: 0x%04x", dest_addr);
            return ft[i].addr;
        }
        
        /* Kiểm tra danh sách gián tiếp (Backprop list) */
        backprop_node_t *current = ft[i].backprop_dest;
        while (current != NULL) {
            // LOG_DBG("  Checking nexthop 0x%04x -> knows 0x%04x?", ft[i].addr, current->addr);
            if (current->addr == dest_addr) {
                LOG_INF("[RRT] Found route to 0x%04x via nexthop 0x%04x",
                        dest_addr, ft[i].addr);
                return ft[i].addr;
            }
            current = current->next;
        }
    }
    
    LOG_WRN("[RRT] No route found to dest 0x%04x", dest_addr);
    return 0;  /* BT_MESH_ADDR_UNASSIGNED */
}


int rrt_cleanup_expired(void *table, size_t table_size,
                        int64_t current_time, int64_t timeout_ms)
{
    bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (bt_mesh_gradient_srv_forwarding_ctx *)table;
    int removed_count = 0;
    
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr == 0) {
            continue;
        }
        
        backprop_node_t **pp = &ft[i].backprop_dest;
        while (*pp != NULL) {
            backprop_node_t *current = *pp;
            int64_t age = current_time - current->last_seen;
            
            if (age > timeout_ms) {
                LOG_INF("[RRT] Expired dest 0x%04x from nexthop 0x%04x (age=%lld ms)",
                        current->addr, ft[i].addr, age);
                *pp = current->next;
                
                /* CHANGED: Use slab free instead of k_free */
                k_mem_slab_free(&rrt_mem_slab, (void **)&current);
                
                removed_count++;
            } else {
                pp = &current->next;
            }
        }
    }
    
    if (removed_count > 0) {
        LOG_INF("[RRT] Cleanup removed %d expired entries", removed_count);
    }
    
    return removed_count;
}

size_t rrt_get_dest_count(const void *table, size_t table_size, size_t index)
{
    if (index >= table_size) {
        return 0;
    }
    
    const bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (const bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    return count_list(ft[index].backprop_dest);
}

void rrt_print_table(const void *table, size_t table_size)
{
    const bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (const bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    LOG_INF("========== Reverse Routing Table ==========");
    
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr == 0) {
            continue;
        }
        
        size_t count = count_list(ft[i].backprop_dest);
        LOG_INF("Entry[%d]: nexthop=0x%04x, %d destinations:", i, ft[i].addr, count);
        
        backprop_node_t *current = ft[i].backprop_dest;
        while (current != NULL) {
            LOG_INF("  -> dest=0x%04x (last_seen=%lld)", 
                    current->addr, current->last_seen);
            current = current->next;
        }
    }
    
    LOG_INF("============================================");
}

void rrt_clear_entry(void *table, size_t table_size, size_t index)
{
    if (index >= table_size) {
        return;
    }
    
    bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    backprop_node_t *current = ft[index].backprop_dest;
    while (current != NULL) {
        backprop_node_t *next = current->next;
        
        /* CHANGED: Use slab free instead of k_free */
        k_mem_slab_free(&rrt_mem_slab, (void **)&current);
        
        current = next;
    }
    
    ft[index].backprop_dest = NULL;
    LOG_DBG("[RRT] Cleared backprop_dest for entry[%d]", index);
}

uint16_t rrt_get_any_destination(const void *table, size_t table_size)
{
    const bt_mesh_gradient_srv_forwarding_ctx *ft = 
        (const bt_mesh_gradient_srv_forwarding_ctx *)table;
    
    if (!ft) {
        return 0;  /* BT_MESH_ADDR_UNASSIGNED */
    }
    
    /* Search through all neighbors for any known destination */
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr == 0) {
            continue;
        }
        
        /* Check if this neighbor has any backprop destinations */
        if (ft[i].backprop_dest != NULL) {
            /* Return the first destination found */
            LOG_DBG("[RRT] Found destination 0x%04x via nexthop 0x%04x",
                    ft[i].backprop_dest->addr, ft[i].addr);
            return ft[i].backprop_dest->addr;
        }
    }
    
    return 0;  /* BT_MESH_ADDR_UNASSIGNED */
}