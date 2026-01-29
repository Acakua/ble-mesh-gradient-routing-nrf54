/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file packet_stats.c
 * @brief Packet Statistics Module Implementation
 */

#include "packet_stats.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(packet_stats, LOG_LEVEL_INF);

/*============================================================================*/
/* Private Data                                                               */
/*============================================================================*/

/** Atomic counters for thread-safe access */
static atomic_t gradient_beacon_count = ATOMIC_INIT(0);
static atomic_t heartbeat_count = ATOMIC_INIT(0);
static atomic_t data_tx_count = ATOMIC_INIT(0);
static atomic_t route_change_count = ATOMIC_INIT(0);
static bool stats_enabled = false;

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void pkt_stats_init(void)
{
    atomic_set(&gradient_beacon_count, 0);
    atomic_set(&heartbeat_count, 0);
    atomic_set(&data_tx_count, 0);
    atomic_set(&route_change_count, 0);
    
    LOG_INF("[PktStats] Initialized - all counters reset");
}

void pkt_stats_inc_gradient_beacon(void)
{
    if (!stats_enabled) return;
    atomic_inc(&gradient_beacon_count);
}

void pkt_stats_inc_heartbeat(void)
{
    if (!stats_enabled) return;
    atomic_inc(&heartbeat_count);
}

void pkt_stats_inc_data_tx(void)
{
    if (!stats_enabled) return;
    atomic_inc(&data_tx_count);
}

void pkt_stats_inc_route_change(void)
{
    if (!stats_enabled) return;
    atomic_inc(&route_change_count);
}

void pkt_stats_get(struct packet_stats *stats)
{
    if (stats == NULL) {
        return;
    }
    
    stats->gradient_beacon_tx = (uint32_t)atomic_get(&gradient_beacon_count);
    stats->heartbeat_tx = (uint32_t)atomic_get(&heartbeat_count);
    stats->data_tx = (uint32_t)atomic_get(&data_tx_count);
    stats->route_change_count = (uint32_t)atomic_get(&route_change_count);
}

uint32_t pkt_stats_get_gradient_beacon(void)
{
    return (uint32_t)atomic_get(&gradient_beacon_count);
}

uint32_t pkt_stats_get_heartbeat(void)
{
    return (uint32_t)atomic_get(&heartbeat_count);
}

uint32_t pkt_stats_get_data_tx(void)
{
    return (uint32_t)atomic_get(&data_tx_count);
}

uint32_t pkt_stats_get_route_change(void)
{
    return (uint32_t)atomic_get(&route_change_count);
}

uint32_t pkt_stats_get_control_total(void)
{
    return pkt_stats_get_gradient_beacon() + pkt_stats_get_heartbeat();
}

void pkt_stats_reset(void)
{
    atomic_set(&gradient_beacon_count, 0);
    atomic_set(&heartbeat_count, 0);
    atomic_set(&data_tx_count, 0);
    atomic_set(&route_change_count, 0);
    
    LOG_INF("[PktStats] All counters reset to 0");
}

void pkt_stats_set_enabled(bool enable)
{
    stats_enabled = enable;
    LOG_INF("[PktStats] Statistics Collection: %s", enable ? "ENABLED" : "DISABLED");
}

bool pkt_stats_is_enabled(void)
{
    return stats_enabled;
}
