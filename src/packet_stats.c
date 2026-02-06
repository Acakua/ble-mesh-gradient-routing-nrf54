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

#define MAX_PENDING_PONGS 10
#define MAX_RTT_HISTORY   50

struct pending_pong {
    uint16_t seq;
    uint32_t send_time;
};

/** Atomic counters for thread-safe access */
static atomic_t gradient_beacon_count = ATOMIC_INIT(0);
static atomic_t heartbeat_count = ATOMIC_INIT(0);
static atomic_t data_tx_count = ATOMIC_INIT(0);
static atomic_t data_fwd_count = ATOMIC_INIT(0);
static atomic_t route_change_count = ATOMIC_INIT(0);
static atomic_t rx_data_count = ATOMIC_INIT(0); /* [NEW] */
static bool stats_enabled = false;

/* [NEW] RTT Tracking Data */
static struct pending_pong pending_pongs[MAX_PENDING_PONGS];
static struct rtt_sample rtt_history[MAX_RTT_HISTORY];
static uint16_t rtt_count = 0;
K_MUTEX_DEFINE(stats_mutex);

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void pkt_stats_init(void)
{
    atomic_set(&gradient_beacon_count, 0);
    atomic_set(&heartbeat_count, 0);
    atomic_set(&data_tx_count, 0);
    atomic_set(&data_fwd_count, 0);
    atomic_set(&route_change_count, 0);
    atomic_set(&rx_data_count, 0); /* [NEW] */
    
    k_mutex_lock(&stats_mutex, K_FOREVER);
    for (int i = 0; i < MAX_PENDING_PONGS; i++) {
        pending_pongs[i].seq = 0;
        pending_pongs[i].send_time = 0;
    }
    rtt_count = 0;
    k_mutex_unlock(&stats_mutex);
    
    LOG_INF("[PktStats] Initialized - all counters reset");
}

void pkt_stats_record_sent(uint16_t seq)
{
    if (!stats_enabled) return;

    k_mutex_lock(&stats_mutex, K_FOREVER);
    
    /* Find empty slot or oldest slot in pending_pongs */
    int slot = -1;
    uint32_t oldest_time = UINT32_MAX;
    int oldest_idx = 0;

    for (int i = 0; i < MAX_PENDING_PONGS; i++) {
        if (pending_pongs[i].send_time == 0) {
            slot = i;
            break;
        }
        if (pending_pongs[i].send_time < oldest_time) {
            oldest_time = pending_pongs[i].send_time;
            oldest_idx = i;
        }
    }

    if (slot == -1) {
        /* Reuse oldest slot if full (stale ping cleanup) */
        slot = oldest_idx;
        LOG_DBG("[PktStats] Pending Pongs full, overwriting oldest (seq=%u)", pending_pongs[slot].seq);
    }

    pending_pongs[slot].seq = seq;
    pending_pongs[slot].send_time = k_uptime_get_32();
    
    k_mutex_unlock(&stats_mutex);
}

bool pkt_stats_record_pong(uint16_t seq)
{
    if (!stats_enabled) return false;
    bool buffer_full = false;

    k_mutex_lock(&stats_mutex, K_FOREVER);

    /* 1. Find the matching pending ping */
    int slot = -1;
    for (int i = 0; i < MAX_PENDING_PONGS; i++) {
        if (pending_pongs[i].send_time != 0 && pending_pongs[i].seq == seq) {
            slot = i;
            break;
        }
    }

    if (slot != -1) {
        uint32_t rtt = k_uptime_get_32() - pending_pongs[slot].send_time;
        
        /* 2. Record in history */
        if (rtt_count < MAX_RTT_HISTORY) {
            rtt_history[rtt_count].seq = seq;
            rtt_history[rtt_count].rtt_ms = (uint16_t)rtt;
            rtt_count++;
            
            if (rtt_count >= MAX_RTT_HISTORY) {
                buffer_full = true;
            }
        }
        
        /* 3. Clear the pending slot */
        pending_pongs[slot].send_time = 0;
        LOG_DBG("[PktStats] PONG received for seq=%u, RTT=%u ms", seq, rtt);
    } else {
        LOG_DBG("[PktStats] PONG received for unknown/expired seq=%u", seq);
    }

    k_mutex_unlock(&stats_mutex);
    return buffer_full;
}

uint16_t pkt_stats_get_rtt_history(struct rtt_sample *buffer, uint16_t max_count)
{
    uint16_t count = 0;
    
    k_mutex_lock(&stats_mutex, K_FOREVER);
    
    count = (rtt_count < max_count) ? rtt_count : max_count;
    if (count > 0 && buffer != NULL) {
        memcpy(buffer, rtt_history, count * sizeof(struct rtt_sample));
    }
    
    k_mutex_unlock(&stats_mutex);
    return count;
}

void pkt_stats_clear_rtt_history(void)
{
    k_mutex_lock(&stats_mutex, K_FOREVER);
    rtt_count = 0;
    k_mutex_unlock(&stats_mutex);
    LOG_DBG("[PktStats] RTT History cleared");
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

void pkt_stats_inc_data_fwd(void)
{
    if (!stats_enabled) return;
    atomic_inc(&data_fwd_count);
}

void pkt_stats_inc_route_change(void)
{
    if (!stats_enabled) return;
    atomic_inc(&route_change_count);
}

void pkt_stats_inc_rx(void)
{
    if (!stats_enabled) return;
    atomic_inc(&rx_data_count);
}

void pkt_stats_get(struct packet_stats *stats)
{
    if (stats == NULL) {
        return;
    }
    
    stats->gradient_beacon_tx = (uint32_t)atomic_get(&gradient_beacon_count);
    stats->heartbeat_tx = (uint32_t)atomic_get(&heartbeat_count);
    stats->data_tx = (uint32_t)atomic_get(&data_tx_count);
    stats->data_fwd_tx = (uint32_t)atomic_get(&data_fwd_count);
    stats->route_change_count = (uint32_t)atomic_get(&route_change_count);
    stats->rx_data_count = (uint32_t)atomic_get(&rx_data_count); /* [NEW] */
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

uint32_t pkt_stats_get_data_fwd(void)
{
    return (uint32_t)atomic_get(&data_fwd_count);
}

uint32_t pkt_stats_get_route_change(void)
{
    return (uint32_t)atomic_get(&route_change_count);
}

uint32_t pkt_stats_get_rx(void)
{
    return (uint32_t)atomic_get(&rx_data_count);
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
    atomic_set(&data_fwd_count, 0);
    atomic_set(&route_change_count, 0);
    atomic_set(&rx_data_count, 0); /* [NEW] */
    
    pkt_stats_clear_rtt_history();
    
    k_mutex_lock(&stats_mutex, K_FOREVER);
    for (int i = 0; i < MAX_PENDING_PONGS; i++) {
        pending_pongs[i].seq = 0;
        pending_pongs[i].send_time = 0;
    }
    k_mutex_unlock(&stats_mutex);

    LOG_INF("[PktStats] All counters, RTT history, and Pending Pongs reset to 0");
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
