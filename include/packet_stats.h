/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file packet_stats.h
 * @brief Packet Statistics Module for Control Overhead Tracking
 *
 * Tracks TX counts for:
 * - Gradient Beacon (CONTROL)
 * - Heartbeat (CONTROL)
 * - DATA packets
 *
 * Used to calculate Control Overhead = N_control / (N_control + N_data)
 */

#ifndef PACKET_STATS_H__
#define PACKET_STATS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Packet statistics structure
 */
struct packet_stats {
    uint32_t gradient_beacon_tx;   /**< Gradient Beacon TX count */
    uint32_t heartbeat_tx;         /**< Heartbeat TX count */
    uint32_t data_tx;              /**< DATA packet TX count (Source) */
    uint32_t data_fwd_tx;          /**< DATA packet Forwarded count (Relay) */
    uint32_t route_change_count;   /**< Number of times best parent changed */
    uint32_t rx_data_count;        /**< [NEW] Count received DATA/BACKPROP at destination */
};

/**
 * @brief Initialize packet statistics (reset all counters to 0)
 */
void pkt_stats_init(void);

/**
 * @brief Increment Gradient Beacon TX counter
 */
void pkt_stats_inc_gradient_beacon(void);

/**
 * @brief Increment Heartbeat TX counter
 */
void pkt_stats_inc_heartbeat(void);

/**
 * @brief Increment DATA TX counter
 */
void pkt_stats_inc_data_tx(void);

/**
 * @brief Increment DATA Forwarding counter
 */
void pkt_stats_inc_data_fwd(void);

/**
 * @brief Increment Route Change counter
 */
void pkt_stats_inc_route_change(void);

/**
 * @brief Get current packet statistics
 *
 * @param stats Pointer to struct to fill with current stats
 */
void pkt_stats_get(struct packet_stats *stats);

/**
 * @brief Get Gradient Beacon TX count
 * @return Current count
 */
uint32_t pkt_stats_get_gradient_beacon(void);

/**
 * @brief Get Heartbeat TX count
 * @return Current count
 */
uint32_t pkt_stats_get_heartbeat(void);

/**
 * @brief Get DATA TX count
 * @return Current count
 */
uint32_t pkt_stats_get_data_tx(void);

/**
 * @brief Get DATA Forward count
 * @return Current count
 */
uint32_t pkt_stats_get_data_fwd(void);

/**
 * @brief Get Route Change count
 * @return Current count
 */
uint32_t pkt_stats_get_route_change(void);

/**
 * @brief Increment RX Data counter
 */
void pkt_stats_inc_rx(void);

/**
 * @brief Get RX Data count
 * @return Current count
 */
uint32_t pkt_stats_get_rx(void);

/**
 * @brief Get total CONTROL packet count (Beacon + Heartbeat)
 * @return Total control packet count
 */
uint32_t pkt_stats_get_control_total(void);

/**
 * @brief Reset all counters to 0
 */
void pkt_stats_reset(void);

/**
 * @brief Enable or disable packet statistics collection
 * @param enable true to enable, false to disable
 */
void pkt_stats_set_enabled(bool enable);

/**
 * @brief Check if statistics collection is currently enabled
 * @return true if enabled, false otherwise
 */
bool pkt_stats_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_STATS_H__ */
