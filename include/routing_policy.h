/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ROUTING_POLICY_H
#define ROUTING_POLICY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RSSI threshold for accepting gradient messages
 *
 * Messages with RSSI below this value are dropped.
 */
#define RP_RSSI_THRESHOLD (-60)

/**
 * @brief Check if a candidate's RSSI is acceptable
 *
 * Preserves existing heuristic: drop gradients with RSSI below threshold.
 *
 * @param rssi Received signal strength indicator
 *
 * @return true if RSSI is acceptable (>= threshold), false otherwise
 */
bool rp_is_candidate_acceptable(int8_t rssi);

/**
 * @brief Check if received gradient should be processed
 *
 * Preserves existing rule: ignore if received_grad > my_grad.
 *
 * @param received_grad Gradient value from received message
 * @param my_grad Current node's gradient value
 *
 * @return true if gradient should be processed, false if should be ignored
 */
bool rp_should_process_gradient(uint8_t received_grad, uint8_t my_grad);

/**
 * @brief Compare two entries to determine if new is better than old
 *
 * Preserves sorting rule: smaller gradient is better; on tie, higher RSSI is better.
 *
 * @param new_grad New entry's gradient
 * @param new_rssi New entry's RSSI
 * @param old_grad Old entry's gradient
 * @param old_rssi Old entry's RSSI
 *
 * @return true if new entry is better than old entry
 */
bool rp_is_better(uint8_t new_grad, int8_t new_rssi, uint8_t old_grad, int8_t old_rssi);

/**
 * @brief Compute new gradient value from best parent
 *
 * Preserves existing: best_parent_grad + 1 with uint8_t semantics.
 *
 * @param best_parent_grad Gradient value of best parent node
 *
 * @return Computed gradient value (best_parent_grad + 1)
 */
uint8_t rp_compute_new_gradient(uint8_t best_parent_grad);

/**
 * @brief Check if node's gradient should be updated
 *
 * Preserves existing update condition: update if my_grad > best_parent_grad + 1.
 *
 * @param my_grad Current node's gradient value
 * @param best_parent_grad Best parent's gradient value
 *
 * @return true if gradient should be updated, false otherwise
 */
bool rp_should_update_my_gradient(uint8_t my_grad, uint8_t best_parent_grad);

#ifdef __cplusplus
}
#endif

#endif /* ROUTING_POLICY_H */
