/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "routing_policy.h"

bool rp_is_candidate_acceptable(int8_t rssi)
{
	/* Preserve existing heuristic from handle_gradient_mesage:
	 * if (rssi < -60) { return 0; }
	 */
	return (rssi >= RP_RSSI_THRESHOLD);
}

bool rp_should_process_gradient(uint8_t received_grad, uint8_t my_grad)
{
	/* Preserve existing rule from handle_gradient_mesage:
	 * if (msg > gradient_srv->gradient) { return 0; }
	 */
	return (received_grad <= my_grad);
}

bool rp_is_better(uint8_t new_grad, int8_t new_rssi, uint8_t old_grad, int8_t old_rssi)
{
	/* Preserve sorting rule from gradient_process_handler:
	 * - Smaller gradient is better
	 * - On equal gradient, higher RSSI is better
	 */
	if (new_grad < old_grad) {
		return true;
	}

	if (new_grad == old_grad && new_rssi > old_rssi) {
		return true;
	}

	return false;
}

uint8_t rp_compute_new_gradient(uint8_t best_parent_grad)
{
	/* Compute new gradient = best_parent_grad + 1
	 * 
	 * Special cases:
	 * - If best_parent_grad = 255, result would overflow to 0
	 * - But only SINK node can have gradient=0
	 * - So clamp to 254 for non-sink nodes
	 */
	if (best_parent_grad >= 254) {
		/* Prevent overflow to 0 (reserved for Gateway) */
		return 254;
	}
	return (uint8_t)(best_parent_grad + 1);
}

bool rp_should_update_my_gradient(uint8_t my_grad, uint8_t best_parent_grad)
{
	/* Preserve existing update condition from gradient_process_handler:
	 * if (gradient_srv->gradient > best_parent_gradient + 1) { ... }
	 */
	return (my_grad > (uint8_t)(best_parent_grad + 1));
}
