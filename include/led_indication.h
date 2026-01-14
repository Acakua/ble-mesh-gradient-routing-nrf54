/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef LED_INDICATION_H__
#define LED_INDICATION_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LED indication module
 *
 * Must be called before using any other LED indication functions.
 */
void led_indication_init(void);

/**
 * @brief Indicate gradient message received
 *
 * Blinks LED to show a gradient beacon was received from a neighbor.
 */
void led_indicate_gradient_received(void);

/**
 * @brief Indicate data is being forwarded
 *
 * Blinks LED to show data packet is being forwarded towards sink.
 */
void led_indicate_data_forwarded(void);

/**
 * @brief Indicate data arrived at sink
 *
 * Toggles LED to show data packet reached the sink node.
 */
void led_indicate_sink_received(void);

/**
 * @brief Indicate backprop data received at destination
 *
 * Toggles LED 0 to show backprop packet reached the destination node.
 */
void led_indicate_backprop_received(void);

/**
 * @brief Indicate mesh attention state
 *
 * @param on true to start attention indication, false to stop
 */
void led_indicate_attention(bool on);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATION_H */
