/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef GRADIENT_WORK_H
#define GRADIENT_WORK_H

#include "gradient_srv.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize gradient work handlers
 *
 * Initializes all workqueue items for gradient processing.
 * Must be called during model initialization.
 */
void gradient_work_init(void);

/**
 * @brief Start periodic cleanup timer
 *
 * Starts the cleanup work that removes expired neighbors.
 */
void gradient_work_start_cleanup(void);

/**
 * @brief Schedule initial gradient publish
 *
 * Schedules the first gradient beacon to be published after a short delay.
 */
void gradient_work_schedule_initial_publish(void);

/**
 * @brief Schedule gradient processing
 *
 * Queues gradient message for processing in work context.
 *
 * @param gradient_srv Pointer to gradient server instance
 * @param gradient Received gradient value
 * @param sender_addr Address of sender
 * @param rssi RSSI of received message
 */
void gradient_work_schedule_process(struct bt_mesh_gradient_srv *gradient_srv,
                                    uint8_t gradient, uint16_t sender_addr, int8_t rssi);

/**
 * @brief Set global gradient server reference
 *
 * @param gradient_srv Pointer to gradient server instance
 */
void gradient_work_set_srv(struct bt_mesh_gradient_srv *gradient_srv);

#ifdef __cplusplus
}
#endif

#endif /* GRADIENT_WORK_H */
