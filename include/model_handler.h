/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief Model handler
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C" {
#endif

const struct bt_mesh_comp *model_handler_init(void);

void sink_start_test(void);
void sink_stop_test(void);

void sink_start_stress_test(uint16_t target_addr); /* [NEW] */

#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */
