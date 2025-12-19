#ifndef BT_MESH_GRADIENT_SRV_H__
#define BT_MESH_GRADIENT_SRV_H__

#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/model_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* .. include_startingpoint_gradient_srv_rst_1 */
#define BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID    CONFIG_BT_COMPANY_ID_NORDIC

/** Model ID of the Gradient Server model. */
#define BT_MESH_GRADIENT_SRV_VENDOR_MODEL_ID      0x000A

/** Gradient message opcode. */
#define BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS BT_MESH_MODEL_OP_3(0x0A, \
				       BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/** Data message opcode. */
#define BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE BT_MESH_MODEL_OP_3(0x0B, \
				       BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* .. include_endpoint_gradient_srv_rst_1 */

#define BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE 1
#define BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE (\
				     CONFIG_BT_MESH_GRADIENT_SRV_MESSAGE_LENGTH \
				     + 1) /* + \0 */

#ifndef CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS
#define CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS 30000  // 30 giây
#endif

typedef struct bt_mesh_gradient_srv_forwarding_ctx{
	uint16_t addr;
	int8_t rssi;
	uint8_t gradient;
	int64_t last_seen;  
} bt_mesh_gradient_srv_forwarding_ctx;

/* Forward declaration of the Bluetooth Mesh Chat Client model context. */
struct bt_mesh_gradient_srv;

/* .. include_startingpoint_gradient_srv_rst_2 */
/** @def BT_MESH_MODEL_GRADIENT_SRV
 *
 * @brief Bluetooth Mesh Chat Client model composition data entry.
 *
 * @param[in] _gradient Pointer to a @ref bt_mesh_gradient_srv instance.
 */
#define BT_MESH_MODEL_GRADIENT_SRV(_gradient)                                          \
		BT_MESH_MODEL_VND_CB(BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID,       \
			BT_MESH_GRADIENT_SRV_VENDOR_MODEL_ID,                      \
			_bt_mesh_gradient_srv_op, &(_gradient)->pub,                   \
			BT_MESH_MODEL_USER_DATA(struct bt_mesh_gradient_srv,       \
						_gradient),                        \
			&_bt_mesh_gradient_srv_cb)
/* .. include_endpoint_gradient_srv_rst_2 */

/** Bluetooth Mesh Chat Client model handlers. */
struct bt_mesh_gradient_srv_handlers {
	/** @brief Called after the node has been provisioned, or after all
	 * mesh data has been loaded from persistent storage.
	 *
	 * @param[in] cli Chat Client instance that has been started.
	 */
	void (*const start)(struct bt_mesh_gradient_srv *chat);
};

/* .. include_startingpoint_gradient_srv_rst_3 */
/**
 * Bluetooth Mesh Chat Client model context.
 */
struct bt_mesh_gradient_srv {
	/** Access model pointer. */
	const struct bt_mesh_model *model;
	/** Publish parameters. */
	struct bt_mesh_model_pub pub;
	/** Publication message. */
	struct net_buf_simple pub_msg;
	/** Publication message buffer. */
	uint8_t buf[BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
					  BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE)];
	/** Handler function structure. */
	const struct bt_mesh_gradient_srv_handlers *handlers;

	uint8_t gradient;

	bt_mesh_gradient_srv_forwarding_ctx
		forwarding_table[
			CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE];
};
/* .. include_endpoint_gradient_srv_rst_3 */

/** @brief Send Gradient Beacon.
 *
 * @param[in] cli Chat Client model instance to send the message.
 * @param[in] msg Pointer to a text message to send. Must be terminated with
 * a null character, '\0'.
 *
 * @retval 0 Successfully sent the message.
 * @retval -EADDRNOTAVAIL Publishing is not configured.
 * @retval -EAGAIN The device has not been provisioned.
 */
int bt_mesh_gradient_srv_gradient_send(struct bt_mesh_gradient_srv *gradient_srv);

/** @brief Send a text message to a specified destination.
 *
 * @param[in] cli  Chat Client model instance to send the message.
 * @param[in] addr Address of the chat client to send message to.
 * @param[in] msg  Pointer to a text message to send. Must be terminated with
 * a null character, '\0'.
 *
 * @retval 0 Successfully sent the message.
 * @retval -EINVAL The model is not bound to an application key.
 * @retval -EAGAIN The device has not been provisioned.
 */
int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
					  uint16_t addr,
					  uint16_t data);

/** @cond INTERNAL_HIDDEN */
extern const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[];
extern const struct bt_mesh_model_cb _bt_mesh_gradient_srv_cb;
/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* BT_MESH_GRADIENT_SRV_H__ */

/** @} */
