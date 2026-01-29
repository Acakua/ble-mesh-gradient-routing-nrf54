#ifndef BT_MESH_GRADIENT_SRV_H__
#define BT_MESH_GRADIENT_SRV_H__

#include <zephyr/bluetooth/mesh.h>
#include <bluetooth/mesh/model_types.h>
#include "gradient_types.h"

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

/** Backprop data message opcode (downlink from Gateway to nodes). */
#define BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA BT_MESH_MODEL_OP_3(0x0C, \
                        BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* [NEW] Report Request opcode (Broadcast from Sink to Nodes - STOP & REPORT) */
#define BT_MESH_GRADIENT_SRV_OP_REPORT_REQ BT_MESH_MODEL_OP_3(0x0D, \
                        BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* [NEW] Report Response opcode (Unicast from Node to Sink with TotalTx) */
#define BT_MESH_GRADIENT_SRV_OP_REPORT_RSP BT_MESH_MODEL_OP_3(0x0E, \
                        BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* [NEW] Test Start opcode (Broadcast from Sink to Nodes - START) */
#define BT_MESH_GRADIENT_SRV_OP_TEST_START BT_MESH_MODEL_OP_3(0x0F, \
                        BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* [NEW] Report ACK opcode (Unicast from Sink to Node - Reliability) */
#define BT_MESH_GRADIENT_SRV_OP_REPORT_ACK BT_MESH_MODEL_OP_3(0x10, \
                        BT_MESH_GRADIENT_SRV_VENDOR_COMPANY_ID)

/* .. include_endpoint_gradient_srv_rst_1 */

/** Default TTL for BACKPROP packets */
#define BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL  10

/** Minimum TTL to forward (drop if TTL <= this value) */
#define BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL      1

/** Heartbeat data marker - distinguishes heartbeat from real data */
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER      0xFFFF

#define BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE 1
#define BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE 20 /* Ensure enough for DATA+TS (9 bytes) */

#define REPORT_RETRY_TIMEOUT_MS 3000 // Tăng thời gian chờ cơ bản
#define REPORT_MAX_RETRIES      10   // Tăng số lần thử lại tối đa

#ifndef CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS
#define CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS 30000  // 30 giây
#endif

/**
 * @brief Forwarding table context - alias to neighbor_entry_t
 * * Uses neighbor_entry_t from gradient_types.h for consistency across modules.
 */
typedef neighbor_entry_t bt_mesh_gradient_srv_forwarding_ctx;

/* Forward declaration of the Bluetooth Mesh Chat Client model context. */
struct bt_mesh_gradient_srv;

/* .. include_startingpoint_gradient_srv_rst_2 */
/** @def BT_MESH_MODEL_GRADIENT_SRV
 *
 * @brief Bluetooth Mesh Chat Client model composition data entry.
 *
 * @param[in] _gradient Pointer to a @ref bt_mesh_gradient_srv instance.
 */
#define BT_MESH_MODEL_GRADIENT_SRV(_gradient)                                  \
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

    /** @brief Called when BACKPROP_DATA is received and this node is the destination.
     *
     * @param[in] srv Gradient server instance.
     * @param[in] data The received data payload.
     */
    void (*const data_received)(struct bt_mesh_gradient_srv *srv, uint16_t data);

    /** [NEW] Called when REPORT_REQ is received (Triggered on Sensor Nodes)
     *
     * The application should handle this by stopping transmission and 
     * sending back a REPORT_RSP containing the Total Transmitted Count.
     *
     * @param[in] srv Gradient server instance.
     */
    void (*const report_req_received)(struct bt_mesh_gradient_srv *srv);

    /** [NEW] Called when TEST_START is received (Triggered on Sensor Nodes)
     *
     * The application should handle this by starting the periodic data transmission.
     *
     * @param[in] srv Gradient server instance.
     */
    void (*const test_start_received)(struct bt_mesh_gradient_srv *srv);
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

    /** Mutex to protect forwarding table access */
    struct k_mutex forwarding_table_mutex;

    bt_mesh_gradient_srv_forwarding_ctx
        forwarding_table[
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE];

    /* Reliable Reporting Context */
    struct k_work_delayable report_retry_work;
    uint8_t report_retry_count;
    bool is_report_pending;
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
                      uint16_t data,
                      uint32_t timestamp);

/** @brief Send BACKPROP_DATA to a specific destination.
 *
 * Looks up the reverse routing table to find nexthop, then sends
 * BACKPROP_DATA packet towards the destination.
 *
 * @param gradient_srv Pointer to gradient server instance.
 * @param dest_addr Address of final destination node.
 * @param payload Data payload to send.
 *
 * @retval 0 Successfully sent the message.
 * @retval -EINVAL Cannot send to self.
 * @retval -ENETUNREACH No route to destination.
 * @retval -EAGAIN The device has not been provisioned.
 */
int bt_mesh_gradient_srv_backprop_send(struct bt_mesh_gradient_srv *gradient_srv,
                                       uint16_t dest_addr,
                                       uint16_t payload);

/** @brief [NEW] Broadcast a REPORT REQUEST to all nodes.
 *
 * This function should be called by the SINK NODE to trigger the 
 * end-of-test reporting phase (STOP).
 *
 * @param gradient_srv Pointer to gradient server instance.
 * @retval 0 Successfully sent.
 */
int bt_mesh_gradient_srv_send_report_req(struct bt_mesh_gradient_srv *gradient_srv);

/** @brief [NEW] Broadcast a TEST START command to all nodes.
 *
 * This function should be called by the SINK NODE to trigger the 
 * start of data transmission.
 *
 * @param gradient_srv Pointer to gradient server instance.
 * @retval 0 Successfully sent.
 */
int bt_mesh_gradient_srv_send_test_start(struct bt_mesh_gradient_srv *gradient_srv);

/** @brief [NEW] Send a REPORT RESPONSE (Unicast) to the Sink.
 *
 * This function should be called by the SENSOR NODE to report its
 * Total Transmitted Count (PDR calculation).
 *
 * @param gradient_srv Pointer to gradient server instance.
 * @param total_tx Total packets sent during the test.
 * @retval 0 Successfully sent.
 */
int bt_mesh_gradient_srv_report_rsp_send(struct bt_mesh_gradient_srv *gradient_srv);

/** @cond INTERNAL_HIDDEN */
extern const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[];
extern const struct bt_mesh_model_cb _bt_mesh_gradient_srv_cb;
/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* BT_MESH_GRADIENT_SRV_H__ */

/** @} */