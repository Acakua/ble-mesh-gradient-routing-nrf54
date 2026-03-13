#include "gradient_srv.h"
#include "mesh/net.h"
#include <limits.h>
#include <string.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>

#include "data_forward.h"
#include "gradient_work.h"
#include "led_indication.h"
#include "neighbor_table.h"
#include "packet_stats.h"
#include "reverse_routing.h"
#include "routing_policy.h"
#include <zephyr/bluetooth/mesh/statistic.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>


LOG_MODULE_REGISTER(gradient_srv, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * DEFINES
 * ------------------------------------------------------------------------- */
#ifndef BT_MESH_GRADIENT_SRV_DEFAULT_TTL
#define BT_MESH_GRADIENT_SRV_DEFAULT_TTL 10
#endif

#ifndef BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER 0xFFFF
#endif

BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
                 BT_MESH_RX_SDU_MAX,
             "The message must fit inside an application SDU.");
BUILD_ASSERT(BT_MESH_MODEL_BUF_LEN(BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
                                   BT_MESH_GRADIENT_SRV_MSG_MAXLEN_MESSAGE) <=
                 BT_MESH_TX_SDU_MAX,
             "The message must fit inside an application SDU.");

/* -------------------------------------------------------------------------
 * STATIC VARIABLES (SEQUENCE NUMBERS & DEDUP)
 * ------------------------------------------------------------------------- */
/* 1. Cho lệnh TEST START */
static uint8_t last_processed_test_id = 0xFF;
static uint8_t current_tx_test_id = 0;

/* 2. Cho lệnh REPORT REQ (STOP COMMAND) */
static uint8_t last_processed_req_id = 0xFF;
static uint8_t current_tx_req_id = 0;

/* [RELATIVE LATENCY] External reference to test start time */
extern uint32_t g_test_start_time;

/* -------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * ------------------------------------------------------------------------- */
static int srv_send_msg_with_stat(struct bt_mesh_gradient_srv *srv,
                                  struct bt_mesh_msg_ctx *ctx,
                                  struct net_buf_simple *msg) {
  int err = bt_mesh_model_send(srv->model, ctx, msg, NULL, NULL);

  /* If send fails at Application level (Buffer Full, Queue Full, etc.)
   * increment Soft Drop counter. We ignore -EAGAIN as it's often transient
   * but -ENOMEM or -EBUSY are definitive drops in this context. */
  if (err && err != -EAGAIN) {
    srv->soft_drop_count++;
    LOG_DBG("Soft Drop detected (err %d), total: %u", err, srv->soft_drop_count);
  }

  return err;
}

/******************************************************************************/
/* Message Handlers                                                           */
/******************************************************************************/

static int handle_gradient_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
  uint8_t msg;
  int8_t rssi = ctx->recv_rssi;
  uint16_t sender_addr = ctx->addr;

  /* Skip messages from self */
  if (sender_addr == bt_mesh_model_elem(gradient_srv->model)->rt->addr) {
    return 0;
  }

  /* Check RSSI threshold using routing_policy */
  if (!rp_is_candidate_acceptable(rssi)) {
    return 0;
  }

  /* LED indication */
  led_indicate_gradient_received();

  msg = net_buf_simple_pull_u8(buf);

  LOG_INF("[CONTROL - Gradient Beacon] Received from: 0x%04x, Gradient: %d",
          sender_addr, msg);

  /* Check if gradient should be processed using routing_policy */
  if (!rp_should_process_gradient(msg, gradient_srv->gradient)) {
    return 0;
  }

  /* Schedule processing in work context */
  gradient_work_schedule_process(gradient_srv, msg, sender_addr, rssi);

  return 0;
}

static int handle_pong_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf);

static int handle_gradient_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf);

static void rrt_update_from_uplink_msg(struct bt_mesh_gradient_srv *srv,
                                       uint16_t sender_addr,
                                       uint16_t original_source, int8_t rssi,
                                       int64_t now) {
  k_mutex_lock(&srv->forwarding_table_mutex, K_FOREVER);

  /* 1. Update/Add Neighbor Table (Robust Discovery)
   * Even if we missed the Gradient Beacon, we accept the node into the table
   * based on Uplink traffic (DATA/Heartbeat/TOPO) to ensure RRT learning.
   */
  uint8_t sender_gradient = UINT8_MAX;
  for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
    if (srv->forwarding_table[i].addr == sender_addr) {
      sender_gradient = srv->forwarding_table[i].gradient;
      break;
    }
  }

  nt_update_sorted((neighbor_entry_t *)srv->forwarding_table,
                   CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                   sender_addr, sender_gradient, rssi, now);

  /* 2. Reverse Route Learning (RRT) */
  rrt_add_dest(srv->forwarding_table,
               CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
               sender_addr,     /* nexthop */
               original_source, /* destination */
               now);

  k_mutex_unlock(&srv->forwarding_table_mutex);
}

static int handle_data_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
  uint16_t sender_addr = ctx->addr;
  int8_t rssi = ctx->recv_rssi;

  /* [DIAGNOSTIC] Hex-dump incoming DATA message */
  LOG_HEXDUMP_INF(buf->data, buf->len, "DATA RX Payload:");

  /* Bóc tách các trường dữ liệu theo đúng thứ tự đóng gói */
  uint16_t original_source = net_buf_simple_pull_le16(buf);
  uint16_t received_data = net_buf_simple_pull_le16(buf);

  /* [UPDATED] Bóc tách nội dung theo format 7 bytes */
  /* TTL Placeholder (1 byte) - Byte 4 */
  (void)net_buf_simple_pull_u8(buf);

  /* Hop Count (1 byte) - Byte 5 */
  uint8_t hop_count = net_buf_simple_pull_u8(buf);

  /* [REMOVED] Timestamp bóc tách cũ */

  /* [NEW] Bóc tách Min RSSI (1 byte) */
  int8_t path_min_rssi = (int8_t)net_buf_simple_pull_u8(buf);

  uint32_t delay_ms = 0; /* Sẽ được tính bằng Ping-Pong sau này */

  /* Logging Logic */
  if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
    LOG_INF("[CONTROL - Heartbeat] Recv from 0x%04x (via 0x%04x), Hops: %d",
            original_source, sender_addr, hop_count);
  } else {
    LOG_INF("[DATA - Sensor] Recv from 0x%04x (via 0x%04x), Seq: %d, Hops: %d, "
            "MinRSSI: %d",
            original_source, sender_addr, received_data, hop_count,
            path_min_rssi);
  }

  int64_t now = k_uptime_get();

  /* [ROBUST RRT] Update/Add RRT route for ANY Uplink message (Data/Heartbeat)
   */
  rrt_update_from_uplink_msg(gradient_srv, sender_addr, original_source, rssi,
                             now);

  /* ════════════════════════════════════════════════════════════════════
   * Forwarding Logic & LOGGING
   * ════════════════════════════════════════════════════════════════════ */

  if (gradient_srv->gradient == 0) {
    /* I AM THE SINK (Gateway) */
    led_indicate_sink_received();

    /* [MODIFIED] Log thêm Delay vào CSV (Chỉ log khi phiên test đang chạy) */
    if (pkt_stats_is_enabled()) {
      if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
        /* [UPDATED] Format: TYPE, Src, Sender, Hops, DelayMs */
        printk("CSV_LOG,HEARTBEAT,0x%04x,0x%04x,%d,%u\n", original_source,
               sender_addr, hop_count, delay_ms);
      } else {
        /* [NEW] Cập nhật với RSSI chặng cuối cùng về Sink */
        if (rssi < path_min_rssi) {
          path_min_rssi = rssi;
        }
        /* [UPDATED] Format: TYPE, Src, Sender, Seq, Hops, DummyDelay,
         * PathMinRSSI */
        printk("CSV_LOG,DATA,0x%04x,0x%04x,%d,%d,0,%d\n", original_source,
               sender_addr, received_data, hop_count, path_min_rssi);

        /* [NEW] Send PONG back to original source */
        bt_mesh_gradient_srv_send_pong(gradient_srv, original_source,
                                       received_data);

        if (gradient_srv->handlers->data_received) {
          gradient_srv->handlers->data_received(gradient_srv, received_data);
        }
      }
    }

  } else {
    /* I AM A RELAY NODE */

    /* [MODIFIED] Check if we should forward? (Heartbeat -> NO, DATA -> YES) */
    if (received_data != BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
      /* [NEW] Cập nhật Path Min RSSI trước khi tiếp tục */
      if (rssi < path_min_rssi) {
        path_min_rssi = rssi;
      }

      /* Forwarding Packet with updated Min RSSI (Timestamp removed) */
      int err = data_forward_send(gradient_srv, received_data, original_source,
                                  sender_addr, hop_count, path_min_rssi);
      if (err) {
        LOG_WRN("[Forward] Failed to forward seq=%d, err=%d", received_data,
                err);
      }
    }
  }

  return 0;
}

/**
 * @brief Handle BACKPROP_DATA message (Gateway -> Node downlink)
 */
static int handle_backprop_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
  uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;
  uint16_t sender_addr = ctx->addr;

  uint16_t final_dest = net_buf_simple_pull_le16(buf);
  uint16_t payload = net_buf_simple_pull_le16(buf);
  uint8_t ttl = net_buf_simple_pull_u8(buf);
  uint8_t hop_count = net_buf_simple_pull_u8(buf);
  uint32_t timestamp = net_buf_simple_pull_le32(buf);
  int8_t path_min_rssi = (int8_t)net_buf_simple_pull_u8(buf);

  /* Update path min RSSI */
  int8_t current_rssi = ctx->recv_rssi;
  if (current_rssi < path_min_rssi) {
    path_min_rssi = current_rssi;
  }

  LOG_INF("[CONTROL - Backprop] Recv: dest=0x%04x, payload=%d, hops=%d, "
          "from=0x%04x",
          final_dest, payload, hop_count, sender_addr);

  if (final_dest == my_addr) {
    /* [RELATIVE LATENCY] */
    uint32_t now_rel =
        k_uptime_get_32() -
        (g_test_start_time > 0 ? g_test_start_time : k_uptime_get_32());
    uint32_t delay_ms = (now_rel >= timestamp) ? (now_rel - timestamp) : 0;

    if (payload == 0xFFFE) {
      LOG_WRN("[CONTROL] Received REPORT REQ (via Backprop)!");
      if (gradient_srv->handlers->report_req_received) {
        gradient_srv->handlers->report_req_received(gradient_srv);
      }
    } else if (payload == 0xFFFD) {
      LOG_INF("[CONTROL] Received STATS RESET (via Backprop)!");
      pkt_stats_reset();
    } else {
      /* [NEW] CSV LOG AT SENSOR FOR DOWNLINK */
      printk("CSV_LOG,BACKPROP,0x%04x,0x%04x,%d,%d,%u,%d\n", my_addr,
             sender_addr, payload, hop_count, delay_ms, path_min_rssi);

      LOG_INF("[CONTROL - Backprop] Destination Reached! Payload: %d, Hops: "
              "%d, Delay: %u ms",
              payload, hop_count, delay_ms);
      pkt_stats_inc_rx();
      if (gradient_srv->handlers->data_received) {
        gradient_srv->handlers->data_received(gradient_srv, payload);
      }
    }
    return 0;
  }

  if (ttl <= 1) {
    return 0;
  }

  uint16_t nexthop = rrt_find_nexthop(
      gradient_srv->forwarding_table,
      CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE, final_dest);

  if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
    LOG_WRN("[CONTROL - Backprop] No route to dest=0x%04x", final_dest);
    return 0;
  }

  /* Forwarding: Cấu trúc mới 11 bytes */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 11);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);

  /* Re-pack for next hop */
  net_buf_simple_add_le16(&msg, final_dest);
  net_buf_simple_add_le16(&msg, payload);
  net_buf_simple_add_u8(&msg, ttl - 1);
  net_buf_simple_add_u8(&msg, hop_count + 1);
  net_buf_simple_add_le32(&msg, timestamp);
  net_buf_simple_add_u8(&msg, (uint8_t)path_min_rssi);

  struct bt_mesh_msg_ctx tx_ctx = {
      .app_idx = gradient_srv->model->keys[0],
      .addr = nexthop,
      .send_ttl = 0,
      .send_rel = true, /* [FIX] Reliable delivery for Backprop */
  };

  srv_send_msg_with_stat(gradient_srv, &tx_ctx, &msg);
  return 0;
}

static int handle_downlink_report(const struct bt_mesh_model *model,
                                  struct bt_mesh_msg_ctx *ctx,
                                  struct net_buf_simple *buf) {
  uint16_t sink_addr = ctx->addr;
  uint16_t total_tx_by_sink = net_buf_simple_pull_le16(buf);

  struct packet_stats stats;
  pkt_stats_get(&stats);

  /* [MATCHING SINK_LOGGER] Print report in standard format at Sensor */
  /* Format: TYPE, Src, TotalTx, TotalRx, ... (simplified for Downlink) */
  printk("CSV_LOG,REPORT,0x%04x,%u,%u,0,0,0,0\n", sink_addr, total_tx_by_sink,
         stats.rx_data_count);

  LOG_INF(">>> DOWNLINK REPORT RECEIVED FROM SINK 0x%04x: PDR %u/%u <<<",
          sink_addr, stats.rx_data_count, total_tx_by_sink);

  return 0;
}

/**
 * @brief [UPDATED] Handle REPORT REQUEST (Broadcast from Sink)
 * Triggered at Sensor Nodes (STOP command)
 * Logic: Receive -> Dedup -> Action -> Re-broadcast
 */
static int handle_report_req(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  /* 1. Kiểm tra Payload (1 byte ID) */
  if (buf->len < 1) {
    return -EINVAL;
  }
  uint8_t received_req_id = net_buf_simple_pull_u8(buf);

  /* 2. Duplicate Check */
  if (received_req_id == last_processed_req_id) {
    /* Đã xử lý lệnh này rồi */
    return 0;
  }

  LOG_WRN(">>> RX STOP/REPORT REQ (ID: %d) from 0x%04x <<<", received_req_id,
          ctx->addr);

  /* 3. Update State */
  last_processed_req_id = received_req_id;

  /* 4. Action: Notify app to stop sending */
  if (srv->handlers->report_req_received) {
    srv->handlers->report_req_received(srv);
  }

  /* 5. Re-broadcast (Controlled Flooding) */
  /* Chỉ re-broadcast nếu TTL còn và tôi không phải là Sink */
  if (srv->gradient != 0 && ctx->recv_ttl > 1) {
    LOG_DBG("Re-broadcasting STOP REQ ID %d...", received_req_id);

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, 1);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ);
    net_buf_simple_add_u8(&msg, received_req_id);

    struct bt_mesh_msg_ctx send_ctx = {
        .app_idx = model->keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES,
        .send_ttl = ctx->recv_ttl - 1,
    };

    (void)srv_send_msg_with_stat(srv, &send_ctx, &msg);
  }

  return 0;
}

/**
 * @brief [NEW] Handle REPORT REQUEST UNICAST (Unicast from Sink)
 * Triggered at Sensor Nodes (STOP & REPORT specific node)
 * Logic: Receive -> Action (No re-broadcast)
 */
static int handle_report_req_unicast(const struct bt_mesh_model *model,
                                     struct bt_mesh_msg_ctx *ctx,
                                     struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  LOG_WRN(">>> RX UNICAST REPORT REQ from 0x%04x <<<", ctx->addr);

  /* Action: Notify app to stop sending/report */
  if (srv->handlers->report_req_received) {
    srv->handlers->report_req_received(srv);
  }

  /* The app handler triggers report_retry_work which sends REPORT_RSP */

  return 0;
}

/**
 * @brief [UPDATED] Handle TEST START (Broadcast/Flood from Sink)
 * Triggered at Sensor Nodes (START command)
 * Logic: Receive -> Dedup -> Action -> Re-broadcast
 */
static int handle_test_start(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  /* ----------------------------------------------------------------------
   * [FIX QUAN TRỌNG] CHẶN LOOPBACK (TIẾNG VỌNG)
   * ----------------------------------------------------------------------
   * Kiểm tra xem địa chỉ người gửi (SRC) có trùng với địa chỉ của chính tôi
   * không. Nếu trùng: Nghĩa là gói tin này do TÔI phát ra, được mạng relay
   * ngược lại. HÀNH ĐỘNG: Bỏ qua ngay lập tức để không Reset lại timer.
   */
  uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
  if (ctx->addr == my_addr) {
    LOG_WRN("Ignored Loopback TEST_START from self (0x%04x)", ctx->addr);
    return 0;
  }
  /* ---------------------------------------------------------------------- */

  /* 1. Kiểm tra độ dài và lấy Test ID */
  if (buf->len < 1) {
    return -EINVAL;
  }
  uint8_t received_test_id = net_buf_simple_pull_u8(buf);

  /* 2. Duplicate Check (Chống xử lý lặp lại ID cũ từ node khác) */
  if (received_test_id == last_processed_test_id) {
    /* Đã xử lý lệnh này rồi, bỏ qua */
    return 0;
  }

  LOG_INF(">>> RX TEST START (ID: %d) from 0x%04x <<<", received_test_id,
          ctx->addr);

  /* 3. Cập nhật trạng thái để không xử lý lại */
  last_processed_test_id = received_test_id;

  /* 4. Notify application to start sending */
  if (srv->handlers->test_start_received) {
    srv->handlers->test_start_received(srv);
  }

  /* 5. Re-broadcast (Controlled Flooding) */
  /* Điều kiện: TTL còn đủ và tôi không phải là Sink
   * (Sink đã phát rồi, không cần re-broadcast lại cái mình vừa nhận)
   */
  if (srv->gradient != 0 && ctx->recv_ttl > 1) {

    LOG_DBG("Re-broadcasting TEST START ID %d...", received_test_id);

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TEST_START, 1);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TEST_START);
    net_buf_simple_add_u8(&msg, received_test_id);

    struct bt_mesh_msg_ctx send_ctx = {
        .app_idx = model->keys[0],
        .addr = BT_MESH_ADDR_ALL_NODES, // Gửi cho tất cả hàng xóm
        .send_ttl = ctx->recv_ttl - 1,  // Giảm TTL đi 1
    };

    (void)srv_send_msg_with_stat(srv, &send_ctx, &msg);
  }

  return 0;
}

/**
 * @brief [NEW] Handle REPORT RESPONSE (Unicast from Node)
 * Triggered at Sink Node
 */
/* FORWARD DECLARATION for public API if needed, but we can access srv directly
 * if passed */

static void report_retry_handler(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct bt_mesh_gradient_srv *srv =
      CONTAINER_OF(dwork, struct bt_mesh_gradient_srv, report_retry_work);

  if (!srv->is_report_pending)
    return;

  /* [FIX] Reset count và giải phóng khóa khi đã thử đủ số lần mà không thành
   * công */
  if (srv->report_retry_count >= REPORT_MAX_RETRIES) {
    LOG_ERR("Max retries for REPORT reached. Giving up.");
    srv->is_report_pending = false;
    srv->report_retry_count = 0; // Reset để chu kỳ sau bắt đầu lại từ đầu

    /* [FIX] Clear RTT buffer to allow future triggers */
    pkt_stats_clear_rtt_history();
    return;
  }

  srv->report_retry_count++;
  LOG_WRN("Resending REPORT_RSP (Retry %u)...", srv->report_retry_count);

  /* Re-send Report */
  /* [NEW] Fetch RTT History */
  struct rtt_sample rtt_history[50];
  uint16_t rtt_count = pkt_stats_get_rtt_history(rtt_history, 50);

  /* [UPDATED] Payload size Max 215 bytes */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP, 220);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP);

  /* Gắn thêm địa chỉ nguồn gốc (Reporter) vào payload để Relay/Sink nhận diện
   * đúng */
  uint16_t my_addr = bt_mesh_model_elem(srv->model)->rt->addr;
  net_buf_simple_add_le16(&msg, my_addr);

  struct packet_stats stats;
  pkt_stats_get(&stats);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.data_tx);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.gradient_beacon_tx);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.heartbeat_tx);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.route_change_count);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.data_fwd_tx);
  net_buf_simple_add_le16(&msg, (uint16_t)stats.rx_data_count);

  /* [NEW] Append RTT Data */
  net_buf_simple_add_u8(&msg, (uint8_t)rtt_count);
  for (int i = 0; i < rtt_count; i++) {
    net_buf_simple_add_le16(&msg, rtt_history[i].seq);
    net_buf_simple_add_le16(&msg, rtt_history[i].rtt_ms);
  }

  /* [DYNAMIC PARENT SWITCHING] - GIỮ NGUYÊN */
  uint16_t target_parent = BT_MESH_ADDR_UNASSIGNED;
  int parent_idx = 0;

  if (srv->report_retry_count >= 6) {
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE;
         i++) {
      if (srv->forwarding_table[i].gradient < srv->gradient &&
          srv->forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
        target_parent = srv->forwarding_table[i].addr;
        parent_idx = i;
        if (i >= (srv->report_retry_count % 3))
          break;
      }
    }
  } else if (srv->report_retry_count >= 3) {
    if (srv->forwarding_table[1].gradient < srv->gradient &&
        srv->forwarding_table[1].addr != BT_MESH_ADDR_UNASSIGNED) {
      target_parent = srv->forwarding_table[1].addr;
      parent_idx = 1;
    }
  }

  if (target_parent == BT_MESH_ADDR_UNASSIGNED) {
    target_parent = srv->forwarding_table[0].addr;
    parent_idx = 0;
  }

  struct bt_mesh_msg_ctx ctx = {
      .app_idx = srv->model->keys[0],
      .addr = target_parent,
      .send_ttl = BT_MESH_TTL_DEFAULT,
      .send_rel = true,
  };

  if (target_parent != BT_MESH_ADDR_UNASSIGNED) {
    LOG_WRN("Resending REPORT_RSP (Retry %u) to Parent [%d]: 0x%04x",
            srv->report_retry_count, parent_idx, target_parent);

    /* [UPDATED] Gửi Report đi nhưng KHÔNG dừng retry ngay lập tức.
     * Chỉ dừng khi nhận được REPORT_ACK từ Sink (xử lý ở handle_report_ack).
     */
    int err = srv_send_msg_with_stat(srv, &ctx, &msg);
    if (err) {
      LOG_ERR("Retry %u: Send failed (err %d)", srv->report_retry_count, err);
    }
  } else {
    LOG_ERR("Retry %u: No parent to send report!", srv->report_retry_count);
  }

  /* Scheduler Retry tiếp theo (để đảm bảo tin cậy) */
  uint32_t jitter = sys_rand32_get() % 1000;
  /* [FIX] Tăng thời gian chờ retry lên 4-5s để tránh nghẽn mạng */
  k_work_schedule(&srv->report_retry_work, K_MSEC(4000 + jitter));
}

static int handle_report_ack(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;
  uint16_t target_addr = net_buf_simple_pull_le16(buf);
  uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

  if (target_addr == my_addr) {
    LOG_INF("Received REPORT_ACK! Stopping retry and clearing RTT history.");
    k_work_cancel_delayable(&srv->report_retry_work);
    srv->is_report_pending = false;

    /* [NEW] Clear history because it has been acknowledged */
    pkt_stats_clear_rtt_history();
    return 0;
  }

  /* Forwarding Logic using RRT */
  uint16_t nexthop = rrt_find_nexthop(
      srv->forwarding_table, CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
      target_addr);

  if (nexthop != BT_MESH_ADDR_UNASSIGNED) {
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, 2);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK);
    net_buf_simple_add_le16(&msg, target_addr);

    struct bt_mesh_msg_ctx new_ctx = {
        .app_idx = model->keys[0],
        .addr = nexthop,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    LOG_DBG("Forwarding REPORT_ACK for 0x%04x to Nexthop 0x%04x", target_addr,
            nexthop);
    srv_send_msg_with_stat(srv, &new_ctx, &msg);
  } else {
    LOG_WRN("No route to forward REPORT_ACK for 0x%04x", target_addr);
  }

  return 0;
}

static int handle_report_rsp(const struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf) {
  /* Kiểm tra chiều dài payload (14 bytes: ReporterAddr(2) + 6 stats * 2) */
  if (buf->len < 14)
    return -EINVAL;

  uint16_t reporter_addr = net_buf_simple_pull_le16(buf);
  uint16_t data_tx = net_buf_simple_pull_le16(buf);
  uint16_t beacon_tx = net_buf_simple_pull_le16(buf);
  uint16_t hb_tx = net_buf_simple_pull_le16(buf);
  uint16_t route_changes = net_buf_simple_pull_le16(buf);
  uint16_t data_fwd = net_buf_simple_pull_le16(buf);
  uint16_t rx_count = net_buf_simple_pull_le16(buf);

  /* [NEW] Bóc tách RTT History */
  uint8_t rtt_samples = 0;
  if (buf->len >= 1) {
    rtt_samples = net_buf_simple_pull_u8(buf);
  }

  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  /* [ROBUST RRT] Update RRT for Report Response for ALL nodes (Sink & Relays)
   * This ensures the node forwarding this packet also learns the uplink route.
   */
  rrt_update_from_uplink_msg(srv, ctx->addr, reporter_addr, ctx->recv_rssi,
                             k_uptime_get());

  /* [LOGIC PHÂN NHÁNH: SINK vs RELAY] */
  if (srv->gradient == 0) {
    /* [1. I AM SINK] - Log dữ liệu và gửi ACK về nguồn gốc thực sự */
    LOG_INF("SINK received REPORT from 0x%04x (Forwarded by 0x%04x)",
            reporter_addr, ctx->addr);

    // Format log cho Python script (Dùng reporter_addr để định danh đúng node)
    // [NEW] Format: TYPE, Src, Tx, Beacon, HB, RouteChanges, FwdCount, RxCount
    printk("CSV_LOG,REPORT,0x%04x,%u,%u,%u,%u,%u,%u\n", reporter_addr, data_tx,
           beacon_tx, hb_tx, route_changes, data_fwd, rx_count);

    /* [NEW] Log Individual RTTs */
    for (int i = 0; i < rtt_samples; i++) {
      if (buf->len >= 4) {
        uint16_t seq = net_buf_simple_pull_le16(buf);
        uint16_t rtt = net_buf_simple_pull_le16(buf);
        printk("CSV_LOG,RTT_DATA,0x%04x,%u,%u\n", reporter_addr, seq, rtt);
      }
    }

    /* SEND ACK via Reverse Routing tới Reporter gốc */
    uint16_t nexthop = rrt_find_nexthop(
        srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE, reporter_addr);

    if (nexthop != BT_MESH_ADDR_UNASSIGNED) {
      BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, 2);
      bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_ACK);
      net_buf_simple_add_le16(&msg, reporter_addr);

      struct bt_mesh_msg_ctx ack_ctx = {
          .app_idx = model->keys[0],
          .addr = nexthop,
          .send_ttl = BT_MESH_TTL_DEFAULT,
      };
      srv_send_msg_with_stat(srv, &ack_ctx, &msg);
      LOG_DBG("Sink sent REPORT_ACK for 0x%04x via Nexthop 0x%04x",
              reporter_addr, nexthop);
    } else {
      LOG_WRN("Sink cannot find RRT route for ACK to 0x%04x (Source: 0x%04x)",
              reporter_addr, ctx->addr);
    }
  } else {
    /* [2. I AM RELAY] - Chuyển tiếp bản báo cáo lên CHA */
    const neighbor_entry_t *best_parent =
        find_strict_upstream_parent(srv, reporter_addr);

    if (best_parent != NULL) {
      LOG_INF("Relaying REPORT from 0x%04x to Parent 0x%04x", reporter_addr,
              best_parent->addr);

      /* [UPDATED] Use larger buffer to accommodate RTT data */
      BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP, 220);
      bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_RSP);

      net_buf_simple_add_le16(&msg, reporter_addr);
      net_buf_simple_add_le16(&msg, data_tx);
      net_buf_simple_add_le16(&msg, beacon_tx);
      net_buf_simple_add_le16(&msg, hb_tx);
      net_buf_simple_add_le16(&msg, route_changes);
      net_buf_simple_add_le16(&msg, data_fwd);
      net_buf_simple_add_le16(&msg, rx_count);

      /* [NEW] Forward RTT count and all RTT samples */
      net_buf_simple_add_u8(&msg, rtt_samples);
      if (rtt_samples > 0 && buf->len > 0) {
        uint8_t *data_ptr = net_buf_simple_add(&msg, buf->len);
        memcpy(data_ptr, buf->data, buf->len);
      }

      struct bt_mesh_msg_ctx fwd_ctx = {
          .app_idx = model->keys[0],
          .addr = best_parent->addr,
          .send_ttl = BT_MESH_TTL_DEFAULT,
          .send_rel = true,
      };

      srv_send_msg_with_stat(srv, &fwd_ctx, &msg);
    } else {
      LOG_ERR("Relay 0x%04x has no PARENT to forward report from 0x%04x!",
              bt_mesh_model_elem(model)->rt->addr, reporter_addr);
    }
  }

  return 0;
}

/******************************************************************************/
/* Topology Reporting Handlers                                                */
/******************************************************************************/

/* Forward declarations for topology handlers */
static int handle_topo_req(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf);
static int handle_topo_rep(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf);
static void topo_reply_work_handler(struct k_work *work);
static void topo_poll_handler(struct k_work *work);

/* Sink-side sequence counter for polling sessions (0-15, wraps around) */
static uint8_t s_topo_seq_counter;

/* [NEW] Accumulator for MAC layer absolute total packets sent 
 * (Preserved across 30s STAT resets to estimate battery) */
static uint32_t s_mac_total_sent_accum = 0;

/**
 * @brief Helper: Execute Atomic Snapshot of neighbor table.
 *        Copies ALL valid (non-expired, non-unassigned) neighbors up to
 *        TOPO_REP_MAX_NEIGHBORS. Returns false if is_reporting lock is active.
 */
static bool topo_take_snapshot(struct bt_mesh_gradient_srv *srv,
                               uint8_t seq_id) {
  /* Điểm mù 1: Chống kẹt Snapshot */
  if (srv->topo_ctx.is_reporting) {
    LOG_WRN("[TOPO] Snapshot rejected: drip-feed still active");
    return false;
  }

  srv->topo_ctx.total_valid = 0;
  srv->topo_ctx.req_seq_id = seq_id;
  int64_t now = k_uptime_get();

  k_mutex_lock(&srv->forwarding_table_mutex, K_FOREVER);

  for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
    if (srv->topo_ctx.total_valid >= TOPO_REP_MAX_NEIGHBORS) {
      break;
    }

    neighbor_entry_t *e = &srv->forwarding_table[i];

    /* Skip UNASSIGNED */
    if (e->addr == BT_MESH_ADDR_UNASSIGNED) {
      continue;
    }

    /* Skip expired (> 2 minutes = 120000 ms) */
    if ((now - e->last_seen) > CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS) {
      continue;
    }

    uint8_t idx = srv->topo_ctx.total_valid;
    srv->topo_ctx.snapshot[idx].addr = e->addr;
    srv->topo_ctx.snapshot[idx].rssi = e->rssi;
    srv->topo_ctx.snapshot[idx].grad = e->gradient;
    /* [NEW] Calculate Link Uptime in seconds */
    uint32_t uptime_s = (uint32_t)((now - e->first_seen) / 1000);
    srv->topo_ctx.snapshot[idx].link_uptime =
        (uint16_t)MIN(uptime_s, UINT16_MAX);
    srv->topo_ctx.total_valid++;
  }

  k_mutex_unlock(&srv->forwarding_table_mutex);

  /* Calculate pagination */
  if (srv->topo_ctx.total_valid == 0) {
    srv->topo_ctx.total_pages = 1; /* Will abort at send time (no uplink) */
  } else {
    srv->topo_ctx.total_pages =
        (srv->topo_ctx.total_valid + TOPO_REP_MAX_PER_PAGE - 1) /
        TOPO_REP_MAX_PER_PAGE;
  }
  srv->topo_ctx.current_page = 1;
  srv->topo_ctx.is_reporting = true; /* Lock ON */

  /* [NEW] Capture Drops and Fwd Rate from Zephyr MAC once for the entire report session */
  struct bt_mesh_statistic stats;
  bt_mesh_stat_get(&stats);
  
  uint32_t mac_planned = stats.tx_local_planned + stats.tx_adv_relay_planned;
  uint32_t mac_succeeded = stats.tx_local_succeeded + stats.tx_adv_relay_succeeded;
  
  /* MAC Drops = Planned - Succeeded (with boundary safety clamping) */
  uint32_t mac_drops = (mac_planned > mac_succeeded) ? (mac_planned - mac_succeeded) : 0;
  
  /* Universal Drops = MAC Hardware Drops + Application Soft Drops */
  uint32_t total_drops = mac_drops + srv->soft_drop_count;
  
  srv->topo_ctx.drop_count_snapshot = (uint16_t)MIN(total_drops, 65535);
  srv->topo_ctx.fwd_rate_snapshot = (uint16_t)MIN(mac_succeeded, 65535);

  /* Absolute Total Sent = Accumulator (from previous resets) + Current statistics window */
  srv->topo_ctx.total_sent_snapshot = s_mac_total_sent_accum + mac_succeeded;
  
  /* Clear local Soft Drop counter for the next window */
  srv->soft_drop_count = 0;

  LOG_INF("[TOPO] Snapshot: %d valid neighbors, %d pages, seq=%d, Drops:%u, FwdR:%u, TotalSent:%u",
          srv->topo_ctx.total_valid, srv->topo_ctx.total_pages, seq_id,
          srv->topo_ctx.drop_count_snapshot, srv->topo_ctx.fwd_rate_snapshot,
          srv->topo_ctx.total_sent_snapshot);

  return true;
}

/**
 * @brief [SENSOR] Handle OP_TOPO_REQ broadcast from Sink.
 *        Extracts seq_id from payload, takes snapshot, schedules jitter reply.
 */
static int handle_topo_req(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  /* Sink does not respond to its own request */
  if (srv->gradient == 0) {
    return 0;
  }

  /* Extract seq_id from payload (1 byte) */
  uint8_t seq_id = 0;
  if (buf->len >= 1) {
    seq_id = net_buf_simple_pull_u8(buf) & 0x0F;
  }

  LOG_INF("[TOPO] RX TOPO_REQ from 0x%04x, seq=%d", ctx->addr, seq_id);

  /* 1. Atomic Snapshot (will fail if drip-feed is active) */
  if (!topo_take_snapshot(srv, seq_id)) {
    return 0; /* Busy, ignore this request */
  }

  /* 2. Schedule reply with Jitter (0 - 8000 ms) */
  uint32_t delay_ms = sys_rand32_get() % 8000;
  k_work_reschedule(&srv->topo_ctx.reply_work, K_MSEC(delay_ms));

  return 0;
}

/**
 * @brief [SENSOR] Workqueue handler: Multi-Page Drip-feed TX.
 *        Sends one page per invocation, then reschedules itself for the next
 * page.
 */
static void topo_reply_work_handler(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct sensor_topo_ctx *tctx =
      CONTAINER_OF(dwork, struct sensor_topo_ctx, reply_work);
  struct bt_mesh_gradient_srv *srv =
      CONTAINER_OF(tctx, struct bt_mesh_gradient_srv, topo_ctx);

  /* Safety: Must have a model and be provisioned */
  if (!srv->model || !bt_mesh_is_provisioned()) {
    tctx->is_reporting = false;
    return;
  }

  uint16_t my_addr = bt_mesh_model_elem(srv->model)->rt->addr;

  /* Find Uplink nexthop (best parent = index 0) */
  k_mutex_lock(&srv->forwarding_table_mutex, K_FOREVER);
  uint16_t nexthop = srv->forwarding_table[0].addr;
  uint16_t parent_addr = nexthop;
  k_mutex_unlock(&srv->forwarding_table_mutex);

  if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
    LOG_WRN("[TOPO] No uplink route, aborting drip-feed");
    tctx->is_reporting = false; /* Unlock */
    return;
  }

  /* Điểm mù 2: Safe array indexing for current page */
  uint8_t start_idx = (tctx->current_page - 1) * TOPO_REP_MAX_PER_PAGE;
  uint8_t neighbors_in_page =
      MIN(TOPO_REP_MAX_PER_PAGE, tctx->total_valid - start_idx);

  /* Stack-allocated buffer (Code Safety: no malloc/free) */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TOPO_REP,
                           TOPO_REP_MAX_PAYLOAD);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TOPO_REP);

  /* === Pack 8-byte Header === */
  net_buf_simple_add_le16(&msg, my_addr);     /* Origin_Addr (2B) */
  net_buf_simple_add_u8(&msg, srv->gradient); /* My_Grad (1B) */
  net_buf_simple_add_le16(&msg, parent_addr); /* My_Parent (2B) */
  /* Seq_ID (4 bits cao) | Total_Pages (4 bits thấp) */
  net_buf_simple_add_u8(&msg,
                        (tctx->req_seq_id << 4) | (tctx->total_pages & 0x0F));
  net_buf_simple_add_u8(&msg, tctx->current_page); /* Current_Page (1B) */
  net_buf_simple_add_u8(&msg, neighbors_in_page);  /* Neighbors_in_Page (1B) */

  /* [UPD] Drop Count (2B) and Fwd Rate (2B): Use values captured at snapshot time */
  net_buf_simple_add_le16(&msg, tctx->drop_count_snapshot);
  net_buf_simple_add_le16(&msg, tctx->fwd_rate_snapshot);

  /* (MAC stats reset only after the full report cycle to ensure continuity) */
  if (tctx->current_page >= tctx->total_pages) {
    struct bt_mesh_statistic stats;
    bt_mesh_stat_get(&stats);
    uint32_t current_window_succeeded = stats.tx_local_succeeded + stats.tx_adv_relay_succeeded;
    /* Accumulator = snapshot value + delta sent DURING drip-feed phase.
     * fwd_rate_snapshot was the mac_succeeded at snapshot time.  
     * current_window_succeeded includes those PLUS packets sent during drip-feed.
     * Delta = current_window_succeeded - fwd_rate_snapshot (drip-feed packets only). */
    uint32_t drip_delta = (current_window_succeeded > tctx->fwd_rate_snapshot) 
                        ? (current_window_succeeded - tctx->fwd_rate_snapshot) : 0;
    s_mac_total_sent_accum = tctx->total_sent_snapshot + drip_delta;
    bt_mesh_stat_reset();
    LOG_DBG("[TOPO] Report done. Accum=%u (snap=%u + drip=%u)", 
            s_mac_total_sent_accum, tctx->total_sent_snapshot, drip_delta);
  }

  /* [NEW] Add Node Uptime (4B) in seconds */
  uint32_t node_uptime_s = (uint32_t)(k_uptime_get() / 1000);
  net_buf_simple_add_le32(&msg, node_uptime_s);

  /* [NEW] Add MAC-layer Total Packets Sent (4B) for precise battery estimation */
  net_buf_simple_add_le32(&msg, tctx->total_sent_snapshot);

  /* === Pack neighbor data (max 24B) === */
  for (uint8_t i = 0; i < neighbors_in_page; i++) {
    uint8_t idx = start_idx + i;
    net_buf_simple_add_le16(&msg, tctx->snapshot[idx].addr);
    net_buf_simple_add_u8(&msg, (uint8_t)tctx->snapshot[idx].rssi);
    net_buf_simple_add_u8(&msg, tctx->snapshot[idx].grad);
    net_buf_simple_add_le16(&msg, tctx->snapshot[idx].link_uptime);
  }

  /* Send Uplink via Gradient Routing (hop-by-hop) */
  struct bt_mesh_msg_ctx ctx = {
      .app_idx = srv->model->keys[0],
      .addr = nexthop,
      .send_ttl = BT_MESH_TTL_DEFAULT,
      .send_rel = true, /* Segmented + BlockAck */
  };

  int err = srv_send_msg_with_stat(srv, &ctx, &msg);
  if (err) {
    LOG_ERR("[TOPO] Failed to send page %d/%d, err=%d", tctx->current_page,
            tctx->total_pages, err);
  } else {
    LOG_INF("[TOPO] Sent page %d/%d (%d neighbors) to 0x%04x",
            tctx->current_page, tctx->total_pages, neighbors_in_page, nexthop);
  }

  /* === Drip-feed state machine === */
  if (tctx->current_page < tctx->total_pages) {
    tctx->current_page++;
    /* Adaptive Jitter: 300ms + 0-100ms random */
    uint32_t delay = 300 + (sys_rand32_get() % 100);
    k_work_reschedule(&tctx->reply_work, K_MSEC(delay));
  } else {
    /* All pages sent! Unlock for next cycle */
    tctx->is_reporting = false;
    LOG_INF("[TOPO] Drip-feed complete (seq=%d)", tctx->req_seq_id);
  }
}

/**
 * @brief [SINK + RELAY] Handle OP_TOPO_REP (Multi-Page aware).
 *   - Sink (gradient==0): Parse 8-byte header, print UART frame with seq_id.
 *   - Relay (gradient>0): Forward payload intact to own best parent.
 */
static int handle_topo_rep(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  /* --- Parse 20-byte header --- */
  if (buf->len < 20) {
    return -EINVAL;
  }
  uint16_t origin_addr = net_buf_simple_pull_le16(buf);
  uint8_t grad = net_buf_simple_pull_u8(buf);
  uint16_t parent = net_buf_simple_pull_le16(buf);
  uint8_t total_and_seq = net_buf_simple_pull_u8(buf);
  uint8_t current_page = net_buf_simple_pull_u8(buf);
  uint8_t count = net_buf_simple_pull_u8(buf);
  uint16_t drop_count = net_buf_simple_pull_le16(buf);
  uint16_t fwd_rate = net_buf_simple_pull_le16(buf);
  uint32_t node_uptime = net_buf_simple_pull_le32(buf);
  uint32_t total_sent = net_buf_simple_pull_le32(buf);

  /* Unpack seq_id and total_pages from combined byte */
  uint8_t seq_id = (total_and_seq >> 4) & 0x0F;
  uint8_t total_pages = total_and_seq & 0x0F;

  if (count > TOPO_REP_MAX_PER_PAGE) {
    count = TOPO_REP_MAX_PER_PAGE; /* Safety clamp */
  }

  /* [ROBUST RRT] Update RRT for Topology Reports for ALL nodes (Sink & Relays)
   * This ensures intermediate nodes learn the path back to the origin.
   */
  rrt_update_from_uplink_msg(srv, ctx->addr, origin_addr, ctx->recv_rssi,
                             k_uptime_get());

  if (srv->gradient == 0) {
    /* ═══════════════════════════════════════════════════════
     * I AM SINK: Parse and print to UART with SOF/EOF
     * Format:
     * $[TOPO],<Origin>,<Seq>,<TotalPg>,<CurPg>,<Count>,<My_Grad>,<My_Parent>,<Drops>,<FwdRate>,<Uptime>s,<TotalSent>,[neighbors]
     * ═══════════════════════════════════════════════════════ */
    char uart_buf[300];
    int pos = snprintf(uart_buf, sizeof(uart_buf),
                       "$[TOPO],%04X,%d,%d,%d,%d,%d,%04X,%u,%u,%u,%u", origin_addr,
                       seq_id, total_pages, current_page, count, grad, parent,
                       drop_count, fwd_rate, node_uptime, total_sent);

    for (int i = 0; i < count; i++) {
      /* Bounds checking before each pull */
      if (buf->len < 4) {
        break;
      }
      uint16_t n_addr = net_buf_simple_pull_le16(buf);
      int8_t n_rssi = (int8_t)net_buf_simple_pull_u8(buf);
      uint8_t n_grad = net_buf_simple_pull_u8(buf);
      uint16_t n_uptime = net_buf_simple_pull_le16(buf);

      if (pos < sizeof(uart_buf)) {
        pos += snprintf(uart_buf + pos, sizeof(uart_buf) - pos,
                        ",[%04X,%d,%d,%u]", n_addr, n_rssi, n_grad, n_uptime);
      }
    }
    printk("%s\n",
           uart_buf); /* Atomic print to avoid shell prompt interleaving */

    LOG_INF("[TOPO] RX page %d/%d from 0x%04X (seq=%d, grad=%d, Drp=%u, FwdR=%u, "
            "UP:%u, TX:%u, parent=0x%04X via 0x%04x)",
            current_page, total_pages, origin_addr, seq_id, grad, drop_count, fwd_rate,
            node_uptime, total_sent, parent, ctx->addr);
  } else {
    /* ═══════════════════════════════════════════════════════
     * I AM RELAY: Forward entire payload to my best parent
     * ═══════════════════════════════════════════════════════ */
    k_mutex_lock(&srv->forwarding_table_mutex, K_FOREVER);
    uint16_t nexthop = srv->forwarding_table[0].addr;
    k_mutex_unlock(&srv->forwarding_table_mutex);

    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
      LOG_ERR("[TOPO] Relay: No parent to forward from 0x%04X", origin_addr);
      return 0;
    }

    /* Re-pack the full payload for forwarding (header + data) */
    BT_MESH_MODEL_BUF_DEFINE(fwd, BT_MESH_GRADIENT_SRV_OP_TOPO_REP,
                             TOPO_REP_MAX_PAYLOAD);
    bt_mesh_model_msg_init(&fwd, BT_MESH_GRADIENT_SRV_OP_TOPO_REP);

    net_buf_simple_add_le16(&fwd, origin_addr);
    net_buf_simple_add_u8(&fwd, grad);
    net_buf_simple_add_le16(&fwd, parent);
    net_buf_simple_add_u8(&fwd, total_and_seq); /* Preserve seq+pages */
    net_buf_simple_add_u8(&fwd, current_page);
    net_buf_simple_add_u8(&fwd, count);
    net_buf_simple_add_le16(&fwd, drop_count);
    net_buf_simple_add_le16(&fwd, fwd_rate);
    net_buf_simple_add_le32(&fwd, node_uptime);
    net_buf_simple_add_le32(&fwd, total_sent);

    for (int i = 0; i < count; i++) {
      if (buf->len < 6) {
        break;
      }
      uint8_t *chunk = net_buf_simple_pull_mem(buf, 6);
      net_buf_simple_add_mem(&fwd, chunk, 6);
    }

    struct bt_mesh_msg_ctx fwd_ctx = {
        .app_idx = model->keys[0],
        .addr = nexthop,
        .send_ttl = BT_MESH_TTL_DEFAULT,
        .send_rel = true,
    };

    int err = srv_send_msg_with_stat(srv, &fwd_ctx, &fwd);
    if (err) {
      LOG_ERR("[TOPO] Relay forward failed, err=%d", err);
    } else {
      LOG_INF("[TOPO] Relayed page %d/%d from 0x%04X to 0x%04X", current_page,
              total_pages, origin_addr, nexthop);
    }
  }

  return 0;
}

/**
 * @brief [SINK] Periodic polling timer handler.
 *        Broadcasts OP_TOPO_REQ every CONFIG_BT_MESH_TOPO_POLL_INTERVAL
 * seconds.
 */
static void topo_poll_handler(struct k_work *work) {
  struct k_work_delayable *dwork = k_work_delayable_from_work(work);
  struct bt_mesh_gradient_srv *srv =
      CONTAINER_OF(dwork, struct bt_mesh_gradient_srv, topo_poll_work);

  if (!bt_mesh_is_provisioned() || srv->gradient != 0) {
    return;
  }

  bt_mesh_gradient_srv_send_topo_req(srv);

  /* Reschedule */
  k_work_reschedule(&srv->topo_poll_work,
                    K_SECONDS(CONFIG_BT_MESH_TOPO_POLL_INTERVAL));
}

/**
 * @brief [PUBLIC] Broadcast OP_TOPO_REQ with Sequence ID.
 */
int bt_mesh_gradient_srv_send_topo_req(struct bt_mesh_gradient_srv *srv) {
  s_topo_seq_counter = (s_topo_seq_counter + 1) & 0x0F; /* Wrap 0-15 */

  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TOPO_REQ, 1);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TOPO_REQ);
  net_buf_simple_add_u8(&msg, s_topo_seq_counter);

  struct bt_mesh_msg_ctx ctx = {
      .app_idx = srv->model->keys[0],
      .addr = BT_MESH_ADDR_ALL_NODES,
      .send_ttl = BT_MESH_TTL_DEFAULT,
      .send_rel = false,
  };

  LOG_INF("[TOPO] Broadcasting TOPO_REQ (seq=%d)", s_topo_seq_counter);
  return srv_send_msg_with_stat(srv, &ctx, &msg);
}

/**
 * @brief [PUBLIC] Trigger an immediate Delta Topology Report.
 *        Called when best_parent changes or an important neighbor drops.
 *        Uses seq_id = 15 (0x0F) to distinguish from polled reports.
 */
void bt_mesh_gradient_srv_trigger_delta_topo(struct bt_mesh_gradient_srv *srv) {
  if (srv->gradient == 0) {
    return; /* Sink does not report */
  }

  /* Respect the is_reporting lock */
  if (!topo_take_snapshot(srv, 0x0F)) { /* Delta uses seq_id = 15 */
    return;
  }

  LOG_INF("[TOPO] Delta trigger! %d neighbors, scheduling report",
          srv->topo_ctx.total_valid);

  /* Short jitter for event-driven reports (100-500 ms) */
  uint32_t delay_ms = 100 + (sys_rand32_get() % 400);
  k_work_reschedule(&srv->topo_ctx.reply_work, K_MSEC(delay_ms));
}

/**
 * @brief [PUBLIC] Initialize Topology Reporting subsystem.
 *        Called once during model_handler_init().
 */
void topo_routing_init(struct bt_mesh_gradient_srv *srv) {
  /* Init sensor-side reply work */
  k_work_init_delayable(&srv->topo_ctx.reply_work, topo_reply_work_handler);
  srv->topo_ctx.total_valid = 0;
  srv->topo_ctx.total_pages = 0;
  srv->topo_ctx.current_page = 0;
  srv->topo_ctx.is_reporting = false;
  srv->topo_ctx.req_seq_id = 0;
  srv->topo_ctx.last_reported_parent = BT_MESH_ADDR_UNASSIGNED;

  /* Init sink-side polling work */
  k_work_init_delayable(&srv->topo_poll_work, topo_poll_handler);
  s_topo_seq_counter = 0;

  /* If Sink, start periodic polling after a short delay */
  if (srv->gradient == 0) {
    k_work_schedule(&srv->topo_poll_work,
                    K_SECONDS(CONFIG_BT_MESH_TOPO_POLL_INTERVAL));
    LOG_INF("[TOPO] Sink polling started (interval: %d s)",
            CONFIG_BT_MESH_TOPO_POLL_INTERVAL);
  }
}

/******************************************************************************/
/* Model Operations                                                           */
/******************************************************************************/

const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[] = {
    {BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS,
     BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
     handle_gradient_message},
    {BT_MESH_GRADIENT_SRV_OP_PONG,
     BT_MESH_LEN_EXACT(4), /* target_addr(2) + seq(2) */
     handle_pong_message},
    {BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE,
     BT_MESH_LEN_MIN(BT_MESH_GRADIENT_SRV_MSG_MINLEN_MESSAGE),
     handle_data_message},
    {BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, BT_MESH_LEN_EXACT(11),
     handle_backprop_message},
    /* [UPDATED] Report Req expects 1 byte ID now */
    {BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, BT_MESH_LEN_MIN(1), handle_report_req},
    {BT_MESH_GRADIENT_SRV_OP_REPORT_RSP,
     BT_MESH_LEN_MIN(15), /* reporter(2) + 6*stats(12) + rtt_count(1) */
     handle_report_rsp},
    {BT_MESH_GRADIENT_SRV_OP_REPORT_REQ_UNICAST,
     BT_MESH_LEN_EXACT(0), /* No payload */
     handle_report_req_unicast},
    {BT_MESH_GRADIENT_SRV_OP_REPORT_ACK, BT_MESH_LEN_EXACT(2),
     handle_report_ack},
    {BT_MESH_GRADIENT_SRV_OP_DOWNLINK_REPORT, BT_MESH_LEN_EXACT(2),
     handle_downlink_report},
    {BT_MESH_GRADIENT_SRV_OP_TEST_START,
     BT_MESH_LEN_MIN(1), /* Expect 1 byte payload (Test ID) */
     handle_test_start},
    /* [NEW] Topology Reporting Opcodes (Multi-Page v5) */
    {BT_MESH_GRADIENT_SRV_OP_TOPO_REQ, BT_MESH_LEN_MIN(1), handle_topo_req},
    {BT_MESH_GRADIENT_SRV_OP_TOPO_REP,
     BT_MESH_LEN_MIN(20), /* 20B header (no neighbors) */
     handle_topo_rep},
    BT_MESH_MODEL_OP_END,
};

/******************************************************************************/
/* Model Callbacks & Init                                                     */
/******************************************************************************/

/******************************************************************************/
/* PONG & Reporting APIs                                                      */
/******************************************************************************/

static int handle_pong_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf) {
  struct bt_mesh_gradient_srv *srv = model->rt->user_data;

  if (buf->len < 4)
    return -EINVAL;

  uint16_t target_addr = net_buf_simple_pull_le16(buf);
  uint16_t seq = net_buf_simple_pull_le16(buf);
  uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

  if (target_addr == my_addr) {
    LOG_INF("Received PONG for seq %u from 0x%04x (Origin: 0x%04x)", seq,
            ctx->addr, target_addr);
    /* Record RTT and check if buffer is full */
    if (pkt_stats_record_pong(seq)) {
      LOG_INF("RTT Buffer FULL (50 samples). Triggering Report.");
      if (!srv->is_report_pending) {
        srv->report_retry_count = 0;
        k_work_reschedule(&srv->report_retry_work, K_NO_WAIT);
        srv->is_report_pending = true;
      }
    }
  } else {
    /* [FORWARDING] Chuyển tiếp PONG về node nguồn thông qua RRT */
    uint16_t nexthop = rrt_find_nexthop(
        srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE, target_addr);
    if (nexthop != BT_MESH_ADDR_UNASSIGNED) {
      LOG_INF("Relaying PONG for 0x%04x to Nexthop 0x%04x", target_addr,
              nexthop);

      BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_PONG, 4);
      bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_PONG);
      net_buf_simple_add_le16(&msg, target_addr);
      net_buf_simple_add_le16(&msg, seq);

      struct bt_mesh_msg_ctx fwd_ctx = {
          .app_idx = model->keys[0],
          .addr = nexthop,
          .send_ttl = BT_MESH_TTL_DEFAULT,
      };
      srv_send_msg_with_stat(srv, &fwd_ctx, &msg);
    } else {
      LOG_WRN("PONG Forward Failed: No route to 0x%04x", target_addr);
    }
  }

  return 0;
}

int bt_mesh_gradient_srv_send_pong(struct bt_mesh_gradient_srv *srv,
                                   uint16_t dest_addr, uint16_t seq) {
  /* Find nexthop back to the source using RRT */
  uint16_t nexthop = rrt_find_nexthop(
      srv->forwarding_table, CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
      dest_addr);

  if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
    LOG_WRN("No route to send PONG to 0x%04x", dest_addr);
    return -ENETUNREACH;
  }

  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_PONG, 4);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_PONG);
  net_buf_simple_add_le16(&msg,
                          dest_addr); /* include destination for multi-hop */
  net_buf_simple_add_le16(&msg, seq);

  struct bt_mesh_msg_ctx ctx = {
      .app_idx = srv->model->keys[0],
      .addr = nexthop,
      .send_ttl = BT_MESH_TTL_DEFAULT,
  };

  LOG_DBG("Sending PONG (Seq %u) for 0x%04x via Nexthop 0x%04x", seq, dest_addr,
          nexthop);
  return srv_send_msg_with_stat(srv, &ctx, &msg);
}

static int
bt_mesh_gradient_srv_update_handler(const struct bt_mesh_model *model) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
  struct net_buf_simple *buf = model->pub->msg;

  if (gradient_srv->gradient == UINT8_MAX) {
    return 0; /* Don't publish uninitialized gradient */
  }

  bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
  net_buf_simple_add_u8(buf, gradient_srv->gradient);

  LOG_INF("[CONTROL - Gradient Beacon] Auto-publishing gradient: %d",
          gradient_srv->gradient);
  return 0;
}

#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_gradient_srv_settings_set(const struct bt_mesh_model *model,
                                             const char *name, size_t len_rd,
                                             settings_read_cb read_cb,
                                             void *cb_arg) {
  return 0;
}
#endif

static int bt_mesh_gradient_srv_init(const struct bt_mesh_model *model) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;
  gradient_srv->model = model;

  /* Init Reliable Report Context */
  k_work_init_delayable(&gradient_srv->report_retry_work, report_retry_handler);
  gradient_srv->is_report_pending = false;
  gradient_srv->report_retry_count = 0;
  gradient_srv->soft_drop_count = 0;

  net_buf_simple_init_with_data(&gradient_srv->pub_msg, gradient_srv->buf,
                                sizeof(gradient_srv->buf));
  gradient_srv->pub.msg = &gradient_srv->pub_msg;
  gradient_srv->pub.update = bt_mesh_gradient_srv_update_handler;

  k_mutex_init(&gradient_srv->forwarding_table_mutex);

  led_indication_init();
  data_forward_init();
  gradient_work_init();
  pkt_stats_init();

  return 0;
}

static int bt_mesh_gradient_srv_start(const struct bt_mesh_model *model) {
  struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

  if (gradient_srv->handlers->start) {
    gradient_srv->handlers->start(gradient_srv);
  }

  gradient_work_set_srv(gradient_srv);

  if (model->pub) {
    model->pub->addr = BT_MESH_ADDR_ALL_NODES;
    model->pub->ttl = 0;
    model->pub->period = BT_MESH_PUB_PERIOD_SEC(5);
    gradient_work_schedule_initial_publish();
  }

  gradient_work_start_cleanup();
  return 0;
}

static void bt_mesh_gradient_srv_reset(const struct bt_mesh_model *model) {
  if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
    (void)bt_mesh_model_data_store(model, true, NULL, NULL, 0);
  }
}

const struct bt_mesh_model_cb _bt_mesh_gradient_srv_cb = {
    .init = bt_mesh_gradient_srv_init,
    .start = bt_mesh_gradient_srv_start,
#ifdef CONFIG_BT_SETTINGS
    .settings_set = bt_mesh_gradient_srv_settings_set,
#endif
    .reset = bt_mesh_gradient_srv_reset,
};

/******************************************************************************/
/* Public APIs                                                                */
/******************************************************************************/

int bt_mesh_gradient_srv_gradient_send(
    struct bt_mesh_gradient_srv *gradient_srv) {
  struct net_buf_simple *buf = gradient_srv->model->pub->msg;
  bt_mesh_model_msg_init(buf, BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS);
  net_buf_simple_add_u8(buf, gradient_srv->gradient);

  pkt_stats_inc_gradient_beacon();
  return bt_mesh_model_publish(gradient_srv->model);
}

int bt_mesh_gradient_srv_data_send(struct bt_mesh_gradient_srv *gradient_srv,
                                   uint16_t addr, uint16_t data,
                                   int8_t initial_rssi) {
  /* [NEW] Record sent time for Ping-Pong RTT */
  pkt_stats_record_sent(data);

  return data_forward_send_direct(gradient_srv, addr, data, initial_rssi);
}

int bt_mesh_gradient_srv_backprop_send(
    struct bt_mesh_gradient_srv *gradient_srv, uint16_t dest_addr,
    uint16_t payload) {
  uint16_t my_addr = bt_mesh_model_elem(gradient_srv->model)->rt->addr;

  if (dest_addr == my_addr)
    return -EINVAL;

  uint16_t nexthop = rrt_find_nexthop(
      gradient_srv->forwarding_table,
      CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE, dest_addr);

  if (nexthop == BT_MESH_ADDR_UNASSIGNED)
    return -ENETUNREACH;

  /* Đóng gói cấu trúc 11 bytes: Dest(2) | Payload(2) | TTL(1) | Hops(1) | TS(4)
   * | MinRSSI(1) */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 11);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);

  net_buf_simple_add_le16(&msg, dest_addr);
  net_buf_simple_add_le16(&msg, payload);
  net_buf_simple_add_u8(&msg, BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL);
  net_buf_simple_add_u8(&msg, 1); // Initial hop count

  /* [RELATIVE TIMESTAMP] */
  uint32_t timestamp =
      k_uptime_get_32() -
      (g_test_start_time > 0 ? g_test_start_time : k_uptime_get_32());
  net_buf_simple_add_le32(&msg, timestamp);

  /* [INITIAL RSSI] Sink bắt đầu với 0 (lớn nhất) */
  net_buf_simple_add_u8(&msg, 0);

  struct bt_mesh_msg_ctx ctx = {
      .app_idx = gradient_srv->model->keys[0],
      .addr = nexthop,
      .send_ttl = 0,
      .send_rel = true, /* [FIX] Reliable delivery for Backprop */
  };

  return srv_send_msg_with_stat(gradient_srv, &ctx, &msg);
}

/**
 * @brief [UPDATED] Gửi lệnh yêu cầu báo cáo xuống toàn mạng (Controlled
 * Flooding)
 */
int bt_mesh_gradient_srv_send_report_req(
    struct bt_mesh_gradient_srv *gradient_srv, bool force_new_id) {
  /* 1. Tăng ID cho lần gửi mới nếu được yêu cầu */
  if (force_new_id) {
    current_tx_req_id++;
  }

  /* 2. Đóng gói ID (1 byte) vào payload */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ, 1);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_REPORT_REQ);
  net_buf_simple_add_u8(&msg, current_tx_req_id);

  /* 3. Gửi Broadcast với TTL mặc định */
  struct bt_mesh_msg_ctx ctx = {
      .app_idx = gradient_srv->model->keys[0],
      .addr = BT_MESH_ADDR_ALL_NODES,
      .send_ttl = BT_MESH_TTL_DEFAULT,
  };

  LOG_INF(">>> BROADCASTING REPORT REQUEST (ID: %d) <<<", current_tx_req_id);
  return srv_send_msg_with_stat(gradient_srv, &ctx, &msg);
}

/**
 * @brief [UPDATED] Gửi lệnh TEST START xuống toàn mạng (Controlled Flooding)
 */
int bt_mesh_gradient_srv_send_test_start(
    struct bt_mesh_gradient_srv *gradient_srv, bool force_new_id) {
  /* 1. Tăng ID cho lần gửi mới nếu được yêu cầu */
  if (force_new_id) {
    current_tx_test_id++;
  }

  /* 2. Tạo gói tin có payload là Test ID (1 byte) */
  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_TEST_START, 1);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_TEST_START);
  net_buf_simple_add_u8(&msg, current_tx_test_id);

  /* 3. Gửi Broadcast với TTL mặc định */
  struct bt_mesh_msg_ctx ctx = {
      .app_idx = gradient_srv->model->keys[0],
      .addr = BT_MESH_ADDR_ALL_NODES,
      .send_ttl = BT_MESH_TTL_DEFAULT,
  };

  LOG_INF(">>> BROADCASTING TEST START (ID: %d) <<<", current_tx_test_id);
  return srv_send_msg_with_stat(gradient_srv, &ctx, &msg);
}

/**
 * @brief [NEW] Gửi phản hồi báo cáo về Sink (Unicast)
 */
int bt_mesh_gradient_srv_report_rsp_send(
    struct bt_mesh_gradient_srv *gradient_srv) {
  LOG_INF("Starting Reliable Report Sequence...");

  gradient_srv->is_report_pending = true;
  gradient_srv->report_retry_count = 0;

  /* [FIX CONGESTION] Thêm Delay ngẫu nhiên ban đầu (0-5s) cho cấu hình 40 node
   * để tránh việc hàng chục node cùng ập vào Sink một lúc gây nghẽn.
   */
  uint32_t initial_jitter = sys_rand32_get() % 5000;
  k_work_schedule(&gradient_srv->report_retry_work, K_MSEC(initial_jitter));

  return 0;
}

int bt_mesh_gradient_srv_send_downlink_report(
    struct bt_mesh_gradient_srv *gradient_srv, uint16_t dest_addr,
    uint16_t total_tx) {
  uint16_t nexthop = rrt_find_nexthop(
      gradient_srv->forwarding_table,
      CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE, dest_addr);

  if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
    LOG_ERR("No route to send Downlink Report to 0x%04x", dest_addr);
    return -ENETUNREACH;
  }

  BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_DOWNLINK_REPORT, 2);
  bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_DOWNLINK_REPORT);
  net_buf_simple_add_le16(&msg, total_tx);

  struct bt_mesh_msg_ctx ctx = {
      .app_idx = gradient_srv->model->keys[0],
      .addr = nexthop,
      .send_ttl = BT_MESH_TTL_DEFAULT,
  };

  LOG_INF("Sending Downlink Report to 0x%04x (via 0x%04x), TX: %u", dest_addr,
          nexthop, total_tx);
  return srv_send_msg_with_stat(gradient_srv, &ctx, &msg);
}