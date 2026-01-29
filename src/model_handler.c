/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h> 

#include "gradient_srv.h"
#include "model_handler.h"
#include "led_indication.h"
#include "reverse_routing.h"
#include "heartbeat.h"
#include "packet_stats.h" // [FIX] Thêm header này để dùng pkt_stats

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

/* ==========================================
 * CẤU HÌNH TEST (TEST CONFIGURATION)
 * ========================================== */
// 1. Cấu hình thời gian chạy test (1 phút)
#define TEST_DURATION_MINUTES   2
#define TEST_DURATION_MS        (TEST_DURATION_MINUTES * 60 * 1000) 

// 2. Cấu hình cho Sensor Node (Tần suất gửi tin)
#define SENSOR_SEND_INTERVAL_MS 1000  // Tăng lên 2 giây để giảm nghẽn mạng

/* ==========================================
 * GLOBALS & WORK ITEMS
 * ========================================== */
static const struct shell *chat_shell;
uint16_t g_total_tx_count = 0; // Biến đếm SeqNum toàn cục (cho cả HB và DATA)
uint16_t g_test_data_tx_count = 0; // [NEW] Chỉ đếm gói DATA trong phiên test
uint32_t g_test_start_time = 0; 

// [NEW] Biến chống rung phím (Debounce)
static int64_t last_action_time = 0;
#define ACTION_COOLDOWN_MS 2000 // Chờ 2 giây giữa các lần thao tác

// --- SINK NODE VARIABLES ---
static struct k_work_delayable auto_stop_work;
static bool is_test_running = false;

// --- SENSOR NODE VARIABLES ---
static struct k_work_delayable send_data_work;
static bool is_sending_active = false; 

/* Forward declarations */
static void button_handler(uint32_t button_state, uint32_t has_changed);
static void auto_stop_handler(struct k_work *work); 
static void send_data_handler(struct k_work *work); 
static void print_neighbor_table(void); 

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/
static void attention_on(const struct bt_mesh_model *mod)
{
    LOG_INF("Attention Mode ON");
    led_indicate_attention(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
    LOG_INF("Attention Mode OFF");
    led_indicate_attention(false);
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/******************************************************************************/
/***************************** Gradient model setup ***************************/
/******************************************************************************/

static void print_client_status(void);

static void handle_chat_start(struct bt_mesh_gradient_srv *Gradient)
{
    print_client_status();
    heartbeat_start(Gradient);
}

static void handle_data_received(struct bt_mesh_gradient_srv *srv, uint16_t data)
{
    LOG_INF("[App] BACKPROP/Data received: %d", data);
}

/* [SENSOR] Xử lý khi nhận lệnh TEST START từ Sink */
static void handle_test_start_received(struct bt_mesh_gradient_srv *srv)
{
    LOG_INF(">> EVENT: START COMMAND RECEIVED FROM SINK <<");
    
    /* 0. Bật thống kê gói tin */
    pkt_stats_set_enabled(true);
    
    /* 1. Reset biến đếm và lưu thời điểm bắt đầu */
    g_total_tx_count = 0; 
    g_test_data_tx_count = 0; // Reset đếm gói test
    g_test_start_time = k_uptime_get_32();
    pkt_stats_reset();
    
    /* 2. Đảm bảo trạng thái gửi là False trước để hủy mọi timer cũ */
    is_sending_active = false;
    k_work_cancel_delayable(&send_data_work);

    /* 3. Bật lại trạng thái và khởi động Timer mới */
    is_sending_active = true;
    
    /* 4. Jitter ngẫu nhiên lớn để tránh các node gửi cùng lúc */
    uint32_t start_delay = sys_rand32_get() % 1000; 
    k_work_schedule(&send_data_work, K_MSEC(start_delay));
    
    /* 5. Nháy đèn liên tục để báo hiệu đang TRONG CHẾ ĐỘ TEST */
    led_indicate_attention(true);

    LOG_INF(">> Auto-sending STARTED (Delay: %d ms)", start_delay);
}

/* [SENSOR] Xử lý khi nhận REPORT REQ (STOP) từ Sink */
static void handle_report_req_received(struct bt_mesh_gradient_srv *srv)
{
    LOG_WRN(">> EVENT: STOP REQUEST RECEIVED FROM SINK <<");
    
    /* 0. Ngừng thống kê gói tin */
    pkt_stats_set_enabled(false);
    
    /* 1. NGỪNG GỬI NGAY LẬP TỨC VÀ TẮT ĐÈN */
    is_sending_active = false; 
    k_work_cancel_delayable(&send_data_work);
    led_indicate_attention(false);
    
    /* [FIX PDR > 100%] Lấy số gói DATA thực tế đã gửi để báo cáo */
    struct packet_stats stats;
    pkt_stats_get(&stats);
    
    LOG_INF(">>> TEST STOPPED. Reported DATA Tx: %u <<<", g_test_data_tx_count);
    
    /* 2. LOGIC JITTER: Đợi một khoảng thời gian ngẫu nhiên trước khi báo cáo 
     * để tránh "nghẽn cổ chai" tại các nexthop gần Gateway.
     */
    uint32_t random_delay = 500 + (sys_rand32_get() % 3500);
    LOG_INF(">> Report will be sent in %d ms (Reliable Sequence)...", random_delay);
    
    /* 3. Bắt đầu tiến trình gửi báo cáo tin cậy (có ACK/Retry) */
    srv->is_report_pending = true;
    srv->report_retry_count = 0;
    k_work_schedule(&srv->report_retry_work, K_MSEC(random_delay));
}

static const struct bt_mesh_gradient_srv_handlers chat_handlers = {
    .start = handle_chat_start,
    .data_received = handle_data_received,
    .report_req_received = handle_report_req_received,
    .test_start_received = handle_test_start_received,
};

struct bt_mesh_gradient_srv gradient_srv = {
    .handlers = &chat_handlers,
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(
        1,
        BT_MESH_MODEL_LIST(
            BT_MESH_MODEL_CFG_SRV,
            BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub)),
        BT_MESH_MODEL_LIST(BT_MESH_MODEL_GRADIENT_SRV(&gradient_srv))),
};

static void print_client_status(void)
{
    if (!bt_mesh_is_provisioned()) {
        shell_print(chat_shell, "Mesh node not provisioned.");
    } else {
        shell_print(chat_shell, "Mesh node provisioned. Addr: 0x%04x",
                bt_mesh_model_elem(gradient_srv.model)->rt->addr);
    }
}

static const struct bt_mesh_comp comp = {
    .cid = CONFIG_BT_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/******************************************************************************/
/******************************** Public API **********************************/
/******************************************************************************/
const struct bt_mesh_comp *model_handler_init(void)
{
    chat_shell = shell_backend_uart_get_ptr();

    // Init Forwarding Table
    for(int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++){
        gradient_srv.forwarding_table[i].addr = BT_MESH_ADDR_UNASSIGNED;
        gradient_srv.forwarding_table[i].rssi = INT8_MIN;
        gradient_srv.forwarding_table[i].gradient = UINT8_MAX;
        gradient_srv.forwarding_table[i].last_seen = 0;
        gradient_srv.forwarding_table[i].backprop_dest = NULL;
    }

    rrt_init(gradient_srv.forwarding_table, 
             CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    heartbeat_init();
    
    // [INIT] Timers
    k_work_init_delayable(&auto_stop_work, auto_stop_handler); // Cho Sink
    k_work_init_delayable(&send_data_work, send_data_handler);     // Cho Sensor

#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
    gradient_srv.gradient = 0;
    LOG_INF("Initialized as SINK node (gradient = 0)\n");
#else
    gradient_srv.gradient = UINT8_MAX;
    LOG_INF("Initialized as regular node (gradient = 255)\n");
#endif

    dk_buttons_init(button_handler);
    return &comp;
}

/* -------------------------------------------------------------------------
 * HELPER FUNCTIONS (SINK CONTROL)
 * ------------------------------------------------------------------------- */
static void sink_start_test(void)
{
    if (is_test_running) return;

    LOG_INF("\n>>> SINK: STARTING AUTO TEST (Duration: %d ms) <<<", TEST_DURATION_MS);
    
    /* 0. Bật thống kê gói tin */
    pkt_stats_set_enabled(true);
    
    /* 1. Broadcast lệnh START (Gửi 3 lần để đảm bảo độ tin cậy) */
    pkt_stats_reset();
    g_test_start_time = k_uptime_get_32();
    
    for (int i = 0; i < 3; i++) {
        bt_mesh_gradient_srv_send_test_start(&gradient_srv);
        k_sleep(K_MSEC(100)); // Chờ một chút giữa các lần phát
    }
    
    /* 2. Cài đặt Timer tự động dừng */
    k_work_schedule(&auto_stop_work, K_MSEC(TEST_DURATION_MS));
    is_test_running = true;
    
    /* 3. Log event */
    printk("CSV_LOG,EVENT,TEST_START,Duration_ms=%d\n", TEST_DURATION_MS);
    
    /* Nháy đèn báo hiệu */
    led_indicate_attention(true);
    k_sleep(K_MSEC(1000));
    led_indicate_attention(false);
}

static void sink_stop_test(void)
{
    if (!is_test_running) return;

    LOG_INF("\n>>> SINK: STOPPING TEST & REQUESTING REPORT <<<");

    /* 0. Ngừng thống kê gói tin */
    pkt_stats_set_enabled(false);

    /* 1. Broadcast lệnh STOP & REPORT */
    bt_mesh_gradient_srv_send_report_req(&gradient_srv);
    
    /* 2. Hủy timer (đề phòng trường hợp gọi thủ công trước khi hết giờ) */
    k_work_cancel_delayable(&auto_stop_work);
    is_test_running = false;
    
    /* 3. Log event */
    printk("CSV_LOG,EVENT,TEST_STOP,Finished\n");
    
    /* Nháy đèn báo hiệu */
    led_indicate_attention(true);
    k_sleep(K_MSEC(1000));
    led_indicate_attention(false);
}

/* -------------------------------------------------------------------------
 * HANDLERS
 * ------------------------------------------------------------------------- */

static void auto_stop_handler(struct k_work *work)
{
    LOG_INF("=== TEST TIMER EXPIRED ===");
    sink_stop_test();
}

/* [QUAN TRỌNG] Hàm gửi dữ liệu của Sensor */
static void send_data_handler(struct k_work *work)
{
    /* CHỐT CHẶN (GUARD CLAUSE): 
     * Nếu đã nhận lệnh STOP, tuyệt đối không chạy tiếp logic bên dưới.
     * Điều này ngăn chặn việc gửi dư gói tin (PDR > 100%).
     */
    if (!is_sending_active) {
        return; 
    }

    if (!bt_mesh_is_provisioned()) return;

    /* Tìm đường về Sink */
    bool has_route = false;
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv.forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
            has_route = true;
            break;
        }
    }

    if (has_route) {
        g_total_tx_count++; // Tăng SeqNum toàn cục
        g_test_data_tx_count++; // Tăng số gói DATA của bài test
        uint16_t dest_addr = gradient_srv.forwarding_table[0].addr; 

        LOG_DBG("[AUTO-SEND] Data Seq=%u -> 0x%04x", g_total_tx_count, dest_addr);
        
        /* [RELATIVE TIMESTAMP] Gửi thời gian tương đối tính từ lúc bắt đầu test */
        uint32_t timestamp = k_uptime_get_32() - (g_test_start_time > 0 ? g_test_start_time : k_uptime_get_32());

        /* [FIX PDR > 100%] Đếm Tx ngay khi phát sinh gói tin 
         * thay vì đợi Callback gửi xong (tránh việc báo cáo xong mới cộng Tx).
         */
        pkt_stats_inc_data_tx(); 
        
        int err = bt_mesh_gradient_srv_data_send(&gradient_srv, dest_addr, g_total_tx_count, timestamp);
        if (err) {
            LOG_WRN("Send failed (err %d)", err);
        } else {
            led_indicate_data_forwarded();
        }
    } else {
        LOG_WRN("[AUTO-SEND] No route! Waiting...");
    }

    /* CHỈ LÊN LỊCH TIẾP NẾU VẪN CÒN ĐANG CHẠY */
    if (is_sending_active) {
        /* Thêm Jitter nhỏ vào mỗi gói tin để tránh hiện tượng rượt đuổi (Sync collision) */
        uint32_t next_jitter = sys_rand32_get() % 200;
       k_work_reschedule(&send_data_work, K_MSEC(SENSOR_SEND_INTERVAL_MS + next_jitter));
    }
}

static void print_neighbor_table(void)
{
    struct bt_mesh_gradient_srv *srv = &gradient_srv;
    int64_t now = k_uptime_get();
    int count = 0;
    
    uint16_t my_addr = bt_mesh_model_elem(srv->model)->rt->addr;

    LOG_INF("\n================ NEIGHBOR TABLE (My Addr: 0x%04x, Grad: %d) ================", 
            my_addr, srv->gradient);
    LOG_INF("| Idx |  Addr  | Grad | RSSI |  Age (ms)  | Status    |");
    LOG_INF("|-----|--------|------|------|------------|-----------|");

    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        neighbor_entry_t *e = &srv->forwarding_table[i];

        if (e->addr != BT_MESH_ADDR_UNASSIGNED) {
            count++;
            
            char *status = "      ";
            if (e->gradient < srv->gradient) {
                if (e->rssi < -65) status = "WEAK_P"; 
                else status = "PARENT";               
            } else if (e->gradient > srv->gradient) {
                status = "CHILD ";
            } else {
                status = "PEER  ";
            }

            LOG_INF("| %3d | 0x%04x | %4d | %4d | %10lld | %s |", 
                    i, e->addr, e->gradient, e->rssi, (now - e->last_seen), status);
        }
    }

    if (count == 0) {
        LOG_INF("|                  (Empty Table)                  |");
    }
    LOG_INF("==========================================================\n");
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & button_state & DK_BTN1_MSK) 
    {
        // Debug thủ công
    }
    
   if (has_changed & button_state & DK_BTN2_MSK)
    {
        LOG_INF("\n=== Debug Info ===");
        LOG_INF(" Role: %s", (gradient_srv.gradient == 0) ? "SINK" : "SENSOR");
        LOG_INF(" Total Packets Sent: %u", g_total_tx_count);
        print_neighbor_table(); 
        LOG_INF("==================\n");
    }

    /* Xử lý Button 3 với cơ chế chống rung phím */
    if (has_changed & button_state & DK_BTN3_MSK)
    {
        int64_t now = k_uptime_get();

        /* Nút này dành riêng cho Sink Node (Gateway) */
        if (gradient_srv.gradient != 0) {
            LOG_WRN("Button 3 is reserved for Sink Node to control test.");
            return;
        }

        /* Chống rung phím (Debounce) - Cooldown 2s */
        if (now - last_action_time < ACTION_COOLDOWN_MS) {
            LOG_WRN("Network busy. Please wait...");
            return;
        }
        last_action_time = now;

        /* Toggle Test State */
        if (!is_test_running) {
            sink_start_test();
        } 
        else {
            sink_stop_test();
        }
    }
}