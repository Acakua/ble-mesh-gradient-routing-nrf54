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
#include <zephyr/sys/reboot.h>

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
#define TEST_DURATION_MINUTES   60
#define TEST_DURATION_MS        (TEST_DURATION_MINUTES * 60 * 1000) 

// 2. Cấu hình cho Sensor Node (Tần suất gửi tin)
#define SENSOR_SEND_INTERVAL_MS 10000

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
static bool is_sensor_test_active = false; /* [NEW] Manual toggle via Button 3 */

// --- SINK STRESS TEST VARIABLES ---
static struct k_work_delayable stress_tx_work;
static struct k_work_delayable stress_timeout_work;
static bool is_sink_stress_active = false;
static uint16_t stress_target_addr = BT_MESH_ADDR_UNASSIGNED;
static uint32_t g_stress_tx_count = 0; /* [NEW] Counter for downlink stress test */

/* Forward declarations */
static void button_handler(uint32_t button_state, uint32_t has_changed);
static void auto_stop_handler(struct k_work *work); 
static void send_data_handler(struct k_work *work); 
static void stress_tx_handler(struct k_work *work); /* [NEW] */
static void stress_timeout_handler(struct k_work *work); /* [NEW] */
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
    /* Node Sink không xử lý lệnh của chính mình */
    if (srv->gradient == 0) return;

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
    
    /* 4. [NEW] Staggered Start: Trì hoãn theo Gradient để tránh nghẽn tại Sink
     * Delay = (Gradient * 60000ms) + Jitter(0-40000ms)
     * Giúp các node bắt đầu cách nhau vài phút để ổn định mạng.
     */
    uint32_t start_delay = (srv->gradient * 60000) + (sys_rand32_get() % 40000); 
    k_work_schedule(&send_data_work, K_MSEC(start_delay));
    
    /* 5. Nháy đèn liên tục để báo hiệu đang TRONG CHẾ ĐỘ TEST */
    led_indicate_attention(true);

    LOG_INF(">> Auto-sending STARTED (Delay: %d ms)", start_delay);
}

/* [SENSOR] Xử lý khi nhận REPORT REQ (STOP) từ Sink */
static void handle_report_req_received(struct bt_mesh_gradient_srv *srv)
{
    /* Node Sink không xử lý lệnh của chính mình */
    if (srv->gradient == 0) return;

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
    
    /* [NEW] Sink Stress Test Timers */
    k_work_init_delayable(&stress_tx_work, stress_tx_handler);
    k_work_init_delayable(&stress_timeout_work, stress_timeout_handler);

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
void sink_start_test(void)
{
    if (is_test_running) return;

    LOG_INF("\n>>> SINK: STARTING AUTO TEST (Duration: %d ms) <<<", TEST_DURATION_MS);
    
    /* 0. Bật thống kê gói tin */
    pkt_stats_set_enabled(true);
    
    /* 1. Broadcast lệnh START (Gửi 3 lần với CÙNG ID để đảm bảo độ tin cậy) */
    pkt_stats_reset();
    g_test_start_time = k_uptime_get_32();
    // Phát lần đầu với force_new_id = true để tăng ID máy phát
    bt_mesh_gradient_srv_send_test_start(&gradient_srv, true);
    
    // Hai lần sau phát lại với cùng ID đó (force_new_id = false)
    for (int i = 0; i < 2; i++) {
        k_sleep(K_MSEC(20));
        bt_mesh_gradient_srv_send_test_start(&gradient_srv, false);
    }
    
    /* 2. Cài đặt Timer tự động dừng */
    k_work_schedule(&auto_stop_work, K_MSEC(TEST_DURATION_MS));
    is_test_running = true;
    
    /* 3. Log event */
    printk("CSV_LOG,EVENT,TEST_START,Duration_ms=%d\n", TEST_DURATION_MS);
    
    /* Nháy đèn báo hiệu */
    led_indicate_attention(true);
}

void sink_stop_test(void)
{
    if (!is_test_running && !is_sink_stress_active) return;

    /* 0. Ngừng thống kê gói tin */
    bool was_broadcast = is_test_running;
    bool was_stress = is_sink_stress_active;
    
    pkt_stats_set_enabled(false);

    if (was_broadcast) {
        LOG_INF("\n>>> SINK: STOPPING BROADCAST TEST & REQUESTING REPORT <<<");
        /* 1. Broadcast lệnh STOP & REPORT */
        bt_mesh_gradient_srv_send_report_req(&gradient_srv, true);
        for (int i = 0; i < 2; i++) {
            k_sleep(K_MSEC(100));
            bt_mesh_gradient_srv_send_report_req(&gradient_srv, false);
        }
        k_work_cancel_delayable(&auto_stop_work);
        is_test_running = false;
        printk("CSV_LOG,EVENT,TEST_STOP,Finished\n");
    }

    if (was_stress) {
        LOG_INF("\n>>> SINK: STOPPING STRESS TEST & SENDING REPORT <<<");
        is_sink_stress_active = false;
        k_work_cancel_delayable(&stress_tx_work);
        k_work_cancel_delayable(&stress_timeout_work);
        
        LOG_INF("Sending Downlink Report to 0x%04x (TX: %u)...", stress_target_addr, g_stress_tx_count);
        bt_mesh_gradient_srv_send_downlink_report(&gradient_srv, stress_target_addr, (uint16_t)g_stress_tx_count);
    }

    /* Nháy đèn báo hiệu kết thúc */
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
    if (!is_sending_active && !is_sensor_test_active) {
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
        
        /* [REMOVED] Timestamp logic (Now handled by Ping-Pong RTT) */

        /* [FIX PDR > 100%] Đếm Tx ngay khi phát sinh gói tin 
         * thay vì đợi Callback gửi xong (tránh việc báo cáo xong mới cộng Tx).
         */
        pkt_stats_inc_data_tx(); 
        
        /* [NEW] Khởi tạo Min RSSI = 0 (giá trị lớn nhất) cho gói tin gốc (Timestamp removed) */
        int err = bt_mesh_gradient_srv_data_send(&gradient_srv, dest_addr, g_total_tx_count, 0);
        if (err) {
            LOG_WRN("Send failed (err %d)", err);
        } else {
            led_indicate_data_forwarded();
        }
    } else {
        LOG_WRN("[AUTO-SEND] No route! Waiting...");
    }

    /* CHỈ LÊN LỊCH TIẾP NẾU VẪN CÒN ĐANG CHẠY */
    if (is_sending_active || is_sensor_test_active) {
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
            /* [NEW] Sensor Node Local Test Mode */
            if (!is_sensor_test_active) {
                LOG_INF(">>> SENSOR: STARTING LOCAL TEST <<<");
                is_sensor_test_active = true;
                g_test_data_tx_count = 0;
                pkt_stats_set_enabled(true);
                pkt_stats_reset();
                k_work_schedule(&send_data_work, K_NO_WAIT);
            } else {
                LOG_INF(">>> SENSOR: STOPPING LOCAL TEST <<<");
                is_sensor_test_active = false;
                k_work_cancel_delayable(&send_data_work);
                
                /* Report Local Stats */
                struct packet_stats stats;
                pkt_stats_get(&stats);
                LOG_INF("Local Test Stopped. TX: %u, RX Backprop: %u", stats.data_tx, stats.rx_data_count);
            }
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
    if (has_changed & button_state & DK_BTN4_MSK)
    {
        LOG_WRN("\n>>> BUTTON 4 PRESSED: REBOOTING SYSTEM... <<<");
        
        /* [QUAN TRỌNG] Ngủ ngắn 200ms để hệ thống kịp in dòng Log trên ra UART 
         * trước khi bị cắt điện khởi động lại.
         */
        k_sleep(K_MSEC(200)); 
        
        /* Thực hiện Cold Reboot (Tương đương bấm nút Reset cứng) */
        sys_reboot(SYS_REBOOT_COLD);
    }
}

void sink_start_stress_test(uint16_t target_addr)
{
    if (gradient_srv.gradient != 0) {
        LOG_WRN("Only Sink can start stress test");
        return;
    }

    if (is_test_running || is_sink_stress_active) {
        LOG_WRN("Test already running. Stop it first.");
        return;
    }

    LOG_INF(">>> SINK: STARTING STRESS TEST to 0x%04x (Duration: %d ms) <<<", target_addr, TEST_DURATION_MS);
    
    is_sink_stress_active = true;
    stress_target_addr = target_addr;
    
    /* Reset counters at Sink to track what we send */
    pkt_stats_set_enabled(true);
    pkt_stats_reset();
    g_test_start_time = k_uptime_get_32();
    g_stress_tx_count = 0; /* Reset stress counter */

    /* [NEW] Send STATS RESET Signal to Target (Payload 0xFFFD) */
    /* This ensures the sensor resets its RX counters before we start flooding */
    LOG_INF("Sending STATS RESET signal to 0x%04x...", target_addr);
    bt_mesh_gradient_srv_backprop_send(&gradient_srv, stress_target_addr, 0xFFFD);
    
    /* Small delay to ensure Reset arrives before Data */
    k_sleep(K_MSEC(100));

    /* 1. Start sending loop */
    k_work_schedule(&stress_tx_work, K_NO_WAIT);
    
    /* 2. Start timeout to stop */
    k_work_schedule(&stress_timeout_work, K_MSEC(TEST_DURATION_MS));
    
    /* Nháy đèn báo hiệu */
    led_indicate_attention(true);
}

static void stress_tx_handler(struct k_work *work)
{
    if (!is_sink_stress_active) return;
    
    /* Send Backprop */
    g_total_tx_count++;
    g_stress_tx_count++;   /* Increment stress counter */
    pkt_stats_inc_data_tx(); // Count TX at Sink
    
    /* Gửi Payload là sequence number tăng dần */
    int err = bt_mesh_gradient_srv_backprop_send(&gradient_srv, stress_target_addr, g_total_tx_count);
    if (err) {
        LOG_WRN("Stress Backprop failed: %d", err);
    }
    
    /* Schedule next */
    if (is_sink_stress_active) {
         /* Gửi mỗi 1s */
        k_work_reschedule(&stress_tx_work, K_MSEC(1000));
    }
}

static void stress_timeout_handler(struct k_work *work)
{
    LOG_INF("=== STRESS TEST FINISHED ===");
    is_sink_stress_active = false;
    k_work_cancel_delayable(&stress_tx_work);
    led_indicate_attention(false);
    
    /* Send Final Downlink Report */
    LOG_INF("Sending Final Downlink Report to 0x%04x (TX: %u)...", stress_target_addr, g_stress_tx_count);
    bt_mesh_gradient_srv_send_downlink_report(&gradient_srv, stress_target_addr, (uint16_t)g_stress_tx_count);
}