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

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_DBG);

/* ==========================================
 * CẤU HÌNH TEST (TEST CONFIGURATION)
 * ========================================== */
// 1. Cấu hình cho Sink Node (Thời gian chạy test tối đa)
#define TEST_DURATION_HOURS    1
#define TEST_DURATION_MS       (TEST_DURATION_HOURS * 60 * 1000) 

// 2. Cấu hình cho Sensor Node (Tần suất gửi tin)
#define SENSOR_SEND_INTERVAL_MS 1000  // Gửi 1 gói mỗi giây

/* ==========================================
 * GLOBALS & WORK ITEMS
 * ========================================== */
static const struct shell *chat_shell;
uint16_t g_total_tx_count = 0; // Biến đếm tổng số gói đã gửi (SeqNum)

// --- SINK NODE VARIABLES ---
static struct k_work_delayable auto_stop_work;
static bool is_test_running = false;

// --- SENSOR NODE VARIABLES ---
static struct k_work_delayable send_data_work;
static bool is_sending_active = false; /* MẶC ĐỊNH LÀ FALSE - CHỜ LỆNH TỪ SINK */

/* Forward declarations */
static void button_handler(uint32_t button_state, uint32_t has_changed);
static void auto_stop_handler(struct k_work *work); // Cho Sink (Timer Stop)
static void send_data_handler(struct k_work *work); // Cho Sensor (Loop Send)

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/

static void attention_on(const struct bt_mesh_model *mod)
{
    led_indicate_attention(true);
}

static void attention_off(const struct bt_mesh_model *mod)
{
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

/* [NEW] Xử lý khi nhận lệnh TEST START từ Sink (Tại Sensor Node) */
static void handle_test_start_received(struct bt_mesh_gradient_srv *srv)
{
    LOG_INF(">> EVENT: START COMMAND RECEIVED FROM SINK <<");
    
    /* [MODIFIED] Reset biến đếm khi bắt đầu test mới */
    g_total_tx_count = 0; 
    LOG_INF(">> Counter Reset to 0 for new test.");

    if (!is_sending_active) {
        is_sending_active = true;
        k_work_schedule(&send_data_work, K_NO_WAIT); // Bắt đầu gửi ngay lập tức
        LOG_INF(">> Auto-sending STARTED");
        
        /* Bật đèn nháy để báo hiệu đang trong quá trình Test */
        led_indicate_attention(true); 
    }
}

/* [NEW] Xử lý khi nhận REPORT REQ (STOP) từ Sink (Tại Sensor Node) */
static void handle_report_req_received(struct bt_mesh_gradient_srv *srv)
{
    LOG_WRN(">> EVENT: STOP REQUEST RECEIVED FROM SINK <<");
    
    /* 1. NGỪNG GỬI NGAY LẬP TỨC */
    if (is_sending_active) {
        is_sending_active = false;
        k_work_cancel_delayable(&send_data_work);
        LOG_INF(">> Auto-sending STOPPED.");
    }

    /* 2. CHUẨN BỊ SỐ LIỆU & RESET */
    uint16_t final_count = g_total_tx_count; // Lưu lại số gói đã gửi
    g_total_tx_count = 0; // [MODIFIED] Reset ngay lập tức
    LOG_INF(">> Final Count: %u (Resetting counter to 0)", final_count);
    
    /* 3. TẮT ĐÈN NHÁY (Để không bị nháy liên tục gây khó chịu) */
    led_indicate_attention(false); 
    
    /* 4. GỬI BÁO CÁO VỀ SINK */
    LOG_INF(">> Reporting TotalTx (%u) to Sink...", final_count);
    
    /* Delay ngẫu nhiên nhỏ để tránh xung đột */
    k_sleep(K_MSEC(10 + (sys_rand32_get() % 500)));

    int err = bt_mesh_gradient_srv_report_rsp_send(srv, final_count);
    
    if (err) {
        LOG_ERR("Failed to send REPORT RSP (err %d)", err);
    } 
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
    
    /* 1. Broadcast lệnh START */
    bt_mesh_gradient_srv_send_test_start(&gradient_srv);
    
    /* 2. Cài đặt Timer tự động dừng (Failsafe) */
    k_work_schedule(&auto_stop_work, K_MSEC(TEST_DURATION_MS));
    is_test_running = true;
    
    /* 3. Log event */
    printk("CSV_LOG,EVENT,TEST_START,Duration_ms=%d\n", TEST_DURATION_MS);
    
    /* Nháy đèn 1 chút để báo hiệu lệnh đã gửi, sau đó tắt */
    led_indicate_attention(true);
    k_sleep(K_MSEC(1000));
    led_indicate_attention(false);
}

static void sink_stop_test(void)
{
    if (!is_test_running) return;

    LOG_INF("\n>>> SINK: STOPPING TEST & REQUESTING REPORT <<<");

    /* 1. Broadcast lệnh STOP & REPORT */
    bt_mesh_gradient_srv_send_report_req(&gradient_srv);
    
    /* 2. Hủy timer (nếu dừng bằng nút bấm) */
    k_work_cancel_delayable(&auto_stop_work);
    is_test_running = false;
    
    /* 3. Log event */
    printk("CSV_LOG,EVENT,TEST_STOP,Finished\n");

    /* Nháy đèn báo hiệu lệnh dừng, sau đó tắt */
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

static void send_data_handler(struct k_work *work)
{
    if (!is_sending_active) return;
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
        g_total_tx_count++; // Tăng SeqNum
        uint16_t dest_addr = gradient_srv.forwarding_table[0].addr; 

        LOG_INF("[AUTO-SEND] Data Seq=%u -> 0x%04x", g_total_tx_count, dest_addr);
        
        int err = bt_mesh_gradient_srv_data_send(&gradient_srv, dest_addr, g_total_tx_count);
        if (err) {
            LOG_WRN("Send failed (err %d)", err);
        } else {
            led_indicate_data_forwarded();
        }
    } else {
        LOG_WRN("[AUTO-SEND] No route! Waiting...");
    }

    if (is_sending_active) {
        k_work_schedule(&send_data_work, K_MSEC(SENSOR_SEND_INTERVAL_MS));
    }
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & button_state & DK_BTN1_MSK) 
    {
        LOG_INF("Button 1 pressed: Manual debug send (Single packet)");
        if (bt_mesh_is_provisioned() && gradient_srv.gradient != 0) {
            // send_data_handler(NULL); // Uncomment nếu muốn gửi thủ công
        }
    }
    
    if (has_changed & button_state & DK_BTN2_MSK)
    {
        LOG_INF("\n=== Debug Info ===");
        LOG_INF(" Role: %s", (gradient_srv.gradient == 0) ? "SINK" : "SENSOR");
        LOG_INF(" Total Packets Sent: %u", g_total_tx_count);
        if (gradient_srv.gradient == 0) {
            LOG_INF(" Test Running: %s", is_test_running ? "YES" : "NO");
        } else {
            LOG_INF(" Sending Active: %s", is_sending_active ? "YES" : "NO");
        }
        LOG_INF("==================\n");
    }

    if (has_changed & button_state & DK_BTN3_MSK)
    {
        if (gradient_srv.gradient != 0) {
            LOG_WRN("Button 3 is reserved for Sink Node to control test.");
            return;
        }

        if (!is_test_running) {
            sink_start_test();
        } 
        else {
            sink_stop_test();
        }
    }
}