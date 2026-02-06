/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell Commands cho Gradient Routing
 *
 * File này định nghĩa các lệnh shell để điều khiển và debug hệ thống
 * gradient routing qua UART terminal.
 *
 * Danh sách lệnh:
 * ===============
 *
 * mesh info
 *   - Hiển thị thông tin cơ bản của node: địa chỉ, gradient, trạng thái
 *
 * mesh fwd
 *   - In ra Forwarding Table (bảng định tuyến chính)
 *   - Hiển thị danh sách neighbor với gradient, rssi, thời gian
 *
 * mesh rrt
 *   - In ra Reverse Routing Table (bảng định tuyến ngược)
 *   - Hiển thị các destination có thể gửi BACKPROP
 *
 * mesh dest
 *   - Liệt kê tất cả destination có thể gửi BACKPROP
 *   - Hiển thị destination và nexthop tương ứng
 *
 * mesh backprop <dest_addr> <payload>
 *   - Gửi BACKPROP_DATA đến địa chỉ cụ thể
 *   - dest_addr: Địa chỉ đích (hex, ví dụ: 0x0003 hoặc 3)
 *   - payload: Dữ liệu gửi đi (số nguyên 0-65535)
 *   - Chỉ hoạt động trên Gateway (gradient = 0)
 *
 * mesh data <payload>
 *   - Gửi DATA packet lên Gateway
 *   - payload: Dữ liệu gửi đi (số nguyên 0-65535)
 *   - Chỉ hoạt động trên Regular Node (gradient > 0)
 *
 * mesh heartbeat
 *   - Hiển thị trạng thái heartbeat (active/inactive)
 */

#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

#include "gradient_srv.h"
#include "reverse_routing.h"
#include "heartbeat.h"
#include "data_forward.h"
#include "packet_stats.h"
#include "model_handler.h"

LOG_MODULE_REGISTER(shell_cmd, LOG_LEVEL_INF);

/*============================================================================*/
/*                         External References                                */
/*============================================================================*/

/* Tham chiếu đến gradient_srv được định nghĩa trong model_handler.c */
extern struct bt_mesh_gradient_srv gradient_srv;

/*============================================================================*/
/*                         Helper Functions                                   */
/*============================================================================*/

/**
 * @brief Kiểm tra node đã được provision chưa
 * @return true nếu đã provision, false nếu chưa
 */
static bool check_provisioned(const struct shell *sh)
{
    if (!bt_mesh_is_provisioned()) {
        shell_error(sh, "Node chua duoc provision!");
        return false;
    }
    return true;
}

/**
 * @brief Lấy địa chỉ của node hiện tại
 * @return Địa chỉ unicast của node
 */
static uint16_t get_my_addr(void)
{
    if (gradient_srv.model == NULL) {
        return 0;
    }
    return bt_mesh_model_elem(gradient_srv.model)->rt->addr;
}

/*============================================================================*/
/*                         Command: mesh info                                 */
/*============================================================================*/

/**
 * @brief Hiển thị thông tin cơ bản của node
 *
 * Lệnh: mesh info
 *
 * Output:
 *   - Địa chỉ unicast
 *   - Giá trị gradient
 *   - Vai trò (Gateway/Regular Node)
 *   - Trạng thái provision
 */
static int cmd_mesh_info(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "");
    shell_print(sh, "=== Thong Tin Node ===");

    if (!bt_mesh_is_provisioned()) {
        shell_print(sh, "Trang thai: CHUA PROVISION");
        shell_print(sh, "======================");
        return 0;
    }

    uint16_t my_addr = get_my_addr();
    uint8_t gradient = gradient_srv.gradient;

    shell_print(sh, "Dia chi    : 0x%04x", my_addr);
    shell_print(sh, "Gradient   : %d", gradient);
    shell_print(sh, "Vai tro    : %s", (gradient == 0) ? "GATEWAY" : "REGULAR NODE");
    shell_print(sh, "Heartbeat  : %s", heartbeat_is_active() ? "ACTIVE" : "INACTIVE");
    shell_print(sh, "======================");

    return 0;
}

/*============================================================================*/
/*                         Command: mesh fwd                                  */
/*============================================================================*/

/**
 * @brief In ra Forwarding Table
 *
 * Lệnh: mesh fwd
 *
 * Forwarding Table chứa danh sách neighbor (hàng xóm 1-hop) với thông tin:
 *   - Địa chỉ neighbor
 *   - Gradient của neighbor
 *   - RSSI (cường độ tín hiệu)
 *   - Thời gian từ lần cuối nhận beacon
 */
static int cmd_mesh_fwd(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!check_provisioned(sh)) {
        return -ENOEXEC;
    }

    shell_print(sh, "");
    shell_print(sh, "=== Forwarding Table ===");

    /* FIX: Add Mutex Lock */
    k_mutex_lock(&gradient_srv.forwarding_table_mutex, K_FOREVER);

    int64_t now = k_uptime_get();
    bool has_entry = false;

    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv.forwarding_table[i].addr != BT_MESH_ADDR_UNASSIGNED) {
            int64_t age_sec = (now - gradient_srv.forwarding_table[i].last_seen) / 1000;
            shell_print(sh, "[%d] addr=0x%04x  gradient=%d  rssi=%d  age=%lld sec",
                        i,
                        gradient_srv.forwarding_table[i].addr,
                        gradient_srv.forwarding_table[i].gradient,
                        gradient_srv.forwarding_table[i].rssi,
                        age_sec);
            has_entry = true;
        }
    }

    if (!has_entry) {
        shell_print(sh, "(trong - chua co neighbor)");
    }

    /* FIX: Add Mutex Unlock */
    k_mutex_unlock(&gradient_srv.forwarding_table_mutex);

    shell_print(sh, "========================");

    return 0;
}

/*============================================================================*/
/*                         Command: mesh rrt                                  */
/*============================================================================*/

/**
 * @brief In ra Reverse Routing Table
 *
 * Lệnh: mesh rrt
 *
 * RRT chứa thông tin để định tuyến BACKPROP (downlink):
 *   - Mỗi neighbor có một linked list các destination
 *   - "Để gửi đến dest X, gửi qua neighbor Y"
 */
static int cmd_mesh_rrt(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!check_provisioned(sh)) {
        return -ENOEXEC;
    }

    shell_print(sh, "");
    shell_print(sh, "=== Reverse Routing Table ===");

    /* FIX: Add Mutex Lock */
    k_mutex_lock(&gradient_srv.forwarding_table_mutex, K_FOREVER);

    int64_t now = k_uptime_get();
    int total_routes = 0;

    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv.forwarding_table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        /* Đếm số destination của neighbor này */
        int dest_count = 0;
        struct backprop_node *node = gradient_srv.forwarding_table[i].backprop_dest;
        while (node != NULL) {
            dest_count++;
            node = node->next;
        }

        if (dest_count == 0) {
            continue;
        }

        shell_print(sh, "Nexthop 0x%04x (%d destinations):",
                    gradient_srv.forwarding_table[i].addr, dest_count);

        /* In từng destination */
        node = gradient_srv.forwarding_table[i].backprop_dest;
        while (node != NULL) {
            int64_t age_sec = (now - node->last_seen) / 1000;
            shell_print(sh, "  -> dest=0x%04x (age=%lld sec)", node->addr, age_sec);
            total_routes++;
            node = node->next;
        }
    }

    if (total_routes == 0) {
        shell_print(sh, "(trong - chua hoc duoc route nao)");
        shell_print(sh, "Cho cac node gui heartbeat...");
    } else {
        shell_print(sh, "---");
        shell_print(sh, "Tong: %d reverse routes", total_routes);
    }

    /* FIX: Add Mutex Unlock */
    k_mutex_unlock(&gradient_srv.forwarding_table_mutex);

    shell_print(sh, "=============================");

    return 0;
}

/*============================================================================*/
/*                         Command: mesh dest                                 */
/*============================================================================*/

/**
 * @brief Liệt kê tất cả destination có thể gửi BACKPROP
 *
 * Lệnh: mesh dest
 *
 * Hiển thị danh sách gọn các destination và nexthop tương ứng.
 * Tiện lợi để chọn destination trước khi gửi BACKPROP.
 */
static int cmd_mesh_dest(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!check_provisioned(sh)) {
        return -ENOEXEC;
    }

    shell_print(sh, "");
    shell_print(sh, "=== Danh Sach Destination ===");

    /* FIX: Add Mutex Lock */
    k_mutex_lock(&gradient_srv.forwarding_table_mutex, K_FOREVER);

    int count = 0;

    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv.forwarding_table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        struct backprop_node *node = gradient_srv.forwarding_table[i].backprop_dest;
        while (node != NULL) {
            shell_print(sh, "  0x%04x  (via nexthop 0x%04x)",
                        node->addr,
                        gradient_srv.forwarding_table[i].addr);
            count++;
            node = node->next;
        }
    }

    if (count == 0) {
        shell_print(sh, "(khong co destination nao)");
    } else {
        shell_print(sh, "---");
        shell_print(sh, "Tong: %d destinations", count);
    }

    /* FIX: Add Mutex Unlock */
    k_mutex_unlock(&gradient_srv.forwarding_table_mutex);

    shell_print(sh, "=============================");
    shell_print(sh, "Dung: mesh backprop <dest> <payload>");

    return 0;
}

/*============================================================================*/
/*                         Command: mesh backprop                             */
/*============================================================================*/

/**
 * @brief Gửi BACKPROP_DATA đến địa chỉ cụ thể
 *
 * Lệnh: mesh backprop <dest_addr> <payload>
 *
 * Tham số:
 *   - dest_addr: Địa chỉ đích (hex hoặc decimal)
 *                Ví dụ: 0x0003 hoặc 3
 *   - payload:   Dữ liệu gửi đi (0-65535)
 *
 * Lưu ý:
 *   - Chỉ Gateway (gradient=0) mới có thể gửi BACKPROP
 *   - Destination phải tồn tại trong RRT
 */
static int cmd_mesh_backprop(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_error(sh, "Sai cu phap!");
        shell_print(sh, "Dung: mesh backprop <dest_addr> <payload>");
        shell_print(sh, "Vi du: mesh backprop 0x0003 123");
        shell_print(sh, "       mesh backprop 3 456");
        return -EINVAL;
    }

    if (!check_provisioned(sh)) {
        return -ENOEXEC;
    }

    /* Kiểm tra có phải Gateway không */
    if (gradient_srv.gradient != 0) {
        shell_error(sh, "Chi Gateway (gradient=0) moi co the gui BACKPROP!");
        shell_print(sh, "Node nay co gradient=%d", gradient_srv.gradient);
        return -ENOEXEC;
    }

    /* Parse địa chỉ đích */
    char *endptr;
    unsigned long dest_addr = strtoul(argv[1], &endptr, 0);
    if (*endptr != '\0' || dest_addr > 0xFFFF) {
        shell_error(sh, "Dia chi khong hop le: %s", argv[1]);
        return -EINVAL;
    }

    /* Parse payload */
    unsigned long payload = strtoul(argv[2], &endptr, 0);
    if (*endptr != '\0' || payload > 0xFFFF) {
        shell_error(sh, "Payload khong hop le: %s", argv[2]);
        return -EINVAL;
    }

    shell_print(sh, "");
    shell_print(sh, "Gui BACKPROP den 0x%04x voi payload=%lu...",
                (uint16_t)dest_addr, payload);

    /* Gửi BACKPROP */
    int err = bt_mesh_gradient_srv_backprop_send(&gradient_srv,
                                                  (uint16_t)dest_addr,
                                                  (uint16_t)payload);

    if (err == 0) {
        shell_print(sh, "BACKPROP da gui thanh cong!");
    } else if (err == -ENETUNREACH) {
        shell_error(sh, "Khong tim thay route den 0x%04x!", (uint16_t)dest_addr);
        shell_print(sh, "Dung 'mesh dest' de xem danh sach destination.");
    } else if (err == -EINVAL) {
        shell_error(sh, "Khong the gui den chinh minh!");
    } else {
        shell_error(sh, "Gui that bai, err=%d", err);
    }

    return err;
}

/*============================================================================*/
/*                         Command: mesh data                                 */
/*============================================================================*/

/**
 * @brief Gửi DATA packet lên Gateway
 *
 * Lệnh: mesh data <payload>
 *
 * Tham số:
 *   - payload: Dữ liệu gửi đi (0-65535)
 *
 * Lưu ý:
 *   - Chỉ Regular Node (gradient>0) mới cần gửi DATA
 *   - Gateway nhận DATA sẽ không forward tiếp
 */
static int cmd_mesh_data(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Sai cu phap!");
        shell_print(sh, "Dung: mesh data <payload>");
        shell_print(sh, "Vi du: mesh data 123");
        return -EINVAL;
    }

    if (!check_provisioned(sh)) {
        return -ENOEXEC;
    }

    /* Kiểm tra không phải Gateway */
    if (gradient_srv.gradient == 0) {
        shell_error(sh, "Gateway khong can gui DATA!");
        shell_print(sh, "Dung 'mesh backprop' de gui xuong node.");
        return -ENOEXEC;
    }

    /* FIX: Protect read of forwarding_table */
    k_mutex_lock(&gradient_srv.forwarding_table_mutex, K_FOREVER);
    uint16_t nexthop = gradient_srv.forwarding_table[0].addr;
    k_mutex_unlock(&gradient_srv.forwarding_table_mutex);

    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        shell_error(sh, "Chua co route den Gateway!");
        shell_print(sh, "Cho nhan gradient beacon...");
        return -ENOEXEC;
    }

    /* Parse payload */
    char *endptr;
    unsigned long payload = strtoul(argv[1], &endptr, 0);
    if (*endptr != '\0' || payload > 0xFFFF) {
        shell_error(sh, "Payload khong hop le: %s", argv[1]);
        return -EINVAL;
    }

    shell_print(sh, "");
    shell_print(sh, "Gui DATA voi payload=%lu qua nexthop 0x%04x...",
                payload, nexthop);

    /* Gửi DATA */
    int err = bt_mesh_gradient_srv_data_send(&gradient_srv, nexthop, (uint16_t)payload, 0);

    if (err == 0) {
        shell_print(sh, "DATA da gui thanh cong!");
    } else {
        shell_error(sh, "Gui that bai, err=%d", err);
    }

    return err;
}

/*============================================================================*/
/*                         Command: mesh heartbeat                            */
/*============================================================================*/

/**
 * @brief Hiển thị trạng thái heartbeat
 *
 * Lệnh: mesh heartbeat
 *
 * Hiển thị:
 *   - Heartbeat đang active hay không
 *   - Interval (khoảng cách giữa các heartbeat)
 */
static int cmd_mesh_heartbeat(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "");
    shell_print(sh, "=== Trang Thai Heartbeat ===");

#ifdef CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    shell_print(sh, "Config   : ENABLED");
    shell_print(sh, "Interval : %d giay", CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC);
    shell_print(sh, "Active   : %s", heartbeat_is_active() ? "YES" : "NO");

    if (gradient_srv.gradient == 0) {
        shell_print(sh, "Ghi chu  : Gateway khong gui heartbeat");
    }
#else
    shell_print(sh, "Config   : DISABLED");
#endif

    shell_print(sh, "============================");

    return 0;
}

/*============================================================================*/
/*                         Command: mesh report stop                          */
/*============================================================================*/

/**
 * @brief Điều khiển bài test (Start/Stop)
 * Lệnh: mesh report stop | start
 */
static int cmd_mesh_report_stop(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Su dung: mesh report start | stop");
        return -EINVAL;
    }

    if (strcmp(argv[1], "stop") == 0) {
        shell_print(sh, "Dang dung test qua Sink Control...");
        sink_stop_test();
    } else if (strcmp(argv[1], "start") == 0) {
        shell_print(sh, "Dang bat dau test qua Sink Control...");
        sink_start_test();
    } else {
        shell_print(sh, "Lenh khong hop le. Su dung: start | stop");
    }

    return 0;
}

/*============================================================================*/
/*                         Command: mesh stress_dl                            */
/*============================================================================*/

/**
 * @brief Test Downlink Stress (Unicast BACKPROP) -> REPORT
 * Lệnh: mesh stress_dl <target_addr>
 */
static int cmd_mesh_stress_dl(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(sh, "Su dung: mesh stress_dl <target_hex>");
        return -EINVAL;
    }

    char *endptr;
    unsigned long addr = strtoul(argv[1], &endptr, 0); // Auto detect hex/dec
    
    if (*endptr != '\0' || addr > 0xFFFF) {
        shell_error(sh, "Dia chi khong hop le: %s", argv[1]);
        return -EINVAL;
    }

    /* Kiểm tra có phải Gateway không */
    if (gradient_srv.gradient != 0) {
        shell_error(sh, "Chi Gateway (gradient=0) moi co the chay stress test!");
        return -ENOEXEC;
    }

    sink_start_stress_test((uint16_t)addr);
    return 0;
}

/*============================================================================*/
/*                         Command: mesh stats                                */
/*============================================================================*/

/**
 * @brief Hiển thị thống kê gói tin TX
 *
 * Lệnh: mesh stats
 *
 * Hiển thị:
 *   - Gradient Beacon TX count
 *   - Heartbeat TX count
 *   - DATA TX count
 *   - Control Overhead percentage
 */
static int cmd_mesh_stats_show(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct packet_stats stats;
    pkt_stats_get(&stats);

    uint32_t control_total = stats.gradient_beacon_tx + stats.heartbeat_tx;
    uint32_t all_total = control_total + stats.data_tx;

    shell_print(sh, "");
    shell_print(sh, "=== Thong Ke Goi Tin TX ===");
    shell_print(sh, "Gradient Beacon : %u", stats.gradient_beacon_tx);
    shell_print(sh, "Heartbeat       : %u", stats.heartbeat_tx);
    shell_print(sh, "DATA            : %u", stats.data_tx);
    shell_print(sh, "---------------------------");
    shell_print(sh, "CONTROL Total   : %u", control_total);
    shell_print(sh, "Total TX        : %u", all_total);

    if (all_total > 0) {
        uint32_t overhead_percent = (control_total * 100) / all_total;
        shell_print(sh, "Control Overhead: %u%%", overhead_percent);
    } else {
        shell_print(sh, "Control Overhead: N/A (chua co goi tin)");
    }

    shell_print(sh, "===========================");

    return 0;
}

/**
 * @brief Reset tất cả counters về 0
 *
 * Lệnh: mesh stats reset
 */
static int cmd_mesh_stats_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    pkt_stats_reset();
    shell_print(sh, "Da reset tat ca counters ve 0.");

    return 0;
}

/* Subcommands for stats */
SHELL_STATIC_SUBCMD_SET_CREATE(stats_subcmds,
    SHELL_CMD_ARG(reset, NULL,
        "Reset tat ca counters ve 0",
        cmd_mesh_stats_reset, 1, 0),
    SHELL_SUBCMD_SET_END
);

/*============================================================================*/
/*                         Shell Command Registration                         */
/*============================================================================*/

/**
 * Đăng ký các sub-command cho "mesh"
 *
 * Cú pháp: mesh <sub-command> [args...]
 */
SHELL_STATIC_SUBCMD_SET_CREATE(mesh_cmds,
    SHELL_CMD_ARG(info, NULL,
        "Hien thi thong tin node (dia chi, gradient, vai tro)",
        cmd_mesh_info, 1, 0),

    SHELL_CMD_ARG(fwd, NULL,
        "In Forwarding Table (danh sach neighbor)",
        cmd_mesh_fwd, 1, 0),

    SHELL_CMD_ARG(rrt, NULL,
        "In Reverse Routing Table (bang dinh tuyen nguoc)",
        cmd_mesh_rrt, 1, 0),

    SHELL_CMD_ARG(dest, NULL,
        "Liet ke tat ca destination co the gui BACKPROP",
        cmd_mesh_dest, 1, 0),

    SHELL_CMD_ARG(backprop, NULL,
        "Gui BACKPROP: mesh backprop <dest_addr> <payload>\n"
        "  Vi du: mesh backprop 0x0003 123",
        cmd_mesh_backprop, 3, 0),

    SHELL_CMD_ARG(report, NULL,
        "Dieu khien bao cao: mesh report stop\n"
        "  stop: Dung test va yeu cau bao cao tu tat ca node",
        cmd_mesh_report_stop, 2, 0),

    SHELL_CMD_ARG(data, NULL,
        "Gui DATA len Gateway: mesh data <payload>\n"
        "  Vi du: mesh data 456",
        cmd_mesh_data, 2, 0),

    SHELL_CMD_ARG(heartbeat, NULL,
        "Hien thi trang thai heartbeat",
        cmd_mesh_heartbeat, 1, 0),

    SHELL_CMD_ARG(stress_dl, NULL,
        "Chay Stress Test DL: mesh stress_dl <addr>",
        cmd_mesh_stress_dl, 2, 0),

    SHELL_CMD(stats, &stats_subcmds,
        "Thong ke goi tin TX (mesh stats | mesh stats reset)",
        cmd_mesh_stats_show),

    SHELL_SUBCMD_SET_END
);

/**
 * Đăng ký command chính "mesh"
 */
SHELL_CMD_REGISTER(mesh, &mesh_cmds,
    "Cac lenh dieu khien Gradient Routing Mesh\n"
    "  mesh info      - Thong tin node\n"
    "  mesh fwd       - Forwarding Table\n"
    "  mesh rrt       - Reverse Routing Table\n"
    "  mesh dest      - Danh sach destination\n"
    "  mesh backprop  - Gui BACKPROP (Gateway)\n"
    "  mesh data      - Gui DATA (Node)\n"
    "  mesh heartbeat - Trang thai heartbeat\n"
    "  mesh stats     - Thong ke goi tin TX",
    NULL);
