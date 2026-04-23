/*
 * sensor_shell.c – Zephyr Shell commands để khai báo / quản lý cảm biến
 *
 * Cách dùng (gõ trên Serial Monitor 115200 baud):
 *
 *   sensor list                           – liệt kê tất cả cảm biến đã đăng ký
 *   sensor add <gpio> <id> [ch_idx]       – đăng ký phần cứng (gpio + ADC channel)
 *   sensor del <gpio>                     – hủy đăng ký cảm biến theo GPIO
 *   sensor teds <id> <type> <unit> <scale> <offset> <min> <max>
 *                                         – khai báo chức năng cảm biến (TEDS)
 *   sensor teds_show <id>                 – xem TEDS đã lưu của cảm biến
 *   sensor read <id>                      – đọc raw + giá trị vật lý
 *   sensor readall                        – đọc tất cả, gửi Mesh OP_SENSOR_DATA
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "sensor_manager.h"
#include "storage.h"

LOG_MODULE_REGISTER(sensor_shell, LOG_LEVEL_INF);

/* ---------------------------------------------------------------
 * Buffer TEDS dùng chung cho các lệnh shell
 * --------------------------------------------------------------- */
#define TEDS_BUF_SIZE 256

/* ---------------------------------------------------------------
 * sensor list
 * --------------------------------------------------------------- */
static int cmd_sensor_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "=== Danh sach cam bien ===");
    list_registered_sensors();
    shell_print(sh, "(Xem them trong log phia tren)");
    return 0;
}

/* ---------------------------------------------------------------
 * sensor add <gpio_pin> <sensor_id> [ch_idx]
 * --------------------------------------------------------------- */
static int cmd_sensor_add(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        shell_error(sh, "Cu phap: sensor add <gpio_pin> <sensor_id> [ch_idx]");
        return -EINVAL;
    }

    int gpio_pin  = atoi(argv[1]);
    int sensor_id = atoi(argv[2]);
    int ch_idx    = (argc == 4) ? atoi(argv[3]) : -1;

    int ret = register_sensor_adc(gpio_pin, (uint8_t)sensor_id, ch_idx);
    if (ret == SENSOR_OK) {
        save_sensor_reg_adc(gpio_pin, (uint8_t)sensor_id, ch_idx);
        shell_print(sh, "OK: GPIO %d -> sensor_id %d, ADC ch%d [da luu Flash]",
                    gpio_pin, sensor_id, ch_idx);
    } else {
        shell_error(sh, "ERR: %d", ret);
    }
    return ret < 0 ? ret : 0;
}

/* ---------------------------------------------------------------
 * sensor del <gpio_pin>
 * --------------------------------------------------------------- */
static int cmd_sensor_del(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Cu phap: sensor del <gpio_pin>");
        return -EINVAL;
    }

    int gpio_pin = atoi(argv[1]);
    int ret = unregister_sensor(gpio_pin);

    if (ret == SENSOR_OK) {
        delete_sensor_reg(gpio_pin);
        shell_print(sh, "OK: Da huy dang ky GPIO %d", gpio_pin);
    } else {
        shell_error(sh, "ERR: %d", ret);
    }
    return ret < 0 ? ret : 0;
}

/* ---------------------------------------------------------------
 * sensor teds <sensor_id> <type> <unit> <scale> <offset> <min> <max>
 * --------------------------------------------------------------- */
static int cmd_sensor_teds(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 8) {
        shell_error(sh, "Cu phap: sensor teds <id> <type> <unit> <scale> <offset> <min> <max>");
        return -EINVAL;
    }

    int     sensor_id = atoi(argv[1]);
    char   *type      = argv[2];
    char   *unit      = argv[3];
    float   scale     = (float)strtod(argv[4], NULL);
    float   offset    = (float)strtod(argv[5], NULL);
    float   rmin      = (float)strtod(argv[6], NULL);
    float   rmax      = (float)strtod(argv[7], NULL);

    static char teds_json[TEDS_BUF_SIZE];
    snprintf(teds_json, sizeof(teds_json),
        "{\"type\":\"%s\",\"unit\":\"%s\",\"scale\":%.6f,\"offset\":%.6f,\"range_min\":%.2f,\"range_max\":%.2f}",
        type, unit, (double)scale, (double)offset, (double)rmin, (double)rmax);

    int ret = save_teds_to_nvs((uint8_t)sensor_id, teds_json);
    if (ret < 0) {
        shell_error(sh, "ERR: Luu Flash that bai (%d)", ret);
        return ret;
    }

    shell_print(sh, "OK: Da luu TEDS sensor_id=%d vao Flash", sensor_id);
    return 0;
}

/* ---------------------------------------------------------------
 * sensor teds_show <sensor_id>
 * --------------------------------------------------------------- */
static int cmd_sensor_teds_show(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) return -EINVAL;
    int sensor_id = atoi(argv[1]);
    static char teds_buf[TEDS_BUF_SIZE];
    int ret = load_teds_from_nvs((uint8_t)sensor_id, teds_buf, sizeof(teds_buf));
    if (ret == 0) {
        shell_print(sh, "JSON: %s", teds_buf);
    } else {
        shell_error(sh, "ERR: %d", ret);
    }
    return ret;
}

/* ---------------------------------------------------------------
 * sensor read <sensor_id>
 * --------------------------------------------------------------- */
static int cmd_sensor_read(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) return -EINVAL;
    int sid = atoi(argv[1]);
    int raw = read_sensor_raw((uint8_t)sid);
    if (raw < 0) {
        shell_error(sh, "Doc loi (%d)", raw);
        return raw;
    }
    shell_print(sh, "ID=%d raw=%d", sid, raw);
    
    static char teds_buf[TEDS_BUF_SIZE];
    if (load_teds_from_nvs(sid, teds_buf, sizeof(teds_buf)) == 0) {
        teds_config_t cfg;
        if (apply_teds_config(teds_buf, &cfg) == 0 && cfg.valid) {
            float physical = 0.0f;
            if (read_sensor_physical(sid, &cfg, &physical) == 0) {
                shell_print(sh, "  Value: %.3f %s", (double)physical, cfg.unit);
            }
        }
    }
    return 0;
}

/* ---------------------------------------------------------------
 * sensor cal <sensor_id> <raw1> <ref1> <raw2> <ref2> <unit> <type> <min> <max>
 * --------------------------------------------------------------- */
static int cmd_sensor_cal(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 10) return -EINVAL;
    int     sid   = atoi(argv[1]);
    float   raw1  = (float)strtod(argv[2], NULL);
    float   ref1  = (float)strtod(argv[3], NULL);
    float   raw2  = (float)strtod(argv[4], NULL);
    float   ref2  = (float)strtod(argv[5], NULL);
    char   *unit  = argv[6];
    char   *type  = argv[7];
    float   rmin  = (float)strtod(argv[8], NULL);
    float   rmax  = (float)strtod(argv[9], NULL);

    static char teds_json[TEDS_BUF_SIZE];
    snprintf(teds_json, sizeof(teds_json),
        "{\"type\":\"%s\",\"unit\":\"%s\",\"cal_raw1\":%.1f,\"cal_ref1\":%.6f,\"cal_raw2\":%.1f,\"cal_ref2\":%.6f,\"range_min\":%.2f,\"range_max\":%.2f}",
        type, unit, (double)raw1, (double)ref1, (double)raw2, (double)ref2, (double)rmin, (double)rmax);

    int ret = save_teds_to_nvs((uint8_t)sid, teds_json);
    if (ret == 0) shell_print(sh, "OK: Da luu hieu chinh 2 diem");
    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sensor_cmds,
    SHELL_CMD_ARG(list, NULL, "List sensors", cmd_sensor_list, 1, 0),
    SHELL_CMD_ARG(add, NULL, "Add sensor <gpio> <id> [ch]", cmd_sensor_add, 3, 1),
    SHELL_CMD_ARG(del, NULL, "Del sensor <gpio>", cmd_sensor_del, 2, 0),
    SHELL_CMD_ARG(teds, NULL, "TEDS linear <id> <type> <unit> <scale> <offset> <min> <max>", cmd_sensor_teds, 8, 0),
    SHELL_CMD_ARG(cal, NULL, "Cal 2pt <id> <raw1> <ref1> <raw2> <ref2> <unit> <type> <min> <max>", cmd_sensor_cal, 10, 0),
    SHELL_CMD_ARG(teds_show, NULL, "Show TEDS", cmd_sensor_teds_show, 2, 0),
    SHELL_CMD_ARG(read, NULL, "Read sensor <id>", cmd_sensor_read, 2, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensor, &sensor_cmds, "Sensor management", NULL);
