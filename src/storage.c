/*
 * storage.c
 * Lưu và đọc TEDS (IEEE 1451) từ Flash/NVS qua Zephyr Settings subsystem.
 * Key format: "wtim/teds/<sensor_id>"
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include "storage.h"

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

/* ---------------------------------------------------------------
 * Lưu TEDS vào Flash
 * --------------------------------------------------------------- */
int save_teds_to_nvs(uint8_t sensor_id, const char *json_str)
{
    if (!json_str || strlen(json_str) == 0) {
        LOG_WRN("save_teds_to_nvs: chuỗi JSON rỗng");
        return -EINVAL;
    }

    char key[32];
    snprintf(key, sizeof(key), "wtim/teds/%d", sensor_id);

    int ret = settings_save_one(key, json_str, strlen(json_str));
    if (ret < 0) {
        LOG_ERR("Lưu TEDS sensor %d thất bại: %d", sensor_id, ret);
    } else {
        LOG_INF("Đã lưu TEDS sensor %d (%zu bytes)", sensor_id, strlen(json_str));
    }
    return ret;
}

/* ---------------------------------------------------------------
 * Xóa TEDS của một sensor
 * --------------------------------------------------------------- */
int delete_teds_from_nvs(uint8_t sensor_id)
{
    char key[32];
    snprintf(key, sizeof(key), "wtim/teds/%d", sensor_id);
    return settings_delete(key);
}

/* ---------------------------------------------------------------
 * Lưu / Load danh sách sensor đã đăng ký vào Flash
 * Key format: "wtim/reg/<gpio_pin>"
 * Value: struct sensor_reg_entry (sensor_id + ch_idx_adc)
 *
 * ch_idx_adc = -1 nếu cảm biến digital (không dùng ADC)
 * --------------------------------------------------------------- */

/* Cấu trúc lưu trong Flash – KHÔNG được thay đổi thứ tự trường */
struct sensor_reg_entry {
    uint8_t sensor_id;
    int8_t  ch_idx;    /* -1 = không có ADC, 0/1/2 = kênh ADC */
};

int save_sensor_reg(int gpio_pin, uint8_t sensor_id)
{
    /* Tương thích ngược: gọi hàm đầy đủ với ch_idx = -1 */
    return save_sensor_reg_adc(gpio_pin, sensor_id, -1);
}

int save_sensor_reg_adc(int gpio_pin, uint8_t sensor_id, int ch_idx)
{
    char key[32];
    snprintf(key, sizeof(key), "wtim/reg/%d", gpio_pin);

    struct sensor_reg_entry entry = {
        .sensor_id = sensor_id,
        .ch_idx    = (int8_t)ch_idx,
    };

    int ret = settings_save_one(key, &entry, sizeof(entry));
    if (ret < 0) {
        LOG_ERR("Luu sensor reg GPIO %d that bai: %d", gpio_pin, ret);
    } else {
        LOG_INF("Da luu sensor reg: GPIO %d -> ID %d (ch_idx=%d)",
                gpio_pin, sensor_id, ch_idx);
    }
    return ret;
}

int delete_sensor_reg(int gpio_pin)
{
    char key[32];
    snprintf(key, sizeof(key), "wtim/reg/%d", gpio_pin);
    int ret = settings_delete(key);
    if (ret < 0) {
        LOG_ERR("Xoa sensor reg GPIO %d that bai: %d", gpio_pin, ret);
    }
    return ret;
}

/* ---------------------------------------------------------------
 * Cơ chế đọc TEDS – dùng Settings callback
 * --------------------------------------------------------------- */

/* Context truyền vào settings_load_subtree_direct */
struct teds_load_ctx {
    char    *buf;
    size_t   buf_size;
    size_t   loaded_len;
    bool     found;
};

/* Callback được gọi bởi settings_load_subtree_direct */
static int teds_direct_loader(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg,
                               void *param)
{
    ARG_UNUSED(name);
    struct teds_load_ctx *ctx = (struct teds_load_ctx *)param;

    if (len == 0 || len >= ctx->buf_size) {
        LOG_WRN("TEDS size %zu vượt buffer %zu", len, ctx->buf_size);
        return -ENOMEM;
    }

    ssize_t read_len = read_cb(cb_arg, ctx->buf, len);
    if (read_len < 0) {
        LOG_ERR("Đọc settings thất bại: %d", (int)read_len);
        return (int)read_len;
    }

    ctx->buf[read_len] = '\0';
    ctx->loaded_len    = (size_t)read_len;
    ctx->found         = true;

    LOG_INF("Đã đọc TEDS (%zu bytes)", ctx->loaded_len);
    return 0;
}

/**
 * @brief Đọc TEDS từ NVS vào buffer do caller cấp.
 *
 * @param sensor_id   ID cảm biến cần đọc.
 * @param buf         Buffer đầu ra (caller cấp).
 * @param buf_size    Kích thước buffer (bao gồm null terminator).
 * @return 0 nếu thành công, âm nếu lỗi, -ENOENT nếu không tìm thấy.
 */
int load_teds_from_nvs(uint8_t sensor_id, char *buf, size_t buf_size)
{
    if (!buf || buf_size == 0) {
        return -EINVAL;
    }

    char subtree[32];
    snprintf(subtree, sizeof(subtree), "wtim/teds/%d", sensor_id);

    struct teds_load_ctx ctx = {
        .buf      = buf,
        .buf_size = buf_size,
        .found    = false,
    };

    int ret = settings_load_subtree_direct(subtree, teds_direct_loader, &ctx);
    if (ret < 0) {
        LOG_ERR("settings_load_subtree_direct thất bại: %d", ret);
        return ret;
    }

    if (!ctx.found) {
        LOG_DBG("Không tìm thấy TEDS cho sensor %d", sensor_id);
        return -ENOENT;
    }

    return 0;
}

/* ---------------------------------------------------------------
 * Handler đăng ký với Settings subsystem (dùng settings_register)
 * --------------------------------------------------------------- */
static int teds_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    ARG_UNUSED(name);
    ARG_UNUSED(len);
    ARG_UNUSED(read_cb);
    ARG_UNUSED(cb_arg);
    /* Việc đọc được xử lý qua load_teds_from_nvs trực tiếp */
    return 0;
}

static struct settings_handler teds_settings_handler = {
    .name = "wtim",
    .h_set = teds_settings_set,
};

/* ---------------------------------------------------------------
 * Load toàn bộ sensor registry từ Flash khi khởi động
 * Duyệt key "wtim/reg/<gpio>" và gọi register_sensor()
 * --------------------------------------------------------------- */
#include "sensor_manager.h"

struct reg_load_ctx {
    int     gpio_pin;
    bool    found;
};

static int reg_loader_cb(const char *name, size_t len,
                          settings_read_cb read_cb, void *cb_arg,
                          void *param)
{
    struct reg_load_ctx *ctx = (struct reg_load_ctx *)param;

    ARG_UNUSED(name);

    struct sensor_reg_entry entry = { .sensor_id = 0, .ch_idx = -1 };

    if (len == sizeof(uint8_t)) {
        /* Dữ liệu cũ (1 byte) – tương thích ngược */
        ssize_t r = read_cb(cb_arg, &entry.sensor_id, sizeof(uint8_t));
        if (r < 0) return (int)r;
        entry.ch_idx = -1;
        LOG_WRN("Compat: doc sensor reg cu (1 byte) GPIO %d -> ID %d",
                ctx->gpio_pin, entry.sensor_id);
    } else if (len == sizeof(struct sensor_reg_entry)) {
        /* Dữ liệu mới (2 byte: sensor_id + ch_idx) */
        ssize_t r = read_cb(cb_arg, &entry, sizeof(entry));
        if (r < 0) return (int)r;
    } else {
        LOG_WRN("Sensor reg GPIO %d: size khong hop le (%zu)",
                ctx->gpio_pin, len);
        return -EINVAL;
    }

    register_sensor_adc(ctx->gpio_pin, entry.sensor_id, (int)entry.ch_idx);
    ctx->found = true;
    LOG_INF("Khoi phuc sensor reg: GPIO %d -> ID %d (ch_idx=%d)",
            ctx->gpio_pin, entry.sensor_id, (int)entry.ch_idx);
    return 0;
}

/*
 * Gọi hàm này sau settings_subsys_init() để nạp lại toàn bộ
 * sensor_registry[] từ Flash. Quét gpio_pin từ 0 đến 47.
 */
static void load_all_sensor_regs(void)
{
    LOG_INF("Dang khoi phuc danh sach cam bien tu Flash...");
    int restored = 0;
    for (int pin = 0; pin <= 47; pin++) {
        char subtree[32];
        snprintf(subtree, sizeof(subtree), "wtim/reg/%d", pin);
        struct reg_load_ctx ctx = { .gpio_pin = pin, .found = false };
        settings_load_subtree_direct(subtree, reg_loader_cb, &ctx);
        if (ctx.found) {
            restored++;
        }
    }
    LOG_INF("Da khoi phuc %d cam bien tu Flash", restored);
}

int storage_init(void)
{
    int ret = settings_subsys_init();
    if (ret < 0) {
        LOG_ERR("settings_subsys_init that bai: %d", ret);
        return ret;
    }

    ret = settings_register(&teds_settings_handler);
    if (ret < 0) {
        LOG_ERR("settings_register that bai: %d", ret);
        return ret;
    }

    /* Khôi phục danh sách cảm biến đã đăng ký từ lần trước */
    load_all_sensor_regs();

    LOG_INF("Storage khoi tao thanh cong");
    return 0;
}