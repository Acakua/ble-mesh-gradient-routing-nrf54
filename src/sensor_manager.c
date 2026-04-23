#include "sensor_manager.h"
#include "storage.h"
#include <cJSON.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_manager, LOG_LEVEL_INF);

/* ---------------------------------------------------------------
 * [Circuits of Conditioning Signal] – Hình 2.5 kiến trúc WTIM
 *
 * Đây là điểm DUY NHẤT trong toàn hệ thống được phép đọc ADC thô.
 * Tầng Communication (ble_handler.c) và tầng Function Blocks
 * (cluster_manager.c) phải gọi qua hàm này, KHÔNG được đọc ADC trực tiếp.
 *
 * Ánh xạ sensor_id → kênh ADC (theo Bảng 3.3 – bảng cứng):
 *   ID 1  (Rain-D,  GPIO 18) → ADC CH0  (AIN4, P1.11)
 *   ID 2  (MQ135D,  GPIO 19) → ADC CH1  (AIN5, P1.12)
 *   ID 13 (DHT22,   GPIO  5) → ADC CH2  (AIN2, P1.06)
 *
 * ⚠️  CẢNH BÁO: AIN0(P1.04) và AIN1(P1.05) trên nRF54L15DK mặc định
 *   bị UART console chiếm dụng → sẽ đọc ra giá trị ảo (loạn)!
 *   Luôn dùng AIN2 (P1.06) trở lên cho đo ADC thực tế.
 *
 * Cấu hình ADC:
 *   Gain    = 1/4  →  VFS = VREF × 4 = 0.6V × 4 = 2.4V
 *   VREF    = Internal 0.6V
 *   Res     = 12-bit  →  1 LSB ≈ 2.4V / 4096 ≈ 0.586 mV
 *   ⚠️ nRF54L15 KHÔNG hỗ trợ gain 1/6 → dùng 1/4 (VFS=2.4V)
 *
 * Với cảm biến đăng ký động (sensor add <gpio> <id>):
 *   Hàm sẽ tra bảng sensor_registry[] để tìm gpio_pin tương ứng,
 *   sau đó tìm ADC channel được gán cho gpio_pin đó.
 * --------------------------------------------------------------- */

/* ---------------------------------------------------------------
 * Cấu hình kênh ADC – PROGRAMMATIC (không dùng DT channel macros)
 *
 * Lý do: ADC_DT_SPEC_GET_BY_IDX qua node zephyr,user gây lỗi build
 *   "pasting ) and _P_ does not give a valid preprocessing token"
 *   trên NCS v3.0.1 khi channel nodes không resolve đúng trong DTS.
 *
 * Thay thế bằng struct adc_channel_cfg + adc_channel_setup() thuần túy.
 *
 * Ánh xạ vật lý:
 *   CH0  channel_id=0  AIN4 (P1.11) – Rain-D  (Gain 1/4, VFS=2.4V)
 *   CH1  channel_id=1  AIN5 (P1.12) – MQ135D  (Gain 1/4, VFS=2.4V)
 *   CH2  channel_id=2  AIN2 (P1.06) – DHT22   (Gain 1/4, VFS=2.4V)
 *   CH3  channel_id=3  AIN3 (P1.07) – Dự phòng cảm biến động
 *                                      (MQ2, LM35, v.v.)
 *
 * Gain 1/4 → VFS = VREF_INT(0.6V) × 4 = 2.4V, 1 LSB ≈ 0.586 mV
 * (nRF54L15 KHÔNG hỗ trợ gain 1/6)
 * --------------------------------------------------------------- */
static const struct adc_channel_cfg adc_ch_cfgs[] = {
    /* CH0 – Rain-D      → AIN4 (P1.11) */
    {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 0,
        .input_positive = NRF_SAADC_AIN4,
    },
    /* CH1 – MQ135D     → AIN5 (P1.12) */
    {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 1,
        .input_positive = NRF_SAADC_AIN5,
    },
    /* CH2 – DHT22      → AIN2 (P1.06) */
    {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 2,
        .input_positive = NRF_SAADC_AIN2,
    },
    /* CH3 – Dự phòng   → AIN3 (P1.07) – dành cho sensor động (MQ2...) */
    {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .channel_id = 3,
        .input_positive = NRF_SAADC_AIN3,
    },
};

#define ADC_NUM_CHANNELS ARRAY_SIZE(adc_ch_cfgs)

/* Device ADC + cờ khởi tạo + buffer sample */
static const struct device *adc_dev;
static bool adc_initialized = false;
static int16_t adc_sample_buf;

/* ---------------------------------------------------------------
 * Khởi tạo tất cả kênh ADC (gọi adc_channel_setup, không dùng DT)
 * Gọi lần đầu từ sensor_manager_init() hoặc lazy từ read_sensor_raw().
 * --------------------------------------------------------------- */
static int adc_init_all(void) {
  if (adc_initialized) {
    return 0;
  }

  adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
  if (!device_is_ready(adc_dev)) {
    LOG_ERR("ADC device chưa sẵn sàng");
    return -ENODEV;
  }

  for (int i = 0; i < (int)ADC_NUM_CHANNELS; i++) {
    int err = adc_channel_setup(adc_dev, &adc_ch_cfgs[i]);
    if (err < 0) {
      LOG_ERR("Không thể setup ADC channel %d (err=%d)", i, err);
      return err;
    }
    LOG_INF("ADC channel %d (ch_id=%d): khởi tạo OK", i,
            adc_ch_cfgs[i].channel_id);
  }

  adc_initialized = true;
  LOG_INF("ADC: tất cả %d kênh đã sẵn sàng", (int)ADC_NUM_CHANNELS);
  return 0;
}

/* Public wrapper – gọi từ main() trước khi BLE hoặc cluster bắt đầu */
int sensor_manager_init(void) { return adc_init_all(); }

/* ---------------------------------------------------------------
 * Đọc 1 kênh ADC thật theo chỉ số channel (0/1/2)
 * Trả về raw ADC (0–4095 với 12-bit), < 0 nếu lỗi.
 * --------------------------------------------------------------- */
static int adc_read_channel(int ch_idx) {
  if (ch_idx < 0 || ch_idx >= (int)ADC_NUM_CHANNELS) {
    LOG_ERR("adc_read_channel: ch_idx=%d ngoài phạm vi", ch_idx);
    return -EINVAL;
  }

  int err = adc_init_all();
  if (err < 0) {
    return err;
  }

  struct adc_sequence seq = {
      .channels = BIT(adc_ch_cfgs[ch_idx].channel_id),
      .buffer = &adc_sample_buf,
      .buffer_size = sizeof(adc_sample_buf),
      .resolution = 12,
  };

  err = adc_read(adc_dev, &seq);
  if (err < 0) {
    LOG_ERR("adc_read ch%d failed (%d)", ch_idx, err);
    return err;
  }

  int raw = (int)adc_sample_buf;
  LOG_DBG("ADC ch%d raw=%d", ch_idx, raw);
  return raw;
}

/* ---------------------------------------------------------------
 * Bảng cứng (built-in) – theo Bảng 3.3 trong tài liệu
 * Thêm trường ch_idx để ánh xạ sang kênh ADC DT.
 * --------------------------------------------------------------- */
static const struct {
  int gpio_pin;
  uint8_t sensor_id;
  int ch_idx; /* index trong adc_channels[] */
  const char *name;
} builtin_sensors[] = {
    /* Tạm tắt – Chưa có phần cứng kết nối vào các chân này trên DK.
     * Bật lại khi có cảm biến thật nối vào board.
     *
     * {18, 1, 0, "Rain-D"},   // GPIO18 → AIN4 (P1.11) → CH0
     * {19, 2, 1, "MQ135D"},   // GPIO19 → AIN5 (P1.12) → CH1
     * {5, 13, 2, "DHT22"},    // GPIO5  → AIN2 (P1.06) → CH2
     */
};

/* ---------------------------------------------------------------
 * Bảng đăng ký động – cho cảm biến chưa có trong bảng cứng
 * Thêm ch_idx để ánh xạ sang kênh ADC (nếu cảm biến động dùng
 * một trong các kênh DT đã khai báo).
 * --------------------------------------------------------------- */
struct sensor_entry {
  int gpio_pin;
  uint8_t sensor_id;
  int ch_idx; /* -1 = chưa gán kênh ADC */
  bool used;
};

static struct sensor_entry sensor_registry[SENSOR_REGISTRY_MAX];

/* Khai báo trước */
static int find_gpio_for_sensor_id(uint8_t sensor_id);
static int find_ch_idx_for_sensor_id(uint8_t sensor_id);

/* ---------------------------------------------------------------
 * read_sensor_raw – điểm đọc ADC DUY NHẤT của hệ thống
 * --------------------------------------------------------------- */
int read_sensor_raw(uint8_t sensor_id) {
  /* --- 1. Tìm trong bảng cứng -------------------------------- */
  for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
    if (builtin_sensors[i].sensor_id == sensor_id) {
      int raw = adc_read_channel(builtin_sensors[i].ch_idx);
      if (raw < 0) {
        LOG_ERR("read_sensor_raw: [builtin] ID=%d CH%d lỗi (%d)", sensor_id,
                builtin_sensors[i].ch_idx, raw);
      } else {
        LOG_DBG("read_sensor_raw: [builtin] %s ID=%d CH%d raw=%d",
                builtin_sensors[i].name, sensor_id, builtin_sensors[i].ch_idx,
                raw);
      }
      return raw;
    }
  }

  /* --- 2. Tìm trong bảng đăng ký động ----------------------- */
  int gpio_pin = find_gpio_for_sensor_id(sensor_id);
  if (gpio_pin < 0) {
    LOG_WRN("read_sensor_raw: sensor_id=%d chưa được đăng ký", sensor_id);
    return -ENODEV;
  }

  int ch_idx = find_ch_idx_for_sensor_id(sensor_id);
  if (ch_idx < 0) {
    /*
     * Cảm biến động chưa được gán kênh ADC DT.
     * Cần gọi register_sensor() với tham số ch_idx hợp lệ,
     * hoặc mở rộng overlay thêm channel.
     */
    LOG_WRN("read_sensor_raw: sensor_id=%d GPIO=%d chưa có kênh ADC", sensor_id,
            gpio_pin);
    return -ENOTSUP;
  }

  int raw = adc_read_channel(ch_idx);
  if (raw < 0) {
    LOG_ERR("read_sensor_raw: [dynamic] ID=%d GPIO=%d CH%d lỗi (%d)", sensor_id,
            gpio_pin, ch_idx, raw);
  } else {
    LOG_DBG("read_sensor_raw: [dynamic] ID=%d GPIO=%d CH%d raw=%d", sensor_id,
            gpio_pin, ch_idx, raw);
  }
  return raw;
}

/* ---------------------------------------------------------------
 * read_sensor_physical – chuyển ADC raw → đại lượng vật lý
 *
 * Luồng:
 *   raw (0–4095)
 *     → millivolt (Zephyr adc_raw_to_millivolts, GAIN_1_4, VREF_INT)
 *     → physical  = mV × cfg->scale + cfg->offset
 *
 * @param sensor_id  ID cảm biến
 * @param cfg        Tham số TEDS (từ apply_teds_config / load_teds_from_nvs)
 *                   Nếu NULL hoặc cfg->valid == false → trả về mV thay thế
 * @param out_value  Kết quả đại lượng vật lý
 * @return 0 nếu OK, <0 nếu lỗi đọc ADC
 * --------------------------------------------------------------- */
int read_sensor_physical(uint8_t sensor_id, const teds_config_t *cfg,
                         float *out_value) {
  if (!out_value) {
    return -EINVAL;
  }

  /* 1. Đọc ADC raw */
  int raw = read_sensor_raw(sensor_id);
  if (raw < 0) {
    LOG_ERR("read_sensor_physical: sensor_id=%d ADC lỗi (%d)", sensor_id, raw);
    return raw;
  }

  /* 2. Chuyển raw → millivolt
   *    Cấu hình hiện tại: GAIN_1/4, VREF_INTERNAL(600mV), 12-bit
   *    VFS = 600mV × 4 = 2400mV  →  1 LSB ≈ 0.586mV             */
  int32_t val_mv = (int32_t)raw;
  if (adc_initialized && adc_dev) {
    int err = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_4, 12,
                                    &val_mv);
    if (err < 0) {
      /* Fallback: tính thủ công nếu API lỗi */
      LOG_WRN("adc_raw_to_millivolts lỗi (%d), dùng tính thủ công", err);
      val_mv = (int32_t)((long)raw * 2400 / 4095);
    }
  } else {
    /* ADC chưa init – tính thủ công */
    val_mv = (int32_t)((long)raw * 2400 / 4095);
  }

  /* 3. Áp TEDS → physical value */
  if (cfg && cfg->valid) {
    if (cfg->use_2pt_cal) {
      /* --- Chế độ A: hiệu chỉnh 2 điểm trên raw ADC ---
       *   physical = (raw − raw1) / (raw2 − raw1) × (ref2 − ref1) + ref1  */
      if (cfg->cal_raw2 == cfg->cal_raw1) {
        LOG_ERR("sensor_id=%d: 2-pt cal lỗi cal_raw2==cal_raw1 (chia cho 0)",
                sensor_id);
        return -EINVAL;
      }
      *out_value = ((float)raw - cfg->cal_raw1)
                  / (cfg->cal_raw2 - cfg->cal_raw1)
                  * (cfg->cal_ref2 - cfg->cal_ref1)
                  + cfg->cal_ref1;
      LOG_INF("sensor_id=%d: raw=%d → %.3f %s  [2PT]",
              sensor_id, raw, (double)(*out_value), cfg->unit);
    } else {
      /* --- Chế độ B: tuyến tính trên mV ---
       *   physical = mV × scale + offset                                   */
      *out_value = (float)val_mv * cfg->scale + cfg->offset;
      LOG_INF("sensor_id=%d: raw=%d → %d mV → %.3f %s  [LIN]",
              sensor_id, raw, (int)val_mv, (double)(*out_value), cfg->unit);
    }

    /* Cảnh báo nếu ngoài dải hợp lệ */
    if (*out_value < cfg->range_min || *out_value > cfg->range_max) {
      LOG_WRN("sensor_id=%d: %.2f %s ngoài dải [%.1f, %.1f]", sensor_id,
              (double)(*out_value), cfg->unit,
              (double)cfg->range_min, (double)cfg->range_max);
    }
  } else {
    /* Không có TEDS → trả về mV */
    *out_value = (float)val_mv;
    LOG_DBG("sensor_id=%d: raw=%d → %d mV (không có TEDS)", sensor_id, raw,
            (int)val_mv);
  }

  return 0;
}

/* ---------------------------------------------------------------
 * Helpers nội bộ
 * --------------------------------------------------------------- */
static int find_gpio_for_sensor_id(uint8_t sensor_id) {
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (sensor_registry[i].used && sensor_registry[i].sensor_id == sensor_id) {
      return sensor_registry[i].gpio_pin;
    }
  }
  return -1;
}

static int find_ch_idx_for_sensor_id(uint8_t sensor_id) {
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (sensor_registry[i].used && sensor_registry[i].sensor_id == sensor_id) {
      return sensor_registry[i].ch_idx;
    }
  }
  return -1;
}

/* ---------------------------------------------------------------
 * API công khai
 * --------------------------------------------------------------- */

int detect_sensor_id(int gpio_pin) {
  /* 1. Tìm trong bảng cứng */
  for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
    if (builtin_sensors[i].gpio_pin == gpio_pin) {
      return builtin_sensors[i].sensor_id;
    }
  }

  /* 2. Tìm trong bảng đăng ký động */
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (sensor_registry[i].used && sensor_registry[i].gpio_pin == gpio_pin) {
      return sensor_registry[i].sensor_id;
    }
  }

  LOG_WRN("GPIO %d: cảm biến chưa được đăng ký", gpio_pin);
  return 0;
}

/*
 * register_sensor – đăng ký cảm biến động
 *
 * Tham số ch_idx_adc: chỉ số kênh ADC DT (0/1/2/...).
 * Nếu cảm biến không dùng ADC (cảm biến digital), truyền -1.
 *
 * Để tương thích ngược với prototype cũ (2 tham số), hàm wrapper
 * register_sensor_gpio() bên dưới sẽ truyền ch_idx=-1.
 */
int register_sensor_adc(int gpio_pin, uint8_t sensor_id, int ch_idx_adc) {
  /* Kiểm tra GPIO đã có trong bảng cứng chưa */
  for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
    if (builtin_sensors[i].gpio_pin == gpio_pin) {
      LOG_WRN("GPIO %d đã có trong bảng cứng (ID=%d)", gpio_pin,
              builtin_sensors[i].sensor_id);
      return SENSOR_ERR_EXISTS;
    }
  }

  /* Kiểm tra sensor_id đã được dùng trong bảng cứng chưa */
  for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
    if (builtin_sensors[i].sensor_id == sensor_id) {
      LOG_WRN("sensor_id %d đã được dùng bởi GPIO %d (bảng cứng)", sensor_id,
              builtin_sensors[i].gpio_pin);
      return SENSOR_ERR_EXISTS;
    }
  }

  /* Kiểm tra GPIO hoặc sensor_id đã đăng ký động chưa */
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (!sensor_registry[i].used) {
      continue;
    }
    if (sensor_registry[i].gpio_pin == gpio_pin) {
      LOG_WRN("GPIO %d đã được đăng ký với ID=%d", gpio_pin,
              sensor_registry[i].sensor_id);
      return SENSOR_ERR_EXISTS;
    }
    if (sensor_registry[i].sensor_id == sensor_id) {
      LOG_WRN("sensor_id %d đã được dùng bởi GPIO %d", sensor_id,
              sensor_registry[i].gpio_pin);
      return SENSOR_ERR_EXISTS;
    }
  }

  /* Tìm slot trống để đăng ký */
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (!sensor_registry[i].used) {
      sensor_registry[i].gpio_pin = gpio_pin;
      sensor_registry[i].sensor_id = sensor_id;
      sensor_registry[i].ch_idx = ch_idx_adc;
      sensor_registry[i].used = true;
      LOG_INF("Đã đăng ký cảm biến: GPIO %d → ID %d (ADC CH%d)", gpio_pin,
              sensor_id, ch_idx_adc);
      return SENSOR_OK;
    }
  }

  LOG_ERR("Bảng đăng ký cảm biến đã đầy (%d mục)", SENSOR_REGISTRY_MAX);
  return SENSOR_ERR_FULL;
}

/* Wrapper tương thích ngược – shell dùng 2 tham số, không biết ch_idx */
int register_sensor(int gpio_pin, uint8_t sensor_id) {
  return register_sensor_adc(gpio_pin, sensor_id, -1);
}

int unregister_sensor(int gpio_pin) {
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (sensor_registry[i].used && sensor_registry[i].gpio_pin == gpio_pin) {
      sensor_registry[i].used = false;
      LOG_INF("Đã hủy đăng ký cảm biến GPIO %d", gpio_pin);
      return SENSOR_OK;
    }
  }

  LOG_WRN("GPIO %d: không tìm thấy để hủy đăng ký", gpio_pin);
  return SENSOR_ERR_NOTFOUND;
}

void list_registered_sensors(void) {
  LOG_INF("=== Danh sách cảm biến (bảng cứng) ===");
  for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
    LOG_INF("  GPIO %-3d → ID %-3d  ADC CH%d  (%s)",
            builtin_sensors[i].gpio_pin, builtin_sensors[i].sensor_id,
            builtin_sensors[i].ch_idx, builtin_sensors[i].name);
  }

  LOG_INF("=== Danh sách cảm biến (đăng ký động) ===");
  bool any = false;
  for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
    if (sensor_registry[i].used) {
      LOG_INF("  GPIO %-3d → ID %-3d  ADC CH%d", sensor_registry[i].gpio_pin,
              sensor_registry[i].sensor_id, sensor_registry[i].ch_idx);
      any = true;
    }
  }
  if (!any) {
    LOG_INF("  (chưa có cảm biến nào được đăng ký động)");
  }
}

/* ---------------------------------------------------------------
 * Tính giá trị hiệu chỉnh từ ADC raw + TEDS JSON
 * --------------------------------------------------------------- */
float calculate_calibrated_value(int raw_adc, const char *teds_json) {
  cJSON *teds = cJSON_Parse(teds_json);
  if (!teds) {
    LOG_WRN("TEDS JSON không hợp lệ, trả raw ADC");
    return (float)raw_adc;
  }

  /* Đọc từng trường với kiểm tra null pointer */
  cJSON *item_raw1 = cJSON_GetObjectItem(teds, "cal_raw1");
  cJSON *item_ref1 = cJSON_GetObjectItem(teds, "cal_ref1");
  cJSON *item_raw2 = cJSON_GetObjectItem(teds, "cal_raw2");
  cJSON *item_ref2 = cJSON_GetObjectItem(teds, "cal_ref2");

  if (!item_raw1 || !item_ref1 || !item_raw2 || !item_ref2) {
    LOG_ERR("TEDS thiếu trường hiệu chỉnh (cal_raw1/ref1/raw2/ref2)");
    cJSON_Delete(teds);
    return (float)raw_adc;
  }

  float cal_raw1 = (float)item_raw1->valuedouble;
  float cal_ref1 = (float)item_ref1->valuedouble;
  float cal_raw2 = (float)item_raw2->valuedouble;
  float cal_ref2 = (float)item_ref2->valuedouble;

  /* Tránh chia cho 0 */
  if (cal_raw2 == cal_raw1) {
    LOG_ERR("TEDS lỗi: cal_raw2 == cal_raw1 (chia cho 0)");
    cJSON_Delete(teds);
    return (float)raw_adc;
  }

  float result =
      (raw_adc - cal_raw1) / (cal_raw2 - cal_raw1) * (cal_ref2 - cal_ref1) +
      cal_ref1;

  cJSON_Delete(teds);
  return result;
}

/* ---------------------------------------------------------------
 * Áp dụng cấu hình TEDS vào struct teds_config_t
 * Theo Hình 3.6: "Cấu hình cảm biến theo TEDS"
 * --------------------------------------------------------------- */
int apply_teds_config(const char *teds_json, teds_config_t *out) {
  if (!teds_json || !out) {
    return -EINVAL;
  }

  /* Khởi tạo giá trị mặc định */
  memset(out, 0, sizeof(*out));
  out->scale = 1.0f;
  out->offset = 0.0f;
  out->range_min = -9999.0f;
  out->range_max = 9999.0f;
  out->valid = false;

  cJSON *teds = cJSON_Parse(teds_json);
  if (!teds) {
    LOG_ERR("apply_teds_config: JSON không hợp lệ");
    return -EINVAL;
  }

  /* Đọc type (bắt buộc) */
  cJSON *jtype = cJSON_GetObjectItem(teds, "type");
  if (cJSON_IsString(jtype) && jtype->valuestring) {
    strncpy(out->type, jtype->valuestring, TEDS_TYPE_MAX_LEN - 1);
  } else {
    LOG_WRN("TEDS thiếu trường 'type' – dùng 'unknown'");
    strncpy(out->type, "unknown", TEDS_TYPE_MAX_LEN - 1);
  }

  /* Đọc unit (tuỳ chọn) */
  cJSON *junit = cJSON_GetObjectItem(teds, "unit");
  if (cJSON_IsString(junit) && junit->valuestring) {
    strncpy(out->unit, junit->valuestring, TEDS_UNIT_MAX_LEN - 1);
  }

  /* Đọc scale (tuỳ chọn, mặc định 1.0) */
  cJSON *jscale = cJSON_GetObjectItem(teds, "scale");
  if (cJSON_IsNumber(jscale)) {
    out->scale = (float)jscale->valuedouble;
  }

  /* Đọc offset (tuỳ chọn, mặc định 0.0) */
  cJSON *joffset = cJSON_GetObjectItem(teds, "offset");
  if (cJSON_IsNumber(joffset)) {
    out->offset = (float)joffset->valuedouble;
  }

  /* Đọc range_min / range_max (tuỳ chọn) */
  cJSON *jrmin = cJSON_GetObjectItem(teds, "range_min");
  if (cJSON_IsNumber(jrmin)) {
    out->range_min = (float)jrmin->valuedouble;
  }
  cJSON *jrmax = cJSON_GetObjectItem(teds, "range_max");
  if (cJSON_IsNumber(jrmax)) {
    out->range_max = (float)jrmax->valuedouble;
  }

  /* Phát hiện chế độ hiệu chỉnh:
   * Nếu JSON có cal_raw1/cal_ref1/cal_raw2/cal_ref2 → chế độ 2 điểm (Phương pháp A)
   * Ngược lại → chế độ tuyến tính scale/offset (Phương pháp B) */
  cJSON *jcr1 = cJSON_GetObjectItem(teds, "cal_raw1");
  cJSON *jcf1 = cJSON_GetObjectItem(teds, "cal_ref1");
  cJSON *jcr2 = cJSON_GetObjectItem(teds, "cal_raw2");
  cJSON *jcf2 = cJSON_GetObjectItem(teds, "cal_ref2");

  if (cJSON_IsNumber(jcr1) && cJSON_IsNumber(jcf1) &&
      cJSON_IsNumber(jcr2) && cJSON_IsNumber(jcf2)) {
    out->use_2pt_cal = true;
    out->cal_raw1    = (float)jcr1->valuedouble;
    out->cal_ref1    = (float)jcf1->valuedouble;
    out->cal_raw2    = (float)jcr2->valuedouble;
    out->cal_ref2    = (float)jcf2->valuedouble;
    LOG_INF("TEDS: chế độ 2 điểm  raw1=%.0f→%.4f  raw2=%.0f→%.4f %s",
            (double)out->cal_raw1, (double)out->cal_ref1,
            (double)out->cal_raw2, (double)out->cal_ref2, out->unit);
  } else {
    out->use_2pt_cal = false;
  }

  out->valid = true;
  cJSON_Delete(teds);

  LOG_INF(
      "TEDS config: type=%s unit=%s scale=%.3f offset=%.3f range=[%.1f, %.1f]",
      out->type, out->unit, (double)out->scale, (double)out->offset,
      (double)out->range_min, (double)out->range_max);
  return 0;
}

/* ---------------------------------------------------------------
 * build_sensor_packet
 *
 * Đọc toàn bộ sensor (builtin + đăng ký động), điền sensor_packet_t.
 * Với mỗi sensor:
 *   • Đọc raw ADC
 *   • Nếu Flash có TEDS → tính physical và điền has_physical + unit
 *   • Nếu không có TEDS → has_physical = false, physical = 0
 * --------------------------------------------------------------- */
int build_sensor_packet(sensor_packet_t *pkt)
{
    if (!pkt) {
        return -EINVAL;
    }

    memset(pkt, 0, sizeof(*pkt));
    pkt->timestamp_ms = k_uptime_get();

    /* Buffer TEDS dùng chung trong hàm này */
    static char _teds_buf[256];

    /* Macro nội bộ để điền 1 entry */
    #define _FILL(sid_val, label) do {                                    \
        if (pkt->count >= SENSOR_PACKET_MAX_ENTRIES) {                    \
            LOG_WRN("build_sensor_packet: packet đầy");                   \
            break;                                                         \
        }                                                                  \
        uint8_t _s = (sid_val);                                           \
        int _r = read_sensor_raw(_s);                                     \
        if (_r < 0) {                                                      \
            LOG_WRN("  %s ID=%d: đọc thất bại (%d)", label, _s, _r);     \
            pkt->errors++;                                                 \
        } else {                                                           \
            sensor_entry_out_t *_e = &pkt->entries[pkt->count];           \
            _e->sensor_id    = _s;                                         \
            _e->raw          = _r;                                         \
            _e->physical     = 0.0f;                                       \
            _e->has_physical = false;                                      \
            _e->unit[0]      = '\0';                                       \
            _e->timestamp_ms = k_uptime_get();                             \
            if (load_teds_from_nvs(_s, _teds_buf, sizeof(_teds_buf)) == 0) { \
                teds_config_t _cfg;                                        \
                if (apply_teds_config(_teds_buf, &_cfg) == 0 && _cfg.valid) { \
                    float _phys = 0.0f;                                    \
                    if (read_sensor_physical(_s, &_cfg, &_phys) == 0) {   \
                        _e->physical     = _phys;                          \
                        _e->has_physical = true;                           \
                        strncpy(_e->unit, _cfg.unit, sizeof(_e->unit)-1); \
                        _e->unit[sizeof(_e->unit)-1] = '\0';               \
                        LOG_INF("  %s ID=%d raw=%d -> %.3f %s",           \
                                label, _s, _r, (double)_phys, _e->unit);  \
                    }                                                       \
                }                                                          \
            } else {                                                       \
                LOG_DBG("  %s ID=%d raw=%d (chưa có TEDS)", label, _s, _r); \
            }                                                              \
            pkt->count++;                                                  \
        }                                                                  \
    } while (0)

    /* --- 1. Bảng cứng ----------------------------------------- */
    for (int i = 0; i < (int)ARRAY_SIZE(builtin_sensors); i++) {
        _FILL(builtin_sensors[i].sensor_id, builtin_sensors[i].name);
    }

    /* --- 2. Bảng đăng ký động ---------------------------------- */
    for (int i = 0; i < SENSOR_REGISTRY_MAX; i++) {
        if (!sensor_registry[i].used) {
            continue;
        }
        _FILL(sensor_registry[i].sensor_id, "dynamic");
    }

    #undef _FILL

    LOG_INF("build_sensor_packet: %d sensor OK, %d lỗi",
            pkt->count, pkt->errors);
    return pkt->count;
}

/* ---------------------------------------------------------------
 * sensor_packet_to_json – serialize packet thành JSON
 *
 * Format được nâng cấp:
 *   {"n":<count>,"sensors":[{"id":<id>,"raw":<raw>,"val":<phys>,"unit":"<unit>"}]}
 *
 * Nếu sensor không có TEDS, bỏ val và unit khỏi JSON entry.
 * --------------------------------------------------------------- */
int sensor_packet_to_json(const sensor_packet_t *pkt, char *buf, size_t size)
{
    if (!pkt || !buf || size == 0) {
        return -EINVAL;
    }

    int off = snprintf(buf, size, "{\"ts\":%lld,\"n\":%d,\"sensors\":[",
                       (long long)pkt->timestamp_ms, pkt->count);
    if (off < 0 || (size_t)off >= size) {
        return -ENOMEM;
    }

    for (int i = 0; i < pkt->count; i++) {
        const sensor_entry_out_t *e = &pkt->entries[i];
        int written;

        if (e->has_physical) {
            /* Có TEDS → xuất cả val + unit + ts */
            written = snprintf(buf + off, size - (size_t)off,
                               "%s{\"id\":%d,\"raw\":%d,\"val\":%.3f,\"unit\":\"%s\",\"ts\":%lld}",
                               (i > 0) ? "," : "",
                               e->sensor_id, e->raw,
                               (double)e->physical, e->unit,
                               (long long)e->timestamp_ms);
        } else {
            /* Không có TEDS → raw + ts */
            written = snprintf(buf + off, size - (size_t)off,
                               "%s{\"id\":%d,\"raw\":%d,\"ts\":%lld}",
                               (i > 0) ? "," : "",
                               e->sensor_id, e->raw,
                               (long long)e->timestamp_ms);
        }

        if (written < 0 || (size_t)(off + written) >= size) {
            LOG_WRN("sensor_packet_to_json: buffer quá nhỏ, dừng ở mục %d", i);
            break;
        }
        off += written;
    }

    /* Đóng JSON */
    if ((size_t)off + 2 < size) {
        buf[off++] = ']';
        buf[off++] = '}';
        buf[off]   = '\0';
    } else {
        buf[size - 1] = '\0';
        return -ENOMEM;
    }

    return off;
}
