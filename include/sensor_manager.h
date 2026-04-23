#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Số cảm biến tối đa có thể đăng ký động */
#define SENSOR_REGISTRY_MAX 16

/* Trạng thái trả về */
#define SENSOR_OK            0
#define SENSOR_ERR_FULL     -1
#define SENSOR_ERR_EXISTS   -2
#define SENSOR_ERR_NOTFOUND -3

/* ---------------------------------------------------------------
 * Packet gom nhiều sensor – dùng để gửi 1 Notify BLE duy nhất
 * --------------------------------------------------------------- */
#define SENSOR_PACKET_MAX_ENTRIES  16

typedef struct {
    uint8_t sensor_id;           /* ID cảm biến */
    int     raw;                 /* Giá trị ADC thô */
    float   physical;            /* Đại lượng vật lý (0.0 nếu chưa có TEDS) */
    char    unit[8];             /* Đơn vị đo ("C", "%", "ppm"...) */
    bool    has_physical;        /* true nếu đã chuyển được ra đơn vị vật lý */
    int64_t timestamp_ms;        /* Thời điểm đọc mẫu (ms kể từ boot, k_uptime_get) */
} sensor_entry_out_t;

typedef struct {
    sensor_entry_out_t entries[SENSOR_PACKET_MAX_ENTRIES];
    uint8_t            count;       /* Số sensor thực sự có dữ liệu */
    uint8_t            errors;      /* Số sensor đọc thất bại */
    int64_t            timestamp_ms;/* Thời điểm tạo packet (ms kể từ boot) */
} sensor_packet_t;

/**
 * @brief Đọc tất cả sensor (builtin + đăng ký động) và đóng gói thành
 *        1 sensor_packet_t.  Sensor nào read_sensor_raw() trả lỗi thì
 *        bỏ qua (tăng packet->errors).
 *
 * @param pkt  Con trỏ packet để ghi kết quả.
 * @return Số sensor đọc thành công (>= 0).
 */
int build_sensor_packet(sensor_packet_t *pkt);

/**
 * @brief Đóng gói sensor_packet_t thành chuỗi JSON vào buffer.
 *        Format: {"ts":<ts_ms>,"n":<count>,"sensors":[{"id":<id>,"raw":<raw>,"ts":<ts_ms>,...}]}
 *
 * @param pkt   Packet đầu vào.
 * @param buf   Buffer đầu ra.
 * @param size  Kích thước buffer.
 * @return Số ký tự ghi (không kể null-terminator), <0 nếu lỗi.
 */
int sensor_packet_to_json(const sensor_packet_t *pkt, char *buf, size_t size);

/* ---------------------------------------------------------------
 * Cấu trúc lưu tham số TEDS đã parse (theo IEEE 1451.4)
 *
 * Hỗ trợ 2 chế độ hiệu chỉnh:
 *
 * [A] 2 điểm thực tế (use_2pt_cal = true):
 *     physical = (raw - cal_raw1) / (cal_raw2 - cal_raw1)
 *               * (cal_ref2 - cal_ref1) + cal_ref1
 *     → Nhập trực tiếp từ datasheet / đo thực nghiệm.
 *     → Ví dụ Rain: (raw_khô=200 → 0%) và (raw_ngập=3800 → 100%)
 *
 * [B] Tuyến tính (use_2pt_cal = false):
 *     mV      = raw * 2400 / 4095   (VFS=2400mV, 12-bit)
 *     physical = mV * scale + offset
 *     → Dùng khi datasheet cho biết đặc tính V-out tuyến tính.
 * --------------------------------------------------------------- */
#define TEDS_TYPE_MAX_LEN  32
#define TEDS_UNIT_MAX_LEN  16

typedef struct {
    char  type[TEDS_TYPE_MAX_LEN]; /* Loại cảm biến: "temperature", "humidity"... */
    char  unit[TEDS_UNIT_MAX_LEN]; /* Đơn vị đo: "C", "%RH", "ppm"... */

    /* Chế độ [B] – tuyến tính (scale/offset trên mV) */
    float scale;                   /* Hệ số tỉ lệ  (mặc định 1.0) */
    float offset;                  /* Độ lệch       (mặc định 0.0) */

    /* Chế độ [A] – 2 điểm hiệu chỉnh (trên raw ADC) */
    bool  use_2pt_cal;             /* true → dùng cal_raw/ref thay vì scale/offset */
    float cal_raw1;                /* Điểm 1: raw ADC (thường = giá trị min) */
    float cal_ref1;                /* Điểm 1: đại lượng vật lý tương ứng */
    float cal_raw2;                /* Điểm 2: raw ADC (thường = giá trị max) */
    float cal_ref2;                /* Điểm 2: đại lượng vật lý tương ứng */

    float range_min;               /* Giá trị min hợp lệ */
    float range_max;               /* Giá trị max hợp lệ */
    bool  valid;                   /* true nếu đã parse thành công */
} teds_config_t;

/**
 * @brief Phát hiện sensor_id từ chân GPIO.
 *        Tra cứu bảng cứng trước, sau đó tra bảng đăng ký động.
 *
 * @param gpio_pin  Số chân GPIO.
 * @return sensor_id > 0 nếu tìm thấy, 0 nếu chưa đăng ký.
 */
/**
 * @brief [Circuits of Conditioning Signal]
 *        Đọc giá trị ADC thô từ cảm biến theo sensor_id.
 *        Đây là điểm giao tiếp duy nhất giữa tầng Function Block
 *        và tầng Signal Conditioning trong kiến trúc WTIM.
 *
 * @param sensor_id  ID cảm biến cần đọc.
 * @return Giá trị ADC thô (0-4095 với ADC 12-bit), -ENODEV nếu không hỗ trợ.
 */
int read_sensor_raw(uint8_t sensor_id);

/**
 * @brief Khởi tạo toàn bộ kênh ADC từ Device Tree.
 *        Có thể gọi từ main() để init sớm, hoặc sẽ tự init
 *        lần đầu khi read_sensor_raw() được gọi.
 *
 * @return 0 nếu OK, <0 nếu lỗi (ENODEV khi hardware chưa ready).
 */
int sensor_manager_init(void);

/**
 * @brief Phát hiọn sensor_id từ chân GPIO.
 */
int detect_sensor_id(int gpio_pin);

/**
 * @brief Đăng ký một cảm biến chưa có trong bảng.
 *
 * @param gpio_pin  Chân GPIO của cảm biến.
 * @param sensor_id ID muốn gán (> 0).
 * @return SENSOR_OK nếu thành công,
 *         SENSOR_ERR_FULL nếu bảng đầy,
 *         SENSOR_ERR_EXISTS nếu gpio_pin đã được đăng ký.
 */
int register_sensor(int gpio_pin, uint8_t sensor_id);

/**
 * @brief Đăng ký cảm biến động với kênh ADC cụ thể.
 *        Dùng khi cảm biến đã được khai báo trong device tree overlay
 *        (io-channels) và biết index kênh.
 *
 * @param gpio_pin    Chân GPIO của cảm biến.
 * @param sensor_id   ID muốn gán (> 0).
 * @param ch_idx_adc  Index kênh trong adc_channels[] (0/1/2/...).
 *                    Truyền -1 nếu cảm biến không dùng ADC (digital).
 * @return SENSOR_OK, SENSOR_ERR_FULL, SENSOR_ERR_EXISTS.
 */
int register_sensor_adc(int gpio_pin, uint8_t sensor_id, int ch_idx_adc);

/**
 * @brief Hủy đăng ký một cảm biến theo chân GPIO.
 *
 * @param gpio_pin  Chân GPIO cần xóa.
 * @return SENSOR_OK nếu thành công,
 *         SENSOR_ERR_NOTFOUND nếu không tìm thấy.
 */
int unregister_sensor(int gpio_pin);

/**
 * @brief In danh sách toàn bộ cảm biến đã đăng ký động ra LOG.
 */
void list_registered_sensors(void);

/**
 * @brief Tính giá trị đã hiệu chỉnh từ ADC raw và chuỗi JSON TEDS.
 */
float calculate_calibrated_value(int raw_adc, const char *teds_json);

/**
 * @brief Parse TEDS JSON và điền vào struct teds_config_t.
 *        Áp dụng cấu hình cảm biến theo TEDS (Hình 3.6).
 *
 * @param teds_json  Chuỗi JSON TEDS từ NVS.
 * @param out        Con trỏ struct để lưu kết quả parse.
 * @return 0 nếu thành công, <0 nếu JSON lỗi.
 */
int apply_teds_config(const char *teds_json, teds_config_t *out);

/**
 * @brief Đọc ADC và chuyển thẳng ra đại lượng vật lý theo TEDS config.
 *
 * Luồng:
 *   raw (0–4095) → millivolt → physical = mV × cfg->scale + cfg->offset
 *
 * @param sensor_id  ID cảm biến cần đọc.
 * @param cfg        Tham số TEDS đã parse (từ apply_teds_config).
 *                   Nếu NULL hoặc cfg->valid == false → trả về mV.
 * @param out_value  Con trỏ nhận kết quả đại lượng vật lý.
 * @return 0 nếu OK, <0 nếu lỗi đọc ADC.
 */
int read_sensor_physical(uint8_t sensor_id,
                         const teds_config_t *cfg,
                         float *out_value);

#endif /* SENSOR_MANAGER_H */
