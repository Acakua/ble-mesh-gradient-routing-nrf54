#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Khởi tạo storage subsystem (gọi 1 lần trong main).
 */
int storage_init(void);

/**
 * @brief Lưu TEDS JSON vào Flash theo sensor_id.
 */
int save_teds_to_nvs(uint8_t sensor_id, const char *json_str);

/**
 * @brief Đọc TEDS JSON từ Flash vào buffer.
 * @return 0 nếu thành công, -ENOENT nếu không tìm thấy.
 */
int load_teds_from_nvs(uint8_t sensor_id, char *buf, size_t buf_size);

/**
 * @brief Xoa TEDS cua mot sensor khoi Flash.
 */
int delete_teds_from_nvs(uint8_t sensor_id);

/**
 * @brief Luu dang ky sensor (gpio_pin -> sensor_id) vao Flash.
 *        Duoc goi boi sensor_shell khi chay lenh 'sensor add'.
 *        Tuong thich nguoc: ch_idx = -1.
 */
int save_sensor_reg(int gpio_pin, uint8_t sensor_id);

/**
 * @brief Luu dang ky sensor kem kenh ADC (ch_idx) vao Flash.
 *        Dung khi sensor dong co ADC channel cu the.
 *
 * @param gpio_pin   Chan GPIO cua cam bien.
 * @param sensor_id  ID cam bien.
 * @param ch_idx     Index kenh ADC (0/1/2), hoac -1 neu cam bien so.
 */
int save_sensor_reg_adc(int gpio_pin, uint8_t sensor_id, int ch_idx);

/**
 * @brief Xoa dang ky sensor khoi Flash.
 *        Duoc goi boi sensor_shell khi chay lenh 'sensor del'.
 */
int delete_sensor_reg(int gpio_pin);

#endif /* STORAGE_H */
