/*
 * teds_tlv.h
 *
 * Binary TLV TEDS format – IEEE 1451.0-2007 TransducerChannel TEDS.
 *
 * Frame layout (stored in NVS + transmitted over BLE):
 *
 *   ┌──────────┬─────────┬──────────────┬─────────────────┬──────────────┐
 *   │ CLASS 1B │ VER 1B  │ BODY_LEN 2B  │  TLV body ...   │  CRC16 2B    │
 *   │          │         │   (LE)       │  BODY_LEN bytes │  (LE)        │
 *   └──────────┴─────────┴──────────────┴─────────────────┴──────────────┘
 *   ↑______________________ CRC16-CCITT covers these bytes ____________↑
 *
 *   Total frame size = 4 (header) + BODY_LEN + 2 (CRC) bytes.
 *
 * TLV field: [TID 1B][LEN 1B][VALUE LEN bytes]
 * End marker: TID = 0x00 (no LEN/VALUE bytes follow).
 *
 * Detection: JSON starts with '{' (0x7B); TEDS_CLASS values are 0x01-0x04,
 * so teds_is_binary() trivially distinguishes the two formats.
 */

#ifndef TEDS_TLV_H
#define TEDS_TLV_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sensor_manager.h"   /* teds_config_t */

/* ---------------------------------------------------------------
 * TEDS Classes (IEEE 1451.0 Table 28 subset)
 * --------------------------------------------------------------- */
#define TEDS_CLASS_META       0x01U  /* Meta-TEDS                    */
#define TEDS_CLASS_XDCR_CHAN  0x02U  /* TransducerChannel TEDS       */
#define TEDS_CLASS_CALIB      0x03U  /* Calibration TEDS             */
#define TEDS_CLASS_USER       0x04U  /* User's TEDS                  */

#define TEDS_VERSION          0x01U  /* Format version               */

/* Header: CLASS(1) + VER(1) + BODY_LEN(2) = 4 bytes.
 * CRC(2) is appended AFTER the TLV body, not in the header struct. */
#define TEDS_HDR_SIZE         4U

/* Maximum binary TEDS size (header + body + CRC). */
#define TEDS_MAX_BIN_SIZE     128U

/* ---------------------------------------------------------------
 * TIDs for TEDS_CLASS_XDCR_CHAN
 * --------------------------------------------------------------- */
#define TEDS_TID_END          0x00U  /* End of TLV body              */
#define TEDS_TID_SENSOR_TYPE  0x01U  /* 1B: 0=sensor, 1=actuator    */
#define TEDS_TID_NAME         0x02U  /* var: sensor type string      */
#define TEDS_TID_UNIT         0x03U  /* var: unit string             */
#define TEDS_TID_LOWER_RANGE  0x04U  /* 4B: float32 lower range      */
#define TEDS_TID_UPPER_RANGE  0x05U  /* 4B: float32 upper range      */
#define TEDS_TID_CAL_MODE     0x06U  /* 1B: TEDS_CAL_LINEAR or 2PT  */
#define TEDS_TID_SCALE        0x07U  /* 4B: float32 linear scale     */
#define TEDS_TID_OFFSET       0x08U  /* 4B: float32 linear offset    */
#define TEDS_TID_CAL_RAW1     0x09U  /* 4B: float32 2-pt raw ADC 1   */
#define TEDS_TID_CAL_REF1     0x0AU  /* 4B: float32 2-pt physical 1  */
#define TEDS_TID_CAL_RAW2     0x0BU  /* 4B: float32 2-pt raw ADC 2   */
#define TEDS_TID_CAL_REF2     0x0CU  /* 4B: float32 2-pt physical 2  */

/* Calibration mode values (byte value of TEDS_TID_CAL_MODE field) */
#define TEDS_CAL_LINEAR       0x00U  /* physical = mV * scale + offset     */
#define TEDS_CAL_2PT          0x01U  /* physical = interp(raw, 2-pt table) */

/* ---------------------------------------------------------------
 * Header struct (packed, little-endian)
 * --------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint8_t  teds_class;  /* TEDS_CLASS_*                              */
    uint8_t  version;     /* TEDS_VERSION                              */
    uint16_t body_len;    /* Length of TLV body (NOT including hdr/CRC)*/
} teds_hdr_t;

/* ---------------------------------------------------------------
 * API
 * --------------------------------------------------------------- */

/**
 * @brief Parse binary TLV TEDS into teds_config_t.
 *
 * @param buf  Binary TEDS frame (header + body + CRC).
 * @param len  Total length of frame in bytes.
 * @param out  Output config struct.
 * @return 0 on success.
 *         -EINVAL  bad format or unsupported class/version.
 *         -EBADMSG CRC mismatch.
 */
int teds_tlv_parse(const uint8_t *buf, size_t len, teds_config_t *out);

/**
 * @brief Serialize teds_config_t into binary TLV TEDS.
 *
 * @param cfg       Input config (must have valid=true).
 * @param buf       Output buffer (needs >= TEDS_MAX_BIN_SIZE bytes).
 * @param buf_size  Size of output buffer.
 * @return Bytes written (header + body + CRC), negative on error.
 */
int teds_tlv_build(const teds_config_t *cfg, uint8_t *buf, size_t buf_size);

/**
 * @brief Return true if the buffer contains binary TEDS (not JSON).
 *        JSON always starts with '{' (0x7B); TEDS class bytes are 0x01-0x04.
 */
static inline bool teds_is_binary(const uint8_t *buf, size_t len)
{
    return (len >= TEDS_HDR_SIZE && buf[0] != (uint8_t)'{');
}

/** @brief CRC16-CCITT (poly=0x1021, init=0xFFFF). */
uint16_t teds_crc16(const uint8_t *data, size_t len);

#endif /* TEDS_TLV_H */