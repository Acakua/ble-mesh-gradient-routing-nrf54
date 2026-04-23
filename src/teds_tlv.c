/*
 * teds_tlv.c
 *
 * Binary TLV TEDS encode/decode – IEEE 1451.0-2007 inspired.
 * See include/teds_tlv.h for frame format documentation.
 */

#include "teds_tlv.h"
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(teds_tlv, LOG_LEVEL_INF);

/* ---------------------------------------------------------------
 * CRC16-CCITT (poly=0x1021, init=0xFFFF)
 * --------------------------------------------------------------- */
uint16_t teds_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---------------------------------------------------------------
 * Internal TLV write helpers – return bytes written, 0 if no space
 * --------------------------------------------------------------- */
static size_t tlv_write_u8(uint8_t *buf, size_t avail, uint8_t tid, uint8_t val)
{
    if (avail < 3U) {
        return 0U;
    }
    buf[0] = tid;
    buf[1] = 1U;
    buf[2] = val;
    return 3U;
}

static size_t tlv_write_f32(uint8_t *buf, size_t avail, uint8_t tid, float val)
{
    if (avail < 6U) {
        return 0U;
    }
    buf[0] = tid;
    buf[1] = 4U;
    memcpy(&buf[2], &val, sizeof(float));
    return 6U;
}

static size_t tlv_write_str(uint8_t *buf, size_t avail,
                             uint8_t tid, const char *str, uint8_t max_len)
{
    uint8_t slen = (uint8_t)strnlen(str, max_len);

    if (avail < (size_t)(2U + slen)) {
        return 0U;
    }
    buf[0] = tid;
    buf[1] = slen;
    memcpy(&buf[2], str, slen);
    return (size_t)(2U + slen);
}

/* ---------------------------------------------------------------
 * teds_tlv_build
 * --------------------------------------------------------------- */
int teds_tlv_build(const teds_config_t *cfg, uint8_t *buf, size_t buf_size)
{
    if (!cfg || !buf || buf_size < (TEDS_HDR_SIZE + 3U)) {
        return -EINVAL;
    }

    /* Write TLV body starting after the 4-byte header */
    uint8_t *body       = buf + TEDS_HDR_SIZE;
    size_t   body_avail = buf_size - TEDS_HDR_SIZE - 2U; /* reserve 2 for CRC */
    size_t   body_len   = 0U;
    size_t   n;

#define APPEND(call)  do {              \
    n = (call);                         \
    if (n == 0U) { return -ENOMEM; }    \
    body       += n;                    \
    body_avail -= n;                    \
    body_len   += n;                    \
} while (0)

    APPEND(tlv_write_u8 (body, body_avail, TEDS_TID_SENSOR_TYPE, 0x00U));
    APPEND(tlv_write_str(body, body_avail, TEDS_TID_NAME,
                          cfg->type, TEDS_TYPE_MAX_LEN - 1U));
    APPEND(tlv_write_str(body, body_avail, TEDS_TID_UNIT,
                          cfg->unit, TEDS_UNIT_MAX_LEN - 1U));
    APPEND(tlv_write_f32(body, body_avail, TEDS_TID_LOWER_RANGE, cfg->range_min));
    APPEND(tlv_write_f32(body, body_avail, TEDS_TID_UPPER_RANGE, cfg->range_max));

    if (cfg->use_2pt_cal) {
        APPEND(tlv_write_u8 (body, body_avail, TEDS_TID_CAL_MODE,  TEDS_CAL_2PT));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_CAL_RAW1,  cfg->cal_raw1));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_CAL_REF1,  cfg->cal_ref1));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_CAL_RAW2,  cfg->cal_raw2));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_CAL_REF2,  cfg->cal_ref2));
    } else {
        APPEND(tlv_write_u8 (body, body_avail, TEDS_TID_CAL_MODE,  TEDS_CAL_LINEAR));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_SCALE,     cfg->scale));
        APPEND(tlv_write_f32(body, body_avail, TEDS_TID_OFFSET,    cfg->offset));
    }

    /* End-of-body marker */
    if (body_avail < 1U) {
        return -ENOMEM;
    }
    *body = TEDS_TID_END;
    body_len++;

#undef APPEND

    /* Fill header */
    teds_hdr_t *hdr = (teds_hdr_t *)buf;
    hdr->teds_class = TEDS_CLASS_XDCR_CHAN;
    hdr->version    = TEDS_VERSION;
    hdr->body_len   = (uint16_t)body_len;

    /* Append CRC16 over [header + body] */
    size_t crc_offset = TEDS_HDR_SIZE + body_len;
    uint16_t crc = teds_crc16(buf, crc_offset);
    buf[crc_offset]     = (uint8_t)(crc & 0xFFU);
    buf[crc_offset + 1] = (uint8_t)((crc >> 8) & 0xFFU);

    int total = (int)(crc_offset + 2U);
    LOG_INF("teds_tlv_build: %d bytes  type=%s unit=%s cal=%s",
            total, cfg->type, cfg->unit,
            cfg->use_2pt_cal ? "2PT" : "LIN");
    return total;
}

/* ---------------------------------------------------------------
 * teds_tlv_parse
 * --------------------------------------------------------------- */
int teds_tlv_parse(const uint8_t *buf, size_t len, teds_config_t *out)
{
    if (!buf || !out || len < (TEDS_HDR_SIZE + 2U)) {
        return -EINVAL;
    }

    const teds_hdr_t *hdr = (const teds_hdr_t *)buf;

    if (hdr->teds_class != TEDS_CLASS_XDCR_CHAN) {
        LOG_ERR("teds_tlv_parse: class=0x%02x không hỗ trợ (cần 0x%02x)",
                hdr->teds_class, TEDS_CLASS_XDCR_CHAN);
        return -EINVAL;
    }
    if (hdr->version != TEDS_VERSION) {
        LOG_ERR("teds_tlv_parse: version=0x%02x không hỗ trợ (cần 0x%02x)",
                hdr->version, TEDS_VERSION);
        return -EINVAL;
    }

    /* Validate total frame length: header + body + 2-byte CRC */
    size_t expected = TEDS_HDR_SIZE + (size_t)hdr->body_len + 2U;
    if (expected > len) {
        LOG_ERR("teds_tlv_parse: body_len=%u vượt quá buffer (got %zu expected %zu)",
                hdr->body_len, len, expected);
        return -EINVAL;
    }

    /* Verify CRC (covers header + body, i.e. bytes [0 .. HDR+body_len-1]) */
    size_t crc_offset   = TEDS_HDR_SIZE + (size_t)hdr->body_len;
    uint16_t stored_crc = (uint16_t)buf[crc_offset]
                        | ((uint16_t)buf[crc_offset + 1] << 8);
    uint16_t calc_crc   = teds_crc16(buf, crc_offset);

    if (stored_crc != calc_crc) {
        LOG_ERR("teds_tlv_parse: CRC lỗi (stored=0x%04x calc=0x%04x)",
                stored_crc, calc_crc);
        return -EBADMSG;
    }

    /* Initialise output with safe defaults */
    memset(out, 0, sizeof(*out));
    out->scale     =  1.0f;
    out->offset    =  0.0f;
    out->range_min = -9999.0f;
    out->range_max =  9999.0f;
    out->valid     = false;

    /* Walk TLV body */
    const uint8_t *p   = buf + TEDS_HDR_SIZE;
    const uint8_t *end = p + hdr->body_len;

    while (p < end) {
        uint8_t tid = *p++;
        if (tid == TEDS_TID_END) {
            break;
        }
        if (p >= end) {
            break;
        }

        uint8_t flen = *p++;
        if (p + flen > end) {
            LOG_ERR("teds_tlv_parse: TID=0x%02x len=%u vượt body", tid, flen);
            return -EINVAL;
        }

        float f32 = 0.0f;
        if (flen == 4U) {
            memcpy(&f32, p, sizeof(float));
        }

        switch (tid) {
        case TEDS_TID_NAME: {
            uint8_t copy = (flen < (TEDS_TYPE_MAX_LEN - 1U))
                           ? flen : (TEDS_TYPE_MAX_LEN - 1U);
            memcpy(out->type, p, copy);
            out->type[copy] = '\0';
            break;
        }
        case TEDS_TID_UNIT: {
            uint8_t copy = (flen < (TEDS_UNIT_MAX_LEN - 1U))
                           ? flen : (TEDS_UNIT_MAX_LEN - 1U);
            memcpy(out->unit, p, copy);
            out->unit[copy] = '\0';
            break;
        }
        case TEDS_TID_LOWER_RANGE:  out->range_min   = f32;                 break;
        case TEDS_TID_UPPER_RANGE:  out->range_max   = f32;                 break;
        case TEDS_TID_CAL_MODE:     out->use_2pt_cal = (p[0] == TEDS_CAL_2PT); break;
        case TEDS_TID_SCALE:        out->scale       = f32;                 break;
        case TEDS_TID_OFFSET:       out->offset      = f32;                 break;
        case TEDS_TID_CAL_RAW1:     out->cal_raw1    = f32;                 break;
        case TEDS_TID_CAL_REF1:     out->cal_ref1    = f32;                 break;
        case TEDS_TID_CAL_RAW2:     out->cal_raw2    = f32;                 break;
        case TEDS_TID_CAL_REF2:     out->cal_ref2    = f32;                 break;
        default:
            LOG_DBG("teds_tlv_parse: TID=0x%02x không biết – bỏ qua", tid);
            break;
        }
        p += flen;
    }

    out->valid = true;
    LOG_INF("teds_tlv_parse OK: type=%s unit=%s cal=%s range=[%.1f, %.1f]",
            out->type, out->unit,
            out->use_2pt_cal ? "2PT" : "LIN",
            (double)out->range_min, (double)out->range_max);
    return 0;
}
