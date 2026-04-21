#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "../Inc/xbee.h"

/*
 * xbee_decode_tx_request
 *
 * Decodes one XBee API Transmit Request (0x10) packet and copies the RF
 * data payload into 'out_buf' as a null-terminated char array.
 *
 * Parameters:
 *   packet      - pointer to the raw packet bytes
 *   packet_len  - total byte count of 'packet'
 *   out_buf     - caller-supplied buffer to receive the payload string
 *   out_buf_len - size of 'out_buf' in bytes (must fit payload + '\0')
 *   out_data_len- (optional, may be NULL) receives the payload byte count
 *                 not counting the null terminator
 *
 * Returns:
 *   XBEE_OK on success; a negative xbee_status_t value on error.
 *
 * Notes:
 *   - The checksum is verified before any data is copied.
 *   - The payload is treated as raw bytes; the null terminator appended
 *     to out_buf is a convenience — payloads containing embedded 0x00
 *     bytes must use out_data_len for the true length.
 */
xbee_status_t xbee_decode_tx_request(
        const uint8_t  *packet,
        size_t          packet_len,
        char           *out_buf,
        size_t          out_buf_len,
        size_t         *out_data_len)
{
    if (!packet || !out_buf) {
        return XBEE_ERR_NULL;
    }

    /* ── 1. Minimum length guard ─────────────────────────────────────── */
    if (packet_len < XBEE_MIN_PACKET_LEN) {
        return XBEE_ERR_TOO_SHORT;
    }

    /* ── 2. Start delimiter ──────────────────────────────────────────── */
    if (packet[OFF_START] != XBEE_START_DELIM) {
        return XBEE_ERR_NO_START;
    }

    /* ── 3. Frame type ───────────────────────────────────────────────── */
    if (packet[OFF_FRAME_TY] != XBEE_FRAME_TX_REQ) {
        return XBEE_ERR_WRONG_TYPE;
    }

    /* ── 4. Length field consistency ─────────────────────────────────── */
    /*
     * The length field counts bytes from the frame-type byte (B3) up to
     * and including the last RF data byte — it does NOT count the start
     * delimiter, the two length bytes, or the checksum byte.
     *
     * Minimum value when payload is empty: 14
     *   (1 type + 1 frame-id + 8 dest64 + 2 dest16 + 1 bcast + 1 opts)
     */
    uint16_t frame_len = ((uint16_t)packet[OFF_LEN_H] << 8) |
                          (uint16_t)packet[OFF_LEN_L];

    size_t expected_total = (size_t)frame_len + XBEE_HEADER_OVERHEAD + 1; /* +1 checksum */
    if (expected_total != packet_len) {
        return XBEE_ERR_BAD_LENGTH;
    }

    /* ── 5. Checksum (sum of bytes B3..B(last-1), low byte must be 0xFF) */
    uint8_t sum = 0;
    for (size_t i = OFF_FRAME_TY; i < packet_len - 1; i++) {
        sum += packet[i];
    }
    if ((uint8_t)(0xFF - sum) != packet[packet_len - 1]) {
        return XBEE_ERR_CHECKSUM;
    }

    /* ── 6. Extract RF data payload ──────────────────────────────────── */
    /*
     * RF data starts at B15 and ends one byte before the checksum.
     * frame_len = 14 (fixed fields from frame-type to options) + data_len
     */
    size_t data_len = 0;
    if (frame_len > 14) {
        data_len = frame_len - 12;
    }

    if (out_buf_len < data_len + 1) { /* +1 for null terminator */
        return XBEE_ERR_SMALL_BUF;
    }

    if (data_len > 0) {
        memcpy(out_buf, &packet[OFF_RF_DATA], data_len);
    }
    out_buf[data_len] = '\0';

    if (out_data_len) {
        *out_data_len = data_len;
    }

    return XBEE_OK;
}
