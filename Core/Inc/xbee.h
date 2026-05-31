/*
 * xbee.h
 *
 *  Created on: Apr 20, 2026
 *      Author: Joel
 */

#ifndef INC_XBEE_H_
#define INC_XBEE_H_

/*
 * XBee API Frame offsets (AP=1, standard mode, frame type 0x10)
 *
 *  B0       : Start delimiter (0x7E)
 *  B1–B2    : Length (big-endian, counts bytes from frame type to last data byte)
 *  B3       : Frame type (must be 0x10 for Transmit Request)
 *  B4       : Frame ID
 *  B5–B12   : 64-bit destination address (big-endian)
 *  B13–B14  : 16-bit network destination address
 *  B15      : Broadcast radius
 *  B16      : Transmit options
 *  B17...   : RF data (payload) — variable length
 *  Last     : Checksum
 *
 * Total minimum packet size (zero payload): 18 bytes
 */

#define XBEE_START_DELIM    0x7E
#define XBEE_FRAME_TX_REQ   0x10
#define XBEE_FRAME_RX       0x90
#define XBEE_MIN_PACKET_LEN 15   /* header (14) + checksum (1) */
#define XBEE_HEADER_OVERHEAD 3   /* start + 2 length bytes, not counted in length field */

/* Byte offsets */
#define OFF_START    0
#define OFF_LEN_H    1
#define OFF_LEN_L    2
#define OFF_FRAME_TY 3
#define OFF_FRAME_ID 4
#define OFF_DEST64   5   /* 8 bytes */
#define OFF_DEST16   13  /* 2 bytes */
#define OFF_BCAST_R  15
#define OFF_OPTIONS  16
#define OFF_RF_DATA  15

typedef enum {
    XBEE_OK              =  0,
    XBEE_ERR_NULL        = -1,  /* NULL pointer argument         */
    XBEE_ERR_TOO_SHORT   = -2,  /* buffer shorter than minimum   */
    XBEE_ERR_NO_START    = -3,  /* missing 0x7E start delimiter  */
    XBEE_ERR_WRONG_TYPE  = -4,  /* frame type is not 0x10        */
    XBEE_ERR_BAD_LENGTH  = -5,  /* length field inconsistent     */
    XBEE_ERR_CHECKSUM    = -6,  /* checksum verification failed  */
    XBEE_ERR_SMALL_BUF   = -7,  /* output buffer too small       */
} xbee_status_t;

xbee_status_t xbee_decode_tx_request(const uint8_t *packet, size_t packet_len, char *out_buf, size_t out_buf_len, size_t *out_data_len);

xbee_status_t xbee_send_api_packet(const char* packet_data, uint16_t packet_len, char* out_buf, uint16_t out_buf_len, uint16_t* out_data_len);

#endif /* INC_XBEE_H_ */
