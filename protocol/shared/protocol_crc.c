/**
 * @file    protocol_crc.c
 * @brief   CRC16-CCITT implementation for the binary protocol.
 */

#include "protocol.h"

#include <stddef.h>

uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)(data[i] << 8);
        for (j = 0U; j < 8U; j++)
        {
            if (crc & 0x8000U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint16_t proto_encode_frame(uint8_t *buf, uint16_t buf_size,
                            uint8_t type, uint16_t seq,
                            const uint8_t *payload, uint16_t payload_len)
{
    uint16_t total_len;
    uint16_t crc;

    if ((payload_len > PROTO_MAX_PAYLOAD) ||
        ((payload_len > 0U) && (payload == NULL)) ||
        (buf == NULL))
    {
        return 0U;
    }

    total_len = PROTO_FRAME_OVERHEAD + payload_len + PROTO_CRC_SIZE;
    if (buf_size < total_len)
    {
        return 0U;
    }

    /* Header */
    buf[0] = PROTO_MAGIC_0;
    buf[1] = PROTO_MAGIC_1;
    buf[2] = PROTO_VERSION;
    buf[3] = type;
    buf[4] = (uint8_t)(seq & 0xFFU);
    buf[5] = (uint8_t)((seq >> 8) & 0xFFU);
    buf[6] = (uint8_t)(payload_len & 0xFFU);
    buf[7] = (uint8_t)((payload_len >> 8) & 0xFFU);

    /* Payload */
    if (payload_len > 0U && payload != NULL)
    {
        uint16_t k;
        for (k = 0U; k < payload_len; k++)
        {
            buf[PROTO_FRAME_OVERHEAD + k] = payload[k];
        }
    }

    /* CRC over header + payload */
    crc = proto_crc16(buf, PROTO_FRAME_OVERHEAD + payload_len);
    buf[PROTO_FRAME_OVERHEAD + payload_len]     = (uint8_t)(crc & 0xFFU);
    buf[PROTO_FRAME_OVERHEAD + payload_len + 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    return total_len;
}
