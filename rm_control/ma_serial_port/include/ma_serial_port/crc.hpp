#ifndef __MA_SERIAL_CRC_H__
#define __MA_SERIAL_CRC_H__

#include <cstdint>
#include <iostream>

namespace crc16
{
    uint32_t verify_crc16_checksum(const uint8_t* pch_message, uint32_t dw_length);

    void append_crc16_checksum(uint8_t* pch_message, uint32_t dw_length);

}

#endif