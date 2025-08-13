
#pragma once

#include <cstdint>

#ifdef ARDUINO
    #include <Arduino.h>

    #define _MODPASH_PROGMEM PROGMEM
#else
    #define _MODPASH_PROGMEM

    #define pgm_read_byte(x) (*(x))
#endif

#ifdef _MODPASH_CRC_TABLE_NOT_PROGMEM
    #define MODPASH_PROGMEM
#endif

namespace modpash {

    /**
     * @brief 增量式计算Modbus CRC16，计算结果的高低字节顺序和MODBUS 帧尾顺序相同，也就是低字节在前
     *
     * @param data
     * @param last_crc_h 初始值为0xff
     * @param last_crc_l 初始值为0xff
     * @return uint16_t
     */
    inline uint16_t incremental_crc16(uint8_t data, uint8_t last_crc_h, uint8_t last_crc_l) {
        extern const uint8_t modbus_crc16_table_h[256] _MODPASH_PROGMEM;
        extern const uint8_t modbus_crc16_table_l[256] _MODPASH_PROGMEM;

        uint_fast16_t index = last_crc_h ^ data;

#ifdef _MODPASH_CRC_TABLE_NOT_PROGMEM
        // 把CRC 表放在PROGMEM 里可能会降低计算速度
        uint8_t table_h = modbus_crc16_table_h[index];
        uint8_t table_l = modbus_crc16_table_l[index];
#else
        uint8_t table_h = pgm_read_byte(&modbus_crc16_table_h[index]);
        uint8_t table_l = pgm_read_byte(&modbus_crc16_table_l[index]);
#endif

        last_crc_h = last_crc_l ^ table_h;
        last_crc_l = table_l;
        return static_cast<uint16_t>(last_crc_h << 8) + last_crc_l;
    }

    /**
     * @brief 增量式计算Modbus CRC16，计算结果的高低字节顺序和MODBUS 帧尾顺序相同，也就是低字节在前
     *
     * @param data
     * @param last_crc    初始值为0xffff
     * @return uint16_t
     */
    inline uint16_t incremental_crc16(uint8_t data, uint16_t last_crc) {
        uint8_t h = static_cast<uint8_t>(last_crc >> 8);
        uint8_t l = static_cast<uint8_t>(last_crc & 0x00ff);
        return incremental_crc16(data, h, l);
    }
}  // namespace modpash