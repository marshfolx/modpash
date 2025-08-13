#pragma once

#include <stdint.h>
#include <stdlib.h>


namespace modpash {
    /**
     * @brief 从头部开始依次读取ADU 数据，得出ADU 帧长度
     *
     */
    class DefaultAduInspector {
       public:
        static constexpr size_t MAX_HEADER_LEN = 6;  //

       private:
        int_fast16_t _adu_size = 0;

        uint_fast8_t _adu_index = 0;  // 当前接收的字节是ADU 中第几个, 0 表示功能码
        uint_fast8_t _function_code = 0;

        bool _is_request = false;

        void _parse_request_byte_5(uint8_t d) {
            switch (_function_code) {
                case 15:
                case 16:
                    _adu_size = d + 6;
                    break;

                default:
                    break;
            }
        }


        /**
         * @brief 解析ADC 第0 字节，即功能码，有些指令请求只需要功能码就能知道整个ADC 的长度
         *
         * @param d
         * @return true
         * @return false
         */
        void _parse_request_byte_0(uint8_t d) {
            switch (d) {
                // 大部分请求帧整体为8 字节，ADU 部分为5 字节
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 8:
                    _adu_size = 5;
                    break;

                case 11:
                case 12:
                case 17:
                    _adu_size = 1;

                default:
                    break;
            }
        }

        void _parse_response_byte_0(uint8_t d) {
            if (d & 0x80) {
                // 异常响应
                _adu_size = 2;
            }
            else {
                switch (d) {
                    case 5:
                    case 6:
                    case 8:
                    case 11:
                    case 15:
                    case 16:
                        _adu_size = 5;
                        break;

                    default:
                        break;
                }
            }
        }

        void _parse_response_byte_1(uint8_t d) {
            // 变长响应帧的第一字节是后面数据域的字节数
            switch (_function_code) {
                case 1:
                case 2:
                case 3:
                case 4:
                case 12:
                case 17:
                    _adu_size = 2 + d;
                    break;

                default:
                    break;
            }
        }

        void _parse_byte_0(uint8_t d) {
            if (_is_request) {
                _parse_request_byte_0(d);
            }
            else {
                _parse_response_byte_0(d);
            }
        }

        void _parse_byte_1(uint8_t d) {
            if (!_is_request) {
                _parse_response_byte_1(d);
            }
        }

        void _parse_byte_5(uint8_t d) {
            if (_is_request) {
                _parse_request_byte_5(d);
            }
        }

       public:
        DefaultAduInspector() {}

        size_t adu_size() {
            return _adu_size;
        }

        uint8_t function_code() {
            return static_cast<uint8_t>(_function_code);
        }

        int_fast16_t feed(uint8_t d) {
            if (_adu_index >= MAX_HEADER_LEN) {
                return -1;
            }

            switch (_adu_index) {
                case 0:
                    if (!is_supported_function(d)) {
                        return -1;
                    }
                    _function_code = d;
                    _parse_byte_0(d);
                    break;

                case 1:
                    _parse_byte_1(d);
                    break;

                case 5:
                    _parse_byte_5(d);
                    break;
            }

            ++_adu_index;

            // DEBUG
            // if (_adu_size > 0) {
            //     Serial.printf("ADU INDEX = %d\n", _adu_index);
            //     Serial.printf("ADU SIZE = %d\n", _adu_size);
            // }

            return _adu_size;
        }

        /**
         * @brief 查询功能码是否在支持范围内
         *
         * 默认支持的功能码包括：
         *
         * 1:  读取线圈输出状态     REQ 5 REP N
         * 2:  读取线圈输入状态     REQ 5 REP N
         * 3:  读取保持寄存器       REQ 5 REP N
         * 4:  读取输入寄存器       REQ 5 REP N
         * 5:  写入单个线圈         REQ 5 REP 5
         * 6:  写单个保持寄存器     REQ 5 REP 5
         * 8:  诊断                 REQ 5 REP 5
         * 11: 获取通信事件计数     REQ 1 REP 5
         * 12: 获取通信事件记录     REQ 1 REP N
         * 15: 写多个线圈           REQ N REP 5
         * 16: 写多个保持寄存器     REQ N REP 5
         * 17: 获取从站ID           REQ 1 REP N
         *
         * 这些功能码的请求、响应，或异常帧可以被收发器接收，但设备不一定支持该功能；
         * 其他功能码收发器无法接收，等同于通信失败。
         *
         * 只有主机才会接收异常响应，从机会忽略。
         *
         * @param function_code
         * @return true
         * @return false
         */
        bool is_supported_function(uint8_t function_code) const {
            if (!_is_request) {
                function_code &= 0x7f;
            }

            if (((function_code >= 1) && (function_code <= 6))
                || (function_code == 8)
                || (function_code == 11)
                || (function_code == 12)
                || ((function_code >= 15) && (function_code <= 17))) {
                // 参考：《MODBUS 软件开发实战指南》-- 杨更更
                return true;
            }
            else {
                return false;
            }
        }

        /**
         * @brief 如果是主机，则接收的应该只有响应，is_request == false；否则，is_request == true
         *
         * @param is_request
         */
        void reset(bool is_request) {
            _adu_index = 0;
            _adu_size = 0;
            _is_request = is_request;
        }
    };
}  // namespace modpash