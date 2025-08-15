#pragma once

// DEBUG
// #include <Arduino.h>

#include <scheduler_tick.hpp>

#include "basic_access.hpp"
#include "rtu/non_block_serial.hpp"

namespace modpash {

    /**
     * @brief 实现寄存器读写功能的主机类
     *
     * BasicClient 只实现基本的uint16 寄存器读写，不将寄存器考虑成“属性”，
     * 读写都以uint16 为单位。
     *
     * 【 工作循环 】
     *
     * BasicClient 与BasicPropertyServer 类似，采用非阻塞轮询的工作方式。
     * 区别是，Server 先轮询并接受请求，然后发送响应；而Client 先发送请求，然后轮询并等待响应，
     * 在等待中，要处理超时问题。
     *
     * 麻烦的地方是，主机经常要顺序执行一串请求，其中每次请求后都要等待响应、判断响应结果。
     * 处理这种情况，最好的方法是用协程，把异步的代码组合成一串看起来是同步的过程；单片机上不方便用协程，
     * 也不方便频繁创建回调闭包对象，所以只能使用状态机的方式来处理，或者用死循环等待的同步执行方式。
     *
     * 【 响应超时 】
     *
     * 等待响应时，有两种处理方式：
     *
     * 1. 只要接收到了可疑的数据，就认为这是响应帧，结束等待超时；
     * 2. 只有接收到有效的响应帧，才结束等待超时。
     *
     * 方式1 比较敏感，如果接收到的帧无效，就可以立刻重试；方式2 则需要继续等待超时。当从机响应的数据受到干扰，
     * 无法正确解析时，方式2 继续等待下去也不会再接收到数据，从而降低了通信效率。
     *
     * 而方式1 接收到的数据有可能不是从机的响应，此时从机可能正准备响应，如果主机立即重试，就会破坏一问一答的通信规则。
     * 所以采用方式2 更加稳妥。
     *
     * 此外，如果请求的地址是广播地址，就不等待响应。
     *
     */
    template <typename TransceiverType>
    class BasicClient {
        // DEBUG
       public:
        // using TransceiverType = NonBlockSerialFrameHandler<scheduler_basic::ArduinoMsSource, HardwareSerial>;
        using TimeSource = typename TransceiverType::TimeSource;
        using TimeType = typename TimeSource::TimeType;

       private:
        TransceiverType *_rtx;  ///< 收发器对象

        TimeSource _response_waiting_timer;  ///< 响应等待计时器

        TimeType _response_waiting_timeout = RTU_RESPONSE_TIMEOUT;  ///< 响应等待超时时间，单位毫秒

        error_level _last_error = error_level::none;  ///< 上一次错误状态

        uint8_t _last_request_function_code = 0;  ///< 上一次请求的功能码

        uint8_t _last_requested_count = 0;  ///< 上一次请求的寄存器数量

        uint8_t _data_remaining_count = 0;  ///< 剩余数据字节数，表示当前请求或响应帧中还有多少数据未处理

        bool _is_waiting_for_response = false;  ///< 是否正在等待响应

        bool _always_flush = true; ///< 是否在每次发送请求后都强制刷新串口缓冲区，默认为true

        // 禁止拷贝构造和赋值
        BasicClient(const BasicClient &) = delete;

       public:
        /**
         * @brief 构造函数
         *
         * @param rtx 收发器对象
         */
        BasicClient(TransceiverType &rtx) noexcept :
            _rtx(&rtx) {}

        /**
         * @brief 准备向指定地址发送请求，并接收响应，如果地址为广播地址，则不接收响应。
         *
         * @param target_address
         */
        void begin(uint8_t target_address) {
            _rtx->end_session();
            _rtx->begin_session(target_address, true);
            _response_waiting_timer.reset();
            _is_waiting_for_response = false;
        }

        void begin() {
            begin(_rtx->address());
        }

        bool request_read(uint16_t address, uint16_t count = 1) {
            // 发送请求帧
            _last_requested_count = count;  // 记录请求的寄存器数量
            _last_request_function_code = static_cast<uint8_t>(function_code::read_holding_registers);

            _rtx->begin_tx(_rtx->address());
            _rtx->tx(static_cast<uint8_t>(function_code::read_holding_registers));
            hide::send_data(_rtx, address);
            hide::send_data(_rtx, count);
            return _rtx->end_tx();
        }

        bool request_read_u32(uint16_t address) {
            return request_read(address, 2);  // 读取2个寄存器
        }

        bool request_read_float(uint16_t address) {
            return request_read(address, 2);  // 读取2个寄存器
        }

        bool request_write_u16(uint16_t address, uint16_t value) {
            _is_waiting_for_response = false;  // 发送请求帧时，设置为false，表示不再等待响应
            _last_requested_count = 1;  // 记录请求的寄存器数量
            _last_request_function_code = static_cast<uint8_t>(function_code::write_register);
            // 发送请求帧
            _rtx->begin_tx(_rtx->address());
            _rtx->tx(static_cast<uint8_t>(function_code::write_register));
            hide::send_data(_rtx, address);
            hide::send_data(_rtx, value);
            return _rtx->end_tx();
        }

        /**
         * @brief 请求写入2 个寄存器，或写入一个uint32
         *
         * @param address 寄存器起始地址
         * @param value 写入的值，假设写入2个寄存器
         * @return true 写入请求发送成功
         * @return false 写入请求发送失败
         */
        bool request_write_u32(uint16_t address, uint32_t value) {
            _is_waiting_for_response = false;  // 发送请求帧时，设置为false，表示不再等待响应
            _last_requested_count = 2;  // 记录请求的寄存器数量
            _last_request_function_code = static_cast<uint8_t>(function_code::write_multiple_registers);
            // 发送请求帧
            _rtx->begin_tx(_rtx->address());
            _rtx->tx(static_cast<uint8_t>(function_code::write_multiple_registers));
            hide::send_data(_rtx, address);
            hide::send_data(_rtx, static_cast<uint16_t>(2));  // 写入2个寄存器
            _rtx->tx(4);                                      // 数据字节数
            hide::send_data(_rtx, value);
            return _rtx->end_tx();
        }

        bool request_write_float(uint16_t address, float value) {
            hide::any_type_to_uint_converter<float> vv{.d = value};
            return request_write_u32(address, vv.dd);
        }

        bool request_multiple_write(uint16_t address, const uint16_t *values, uint16_t count) {
            _is_waiting_for_response = false;  // 发送请求帧时，设置为false，表示不再等待响应
            _last_requested_count = count;  // 记录请求的寄存器数量
            _last_request_function_code = static_cast<uint8_t>(function_code::write_multiple_registers);

            if (count == 0 || count > 123) {
                return false;  // MODBUS 协议限制一次最多写入123个寄存器
            }

            // 发送请求帧
            _rtx->begin_tx(_rtx->address());
            _rtx->tx(static_cast<uint8_t>(function_code::write_multiple_registers));
            hide::send_data(_rtx, address);
            hide::send_data(_rtx, count);
            _rtx->tx(static_cast<uint8_t>(count * 2));  // 数据字节数

            for (uint16_t i = 0; i < count; ++i) {
                hide::send_data(_rtx, values[i]);
            }

            return _rtx->end_tx();
        }

        /**
         * @brief 轮询调用，检查是否接收到有效的响应帧。
         *
         * 发送一帧请求后调用，如果尚未开始等待超时，则启动计时，如果超时范围内没有接收到有效响应帧，
         * 返回-1。
         *
         * 向广播地址发送的请求不会收到响应，所以固定返回-1。
         *
         */
        int_fast8_t response_available() {
            uint8_t addr = _rtx->requested_address();
            if (_rtx->is_broadcast_address(addr)) {
                _last_error = error_level::broadcast_no_response;
                return -1;
            }

            if (_is_waiting_for_response) {
                if (_rtx->rx_available() && _rtx->rx()) {
                    uint8_t rx_addr = _rtx->rx_frame_address();
                    uint8_t rx_function = _rtx->rx_frame_function() & 0x7F;  // 去掉异常响应标志位

                    if (rx_addr != addr || rx_function != _last_request_function_code) {
                        // 如果接收到的地址或功能码与请求不匹配，则认为是无效响应
                        // _rtx->clear();
                        _last_error = error_level::response_mismatch;  // 设置错误状态为响应不匹配
                        return -1;                                     // 无效响应
                    }
                    else {
                        _last_error = error_level::ok;     // 设置错误状态为正常
                        _is_waiting_for_response = false;  // 收到有效响应，停止等待
                        return 1;                          // 有效响应
                    }
                }
                else {
                    // 如果已经在等待响应，则检查是否超时
                    if (_response_waiting_timer.diff() > _response_waiting_timeout) {
                        _is_waiting_for_response = false;
                        _last_error = error_level::response_timeout;  // 设置错误状态为响应超时
                        return -1;                                    // 超时
                    }
                }
            }
            else {
                // 如果没有在等待响应，则开始等待
                _is_waiting_for_response = true;
                _response_waiting_timer.reset();

                // 等待发送完成
                if(_always_flush) {
                    _rtx->tx_complete(true);
                }
            }

            return 0; // 正在等待响应
        }

        /**
         * @brief 读取并检查响应帧，如果响应有效，返回寄存器数量或异常码，否则返回0。
         * 
         * 如果响应是异常响应，则返回负的异常码。
         * 如果响应是正常响应，则返回寄存器数量。
         * 
         * 就算是正常响应，也可能存在错误，错误状态可从last_error 获取。
         * 
         * @return int_fast8_t 
         */
        int_fast8_t check_response() {
            _last_error = error_level::none;  // 重置错误状态
            int func = _rtx->read();
            if (func < 0) {
                // 没有接收到响应
                return 0;
            }

            _last_error = error_level::ok;  // 设置错误状态为正常
            if (func & 0x80) {
                // 异常响应
                uint8_t err = static_cast<uint8_t>(_rtx->read());
                return 0 - err;
            }
            else {
                // 正常响应
                // uint16_t addr = _rtx->read();
                uint16_t count = 0;

                if (func == static_cast<int>(function_code::write_register)) {
                    // 单寄存器写入响应
                    count = 1;
                }
                else if (func == static_cast<int>(function_code::write_multiple_registers)) {
                    // 多寄存器写入响应
                    _rtx->read();  // 读取地址，虽然不需要使用
                    _rtx->read();
                    uint8_t count_h = _rtx->read();
                    uint8_t count_l = _rtx->read();
                    count = (static_cast<uint16_t>(count_h) << 8) | count_l;
                }
                else if(func == static_cast<int>(function_code::read_holding_registers)) {
                    // 读取寄存器响应
                    uint8_t byte_count = _rtx->read();
                    count = byte_count / 2;  // 每个寄存器2字节
                    _data_remaining_count = byte_count;  // 更新剩余数据字节数

                    if (count != _last_requested_count || byte_count % 2 != 0) {
                        _last_error = error_level::response_count_mismatch;  // 设置错误状态为响应寄存器数量不匹配
                    }
                }
                else {
                    _last_error = error_level::invalid_function;  // 设置错误状态为无效功能码
                    return 0;
                }

                return count;
            }
        }

        uint16_t read_u16() {
            if (_data_remaining_count < 2) {
                _last_error = error_level::data_insufficient;  // 设置错误状态为数据不足
                return 0;
            }

            uint16_t value = static_cast<uint16_t>(_rtx->read());
            value <<= 8;
            value += static_cast<uint16_t>(_rtx->read());
            _last_error = error_level::ok;  // 设置错误状态为正常
            _data_remaining_count -= 2;  // 减少剩余数据字节数
            return value;
        }

        uint32_t read_u32() {
            if (_data_remaining_count < 4) {
                _last_error = error_level::data_insufficient;  // 设置错误状态为数据不足
                return 0;
            }

            uint32_t value = static_cast<uint32_t>(_rtx->read());
            value <<= 8;
            value += static_cast<uint32_t>(_rtx->read());
            value <<= 8;
            value += static_cast<uint32_t>(_rtx->read());
            value <<= 8;
            value += static_cast<uint32_t>(_rtx->read());
            _last_error = error_level::ok;  // 设置错误状态为正常
            _data_remaining_count -= 4;  // 减少剩余数据字节数
            return value;
        }

        float read_float() {
            if (_data_remaining_count < 4) {
                _last_error = error_level::data_insufficient;  // 设置错误状态为数据不足
                return 0.0f;
            }

            hide::any_type_to_uint_converter<float> vv;
            vv.dd = 0;

            for (int i = 0; i < 4; ++i) {
                vv.dd <<= 8;
                vv.dd += static_cast<uint8_t>(_rtx->read());
            }

            _last_error = error_level::ok;  // 设置错误状态为正常
            _data_remaining_count -= 4;  // 减少剩余数据字节数
            return vv.d;
        }

        error_level last_error() const {
            return _last_error;
        }

        /**
         * @brief 获取响应的功能码
         */
        uint8_t response_function_code() const {
            return _rtx->rx_frame_function();
        }

        void set_response_timeout(TimeType timeout) {
            _response_waiting_timeout = timeout;
        }

        TimeType response_timeout() const {
            return _response_waiting_timeout;
        }

        bool always_flush() const {
            return _always_flush;
        }

        void enable_always_flush(bool flush) {
            _always_flush = flush;
        }
    };

}  // namespace modpash
