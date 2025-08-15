
#pragma once

// DEBUG
// #include <Arduino.h>
// #include <scheduler_tick.hpp>

#include "basic_access.hpp"
// #include "rtu/non_block_serial.hpp"
#include "modpash_common.hpp"

namespace modpash {

    /**
     * @brief 基本的从机属性Server 类
     *
     * 【 MODBUS 大小端兼容性 】
     *
     * MODBUS 默认将数据都作为uint16 传输，且通信数据帧中使用大端序。
     * 现成的MODBUS 库基本都将连续的数据分割成uint16 的多个寄存器数据；将接收到的数据自动转换为小端序。
     * 考虑到上位机开发都会使用pymodbus 之类现成的库，所以modpash 应该尽量与之兼容。
     * 收发器层面上无视数据端序，只在应用层处理。
     *
     * 【 传输多字节数据 】
     *
     * 需要传输宽度大于16bit 的数据时，MODBUS 默认没有规定字节序，所以就整出来了所谓的四种排列组合顺序。
     * MODPASH 强制要求所有数据使用大端序，如果要发送一个uint32 变量，应该按照最高字节到低字节的顺序写入数据帧。
     * 比如，32 位数据值为: 0xaabbccdd，默认从机硬件使用小端序，则存储形式是: dd cc bb aa，左侧对应低地址；
     * 发送时，转为大端序: aa bb cc dd。
     * 上位机MODBUS 通信库将32 位变量分成两个寄存器: [ aa bb ] [ cc dd ]，再分别转换回小端序: [ bb aa ] [ dd cc ]，
     * 对应数据为：0xaabb 0xccdd
     * 上位机软件接着只需将这两个16 位数据转换回去：(0xaabb << 16) + (0xccdd) == 0xaabbccdd。
     *
     * 【 寄存器组织 】
     *
     * BasicPropertyServer 不会直接构造出线性连续的寄存器组供主机访问，取而代之的是将寄存器地址视为访问路径或“属性”。
     * 每个“寄存器”不与实际存储空间对应，访问寄存器01 时，无论读取多少数据，都不会直接牵涉到寄存器02。
     * 每个地址具有独立的意义，由应用实现，所以称之为"属性" 更恰当。
     * 比如，01 对应PID 的KP 参数，类型是float，要读取它，需要一次读取4 个字节；
     * 02, 03, 分别对应KI 和KD；而04 是一次读写KP、KI、KD 三个参数，需要一次请求 4 x 3 = 12 个字节。
     * 可以将04 视为数组，类型是float，长度是3。在通信中，每个数组元素都和单个数据一样按大端序排列，
     * 数组整体则按照地址由低到高排列。
     *
     * 请求中的“寄存器数”只被作为与标准MODBUS 保持兼容的特性。如果主机要读取的一个属性类型是double，尺寸是8 字节，
     * 请求的寄存器数就应该等于4，否则从机可以返回异常响应，或者只返回8 字节数据的一部分。
     *
     * 这样做最大的好处是解除了数据连续的保证，从机实现可以更灵活的支持多字节数据，主机不再需要关注从机的寄存器地址布局，
     * 只需按地址访问需要的数据。甚至可以将json 或msgpack 数据包整合到某个地址，向主机发送所有属性的列表及类型信息。
     * 缺点是，这样就不完全兼容Modbus Poll 之类的工具了，因为它默认一个寄存器地址只对应2 字节数据。
     *
     * 如果有必要，可以设置一个属性用来获取其他属性的数据长度。
     *
     * 【 功能码 】
     *
     * BasicPropertyServer 将不会支持有关线圈和输入寄存器的指令，可以设置在遇到不支持的功能码时返回异常响应或不响应。
     *
     * 支持的功能码包括：
     *
     * - 03: 读取保持寄存器
     * - 06: 写单个保持寄存器
     * - 11: 获取通信事件计数
     * - 16: 写多个保持寄存器
     * - 17: 获取从站ID
     *
     * 【 通信事件计数 】
     *
     * 可以设置Server 是否处于忙状态，事件计数值等于Server 成功响应计数，不包括诊断功能码的响应。
     *
     * 【 从站ID 】
     *
     * 可以设置响应的设备ID、运行状态，并追加自定义信息。
     * BasicPropertyServer 会在附加信息前部输出状态计数器等属性，每个属性是一个uint16 变量，
     * 高字节在前。这些属性之后是用户自定义信息。
     *
     * 附加信息列表：
     *
     * - [1]      : 1 字节，表示随后的属性个数，；
     * - [2, 3]   : Server 配置标志，就是一组标志位，用来反馈Server 当前的配置信息；
     * - [4, 5]   : 底层收发器成功接收的帧个数，对应收发器的rx_frame_count()；
     * - [6, 7]   : 底层收发器接收失败的帧个数，对应收发器的rx_miss_count()；
     * - [8, 9]   : Server 成功响应计数；
     * - [10, 11] : Server 异常响应计数，如果不启用异常响应，就不发出响应；
     * - [12]     : 用户自定义信息开始；
     *
     * 如果用户没有自定义信息，那么从站ID 的响应帧ADU 部分长度为 4 + 11 = 15 字节；
     * 用户自定义信息数量最多为255 - 11 = 244 字节。
     *
     * 【 Server 配置标志 】
     *
     * - 15 (MSB) : 0 表示关闭异常响应，否则为启用异常响应
     *
     * 其他位保留待用。
     *
     * @tparam TransceiverType
     */
    template <typename TransceiverType>
    class BasicPropertyServer {
       public:
        // static constexpr size_t DEFAULT_PROPERTY_BUFFER_SIZE = 32;  // 默认用来存储响应帧中的数据的缓冲区尺寸，单位是字节

        // static_assert((DEFAULT_PROPERTY_BUFFER_SIZE >= 16) && (DEFAULT_PROPERTY_BUFFER_SIZE % 2 == 0));

       private:
        TransceiverType *_rtx;

        uint16_t _ok_counter = 0;
        uint16_t _exception_counter = 0;
        uint16_t _status = 0;  // 对应返回事件计数时的状态字，0xFFFF 表示设备忙，0x0000 表示无事发生

        uint16_t _max_address = 0;

        uint8_t _server_id = 0;      // 报告设备ID 时返回，可自定义
        uint8_t _run_status = 0xff;  // 报告设备ID 时返回，可自定义；默认用0x00 表示OFF，0xff 表示ON

        uint8_t _data_remaining_cout;
        uint8_t _requested_count;

        uint8_t _pending_address = 0;  // 如果在处理请求过程中修改从机地址，会导致通信异常
                                       // 必须延迟到请求处理完成后

        bool _enable_exception_response = true;
        bool _enable_busy_exception = true;  // 状态为BUSY 时，自动返回BUSY 异常，默认启用

        bool _always_flush = true;  // 是否在每次发送请求后都强制刷新串口缓冲区，默认为true

        bool _locked = false;  // 正在处理响应时，锁定server，不处理新的请求

       protected:
        bool _is_diagnostic_function(uint8_t function_code) {
            return (function_code == 11) || (function_code == 12) || (function_code == 17);
        }

        bool _is_read_function(uint8_t function_code) {
            return (function_code == 3);
        }

        bool _is_write_function(uint8_t function_code) {
            return (function_code == 6) || (function_code == 16);
        }

        void _respond_exception(uint8_t flag) {
            if (_enable_exception_response) {
                _rtx->begin_tx(_rtx->rx_frame_address());
                _rtx->tx(_rtx->rx_frame_function() | 0x80);
                _rtx->tx(0 - flag);
                _rtx->end_tx();
            }
        }

        void _respond_event_counter() {
            _rtx->begin_tx(_rtx->rx_frame_address());
            _rtx->tx(11);        // 功能码
            _rtx->tx(4);         // 返回4 字节数据
            send_data(_status);  // 状态字，0xFFFF 表示设备忙，0x0000 表示无事发生
            send_data(_ok_counter);
            _rtx->end_tx();
        }

        void _respond_report_id() {
            constexpr auto EXTRA_PROPERTY_COUNT = 5;  // 附加属性个数，1 个属性 + 4 个状态计数器

            // 先发送从站ID
            _rtx->begin_tx(_rtx->rx_frame_address());
            _rtx->tx(17);                            // 功能码
            _rtx->tx(3 + EXTRA_PROPERTY_COUNT * 2);  // 2 个1 字节状态，加上1 字节属性个数，五个uint16 属性
            _rtx->tx(_server_id);                    // 从站ID
            _rtx->tx(_run_status);                   // 运行状态

            // 发送附加信息
            send_data(static_cast<uint8_t>(EXTRA_PROPERTY_COUNT));  // 属性个数，1 字节
            send_data(_config_flag());                              // Server 配置标志

            send_data(_rtx->rx_frame_count());  // 底层收发器成功接收的帧个数
            send_data(_rtx->rx_miss_count());   // 底层收发器接收失败的帧个数

            send_data(_ok_counter);         // Server 成功响应计数
            send_data(_exception_counter);  // Server 异常响应计数

            // TODO: 用户自定义信息

            _rtx->end_tx();
        }

        /**
         * @brief 返回代表Server 配置的bit flag
         *
         * @return uint16_t
         */
        uint16_t _config_flag() const {
            uint16_t flag = 0;
            if (_enable_exception_response) {
                flag |= 0x8000;
            }
            return flag;
        }

        void set_max_address(uint16_t addr) {
            _max_address = addr;
        }

       public:
        BasicPropertyServer(TransceiverType &rtx, uint16_t max_address) noexcept :
            _rtx(&rtx), _max_address(max_address) {}

        /**
         * @brief 轮询调用，检查是否接收到有效的请求帧，之后可以调用respond 执行响应。
         *
         * 如果返回request_type::none，表示没有接收到，否则返回接收到的请求类型。
         *
         * 如果请求帧的功能码不在支持的范围内，返回request_type::invalid_request，
         * 此时调用respond，如果设置启用异常响应，就会发送功能码异常响应。
         *
         * @return request_type
         */
        request_type request_available() {
            if (_rtx->rx_available()) {
                if (_rtx->rx()) {
                    auto func = _rtx->rx_frame_function();

                    switch (func) {
                        case 0:
                            return request_type::none;

                        case 3:
                            return request_type::read;

                        case 6:
                        case 16:
                            return request_type::write;

                        case 11:
                            return request_type::read_counter;

                        case 17:
                            return request_type::report_id;

                        default:
                            return request_type::invalid_function;
                    }
                }
            }

            return request_type::none;
        }

        /**
         * @brief 调用收发器获取请求帧，然后返回响应
         *
         * @return true
         * @return false
         */
        void respond(request_type t) {
            if(_enable_busy_exception && busy()) {
                // 如果处于忙状态，直接返回异常响应
                _respond_exception(RESPONSE_ERROR_BUSY);
                return;
            }

            _locked = true;

            switch (t) {
                case request_type::none:
                    break;

                case request_type::read:
                case request_type::write:
                    on_access(t);
                    break;

                case request_type::report_id:
                    _respond_report_id();
                    break;

                case request_type::read_counter:
                    _respond_event_counter();
                    break;

                case request_type::invalid_function:
                    ++_exception_counter;
                    _respond_exception(RESPONSE_ERROR_INVALID_FUNCTION);
                    break;

                default:
                    // 发生了未知错误，可能是收发器异常或其他问题
                    ++_exception_counter;
                    _respond_exception(RESPONSE_ERROR_DEVICE_ERROR);
                    break;
            }

            // 发送完响应后，等待发送完成
            if (_always_flush) {
                _rtx->tx_complete(true);
            }

            if (_pending_address != 0) {
                _rtx->end_session();
                _rtx->begin_session(_pending_address, false);
                _pending_address = 0;
            }

            _locked = false;
        }

        void listen() {
            // 轮询接收数据
            if (_locked)
                return;

            auto t = request_available();
            if (t != request_type::none) {
                respond(t);
            }
        }

        virtual void on_access(request_type t) {
            uint8_t func = static_cast<uint8_t>(_rtx->read());

            // 03 和16 功能码，[1, 2] 字节是寄存器地址，[3, 4] 是寄存器数量；
            // 06 功能码[3, 4] 是要修改的值。
            uint8_t addr_h = _rtx->read();
            uint8_t addr_l = _rtx->read();
            uint8_t count_or_value_h = _rtx->read();
            uint8_t count_or_value_l = _rtx->read();

            uint16_t addr = (static_cast<uint16_t>(addr_h) << 8) + addr_l;
            uint16_t count_or_value = (static_cast<uint16_t>(count_or_value_h) << 8) + count_or_value_l;

            if (addr > _max_address) {
                _respond_exception(RESPONSE_ERROR_INVALID_ADDRESS);
                return;
            }

            auto flag = RESPONSE_TYPE_NONE;

            // 03
            if (func == static_cast<uint8_t>(function_code::read_holding_registers)) {
                // 寄存器数量不会超过uint8_t 的范围，
                // 除非是要操作线圈，但是这个Server 不支持线圈。
                // 读取操作的响应帧在on_read 内部构造并发送
                // 除非返回了异常码。
                if (count_or_value_l < 1) {
                    _respond_exception(RESPONSE_ERROR_INVALID_COUNT);
                    return;
                }

                _requested_count = count_or_value_l;
                flag = on_read(addr, count_or_value_l);
            }
            else {
                if (func == static_cast<uint8_t>(function_code::write_multiple_registers)) {
                    // 字节数，之后是要写入的数据
                    _data_remaining_cout = _rtx->read();
                }
                else {
                    // 06 功能码，写入一个寄存器，两字节
                    _data_remaining_cout = 2;
                    count_or_value_l = 1;
                    // 回退到uint16 数据之前
                    _rtx->rewind();
                    _rtx->read();
                    _rtx->read();
                    _rtx->read();
                }

                if (count_or_value_l < 1) {
                    _respond_exception(RESPONSE_ERROR_INVALID_COUNT);
                    return;
                }

                // 改写操作的响应帧由返回值自动生成
                flag = on_write(addr, count_or_value_l);
            }

            if (flag == 0) {
                flag = RESPONSE_ERROR_DEVICE_ERROR;  // 如果on_read 或 on_write 没有返回异常码，就认为是设备错误
            }

            if (flag < 0) {
                // 如果启用了异常响应，就发出响应，否则只计数
                ++_exception_counter;
                _respond_exception(flag);
            }
            else {
                // 否则是正常响应
                ++_ok_counter;

                if (func != static_cast<uint8_t>(function_code::read_holding_registers)) {
                    // 对于06 功能码，响应是请求的寄存器地址和值，
                    // 对于16 功能码，响应是请求的寄存器地址和数量，
                    _rtx->begin_tx(_rtx->rx_frame_address());
                    send_data(func);
                    send_data(addr);
                    send_data(count_or_value);
                    _rtx->end_tx();

                    // 处理其他需要延后执行的操作
                    after_write();
                }
            }
        }

        /**
         * @brief 响应读取寄存器的请求
         *
         * 具体的响应内容必须用begin_response 等函数构造。
         *
         * @param address
         * @param len
         * @return int_fast8_t
         */
        virtual int_fast8_t on_read(uint16_t address, uint8_t count) = 0;

        /**
         * @brief 响应写入单个或多个寄存器的请求
         *
         * 响应内容由返回值决定，不需要在函数内构造。
         *
         * @param address
         * @param len
         * @return int_fast8_t
         */
        virtual int_fast8_t on_write(uint16_t address, uint8_t count) = 0;


        /**
         * @brief 只在on_write 返回成功时调用，执行在请求完成后才能做的操作，比如修改从机波特率。
         * 
         */
        virtual void after_write() {}


        uint8_t requested_address() const {
            return _rtx->rx_frame_address();
        }

        template <typename T>
        T read_property() {
            // 一次最大可以读取一个八字节数据
            static_assert(sizeof(T) == 8 || sizeof(T) == 4 || sizeof(T) == 2);

            if (_data_remaining_cout < sizeof(T)) {
                return 0;
            }

            // 读取多字节数据时，默认数据是大端序
            hide::any_type_to_uint_converter<T> vv;
            vv.dd = 0;

            for (auto i = sizeof(T); i > 0; --i) {
                vv.dd <<= 8;
                vv.dd += static_cast<uint8_t>(_rtx->read());
            }

            _data_remaining_cout -= sizeof(T);
            return vv.d;
        }

        bool empty() {
            return _data_remaining_cout == 0;
        }

        /**
         * @brief 开始构造并发送03 功能码的响应帧
         *
         * 响应数据必须不间断连续发送，中间不能出现延时或耗时操作。
         * 可以提前将数据准备好，再开始发送。
         *
         * @param len 要发送的字节数，实际数据与长度不匹配时不会报错
         *
         */
        void begin_response(uint8_t len) {
            _rtx->begin_tx(requested_address());
            _rtx->tx(3);
            _rtx->tx(len);
        }

        void begin_response() {
            begin_response(_requested_count * 2);
        }

        /**
         * @brief 按Big Endian 发送多字节数据
         *
         * @tparam T
         * @param d
         */
        template <typename T>
        void send_data(T d) {
            hide::send_data(_rtx, d);
        }


        /**
         * @brief 发送一个属性的数据，如果只请求了uint16 长度的数据，就截取低字节
         *
         * 主要是考虑到兼容性，大部分MODBUS 软件都假设一个寄存器地址只对应一个uint16，
         * 如果访问一个地址时，不管请求的数据长度，一股脑把整个属性的数据都发出去，
         * 会破坏与Modbus Poll 之类的软件的兼容性。
         *
         * @tparam T  属性数据类型，宽度必须大于等于uint16
         * @param d
         * @param send_two_bytes 为 true 时只发送两个低字节数据
         */
        template <typename T>
        void send_property(T d, bool send_two_bytes = false) {
            static_assert(sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8);

            if (send_two_bytes) {
                // 必须先把T 转换成整数类型，
                // 否则，如果T 是float，用static_cast 转换，
                // 结果是将浮点转化成整数，而不是把原始二进制数据变成整数

                hide::any_type_to_uint_converter<T> vv{.d = d};
                send_data(static_cast<uint16_t>(vv.dd));
            }
            else {
                send_data<T>(d);
            }
        }

        void end_response() {
            _rtx->end_tx();
        }

        void begin(uint8_t server_address, uint8_t super_address = 0, uint8_t secondary_broadcast = 0) {
            _rtx->end_session();
            _rtx->begin_session(server_address, false);
            _rtx->set_super_address(super_address);
            _rtx->set_secondary_broadcast_address(secondary_broadcast);
        }

        /**
         * @brief 一次响应尚未完成时，需要修改从机地址，只能在响应完成后才能修改。
         *
         * @param new_address
         */
        void begin_later(uint8_t new_address) {
            _pending_address = new_address;
        }

        void enable_exception_response(bool e) {
            _enable_exception_response = e;
        }

        /**
         * @brief 设置自定义运行状态
         *
         * @param status
         */
        void set_run_status(uint8_t status) {
            _run_status = status;
        }

        /**
         * @brief 设置运行状态
         *
         * @param running
         */
        void set_running(bool running) {
            _run_status = running ? 0xff : 0x00;  // 0xff 表示ON，0x00 表示OFF
        }

        /**
         * @brief 设置Server ID，用于提供设备标识
         *
         * @param id
         */
        void set_server_id(uint8_t id) {
            _server_id = id;
        }

        /**
         * @brief 获取Server ID
         *
         * @return uint8_t
         */
        uint8_t server_id() const {
            return _server_id;
        }

        /**
         * @brief 获取当前Server 的状态字
         *
         * @return uint16_t
         */
        uint16_t status() const {
            return _status;
        }

        bool busy() const {
            return _status == 0xFFFF;  // 如果状态字是0xFFFF，表示设备忙
        }

        void set_status_busy(bool is_busy) {
            if (is_busy) {
                _status = 0xFFFF;  // 表示设备忙
            }
            else {
                _status = 0x0000;  // 表示无事发生
            }
        }

        void set_status(uint16_t status) {
            _status = status;
        }

        /**
         * @brief 获取当前Server 的成功响应计数
         *
         * @return uint16_t
         */
        uint16_t ok_counter() const {
            return _ok_counter;
        }

        /**
         * @brief 获取当前Server 的异常响应计数
         *
         * @return uint16_t
         */
        uint16_t exception_counter() const {
            return _exception_counter;
        }

        /**
         * @brief 获取收发器接收帧计数
         *
         * @return uint16_t
         */
        uint16_t rx_counter() const {
            return _rtx->rx_frame_count();
        }

        /**
         * @brief 获取收发器接收丢失帧计数
         *
         * 这个计数不一定能代表什么问题，如果计数增大的很快，可能说明总线环境比较差，
         * 也可能是主机时序和从机不匹配。
         *
         * @return uint16_t
         */
        uint16_t rx_miss_counter() const {
            return _rtx->rx_miss_count();
        }

        /**
         * @brief 获取当前Server 的最大寄存器地址
         *
         * @return uint16_t
         */
        uint16_t max_address() const {
            return _max_address;
        }

        bool always_flush() const {
            return _always_flush;
        }

        void enable_always_flush(bool flush) {
            _always_flush = flush;
        }

        void enable_busy_exception(bool enable) {
            _enable_busy_exception = enable;
        }

        bool locked() const {
            return _locked;
        }
    };

}  // namespace modpash
