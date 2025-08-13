
#pragma once

// DEBUG
// #include <Arduino.h>
// #include <scheduler_tick.hpp>

#include "modpash_common.hpp"
#include "modpash_crc16.hpp"
#include "modpash_include_config.hpp"
#include "rtu/common.hpp"


namespace modpash {

    template <typename TimeSourceType, typename SerialType, typename PinType = DummyPinType, typename Inspector = RtuFrameInspectorType>
    class NonBlockSerialFrameHandlerBase {
       public:
        // DEBUG
        // using SerialType = HardwareSerial;
        // using TimeSource = scheduler_basic::ArduinoMsSource;
        // using PinType = DummyPinType;

        using TimeSource = TimeSourceType;
        using TimeType = typename TimeSource::TimeType;

       private:
        SerialType &_serial_instance;

        // uint32_t _read_timeout;
        // uint32_t _write_timeout;
        // TimeType _response_timeout = 0;
        // TimeType _response_waiting_start_time = 0;

        // 接收帧时，最小1.5T 帧间隔
        TimeType _rx_frame_break_timeout = RTU_RX_FRAME_BREAK_TIMEOUT;

        // 发送帧时，3.5T 帧间隔
        TimeType _tx_frame_break_timeout = RTU_TX_FRAME_BREAK_TIMEOUT;

        TimeSource _frame_break_timer;

        // data_basic::RingBuffer<uint8_t, RTU_RX_BUFFER_SIZE> _ring;

        error_level _last_error = error_level::none;

        PinType _de_re;  // 对应485 收发器的DE/RE# 引脚，高电平发送，低电平接收

        uint16_t _rx_buffer_len = 0;
        uint16_t _rx_frame_len = 0;
        uint16_t _rx_frame_read_index = 0;
        uint16_t _rx_pdu_start_index = 0;

        uint16_t _last_rx_frame_crc;
        uint16_t _rx_frame_crc;

        uint16_t _rx_counter = 0;       // 成功接收的帧计数
        uint16_t _rx_miss_counter = 0;  // rx_available 返回true，但rx 没有识别到有效帧，此时加一。
                                        // 这个计数不一定能代表什么问题，如果计数增大的很快，可能说明总线环境比较差，
                                        // 也可能是主机时序和从机不匹配。

        uint16_t _tx_current_crc;

        Inspector _inspector;

        uint8_t _rx_buffer[RTU_RX_BUFFER_SIZE];

        uint8_t _addr = 0;
        uint8_t _supper_addr = 0;          // 从机可能具有第二个固定的设备地址，无论设备地址是多少，都可以用这个固定地址访问从机，
                                           // 从而方便生产测试，设置为0 则无效。
        uint8_t _secondary_broadcast = 0;  // MODBUS 规范中使用0 作为广播地址，此处可以设置第二个专门用于某些从机的广播地址，
                                           // 设置为0 则无效。
                                           // 从机不会响应发往广播地址的请求帧。
        uint8_t _requested_addr = 0;       // 上一个发送的请求或响应帧的地址

        bool _is_master = false;
        bool _waiting_for_frame_break = false;


        bool _buffer_push_back(uint8_t d) {
            if (_rx_buffer_len >= sizeof(_rx_buffer)) {
                return false;
            }
            else {
                _rx_buffer[_rx_buffer_len] = d;
                ++_rx_buffer_len;
                return true;
            }
        }


        void _clear_rx_buffer() {
            _rx_buffer_len = 0;
        }


        _MAKE__OBJECT__NOT__COPYABLE(NonBlockSerialFrameHandlerBase)

       public:
        NonBlockSerialFrameHandlerBase(SerialType &s, PinType de_re_485 = DummyPinType()) noexcept :
            _serial_instance(s), _de_re(de_re_485) {
            //
        }


        error_level last_error() const {
            return _last_error;
        }


        const char *last_error_text() const {
            return error_level_text(_last_error);
        }


        void end_session() {
            // TODO: 复位各种内部状态
            clear();
            _waiting_for_frame_break = false;
            _last_error = error_level::none;
        }


        void begin_session(uint8_t address, bool is_master) {
            _addr = address;
            _is_master = is_master;
            set_read_timeout(_rx_frame_break_timeout);
        }


        void begin_session(uint8_t address, bool is_master, SerialType &s) {
            _addr = address;
            _is_master = is_master;
            set_read_timeout(_rx_frame_break_timeout);
            _serial_instance = s;
        }


        void begin_session() {
            begin_session(_addr, _is_master);
        }


        void set_serial(SerialType &s) {
            _serial_instance = s;
        }
        

        void set_read_timeout(TimeType t) {
            _serial_instance.setTimeout(t);
        }


        TimeType rx_frame_break_timeout() const {
            return _rx_frame_break_timeout;
        }


        TimeType tx_frame_break_timeout() const {
            return _tx_frame_break_timeout;
        }


        void set_frame_break_timeout(TimeType rx, TimeType tx) {
            if (rx == 0) {
                _rx_frame_break_timeout = RTU_RX_FRAME_BREAK_TIMEOUT;
            }
            else {
                _rx_frame_break_timeout = rx;
            }

            set_read_timeout(_rx_frame_break_timeout);

            if (tx == 0) {
                _tx_frame_break_timeout = RTU_TX_FRAME_BREAK_TIMEOUT;
            }
            else {
                _tx_frame_break_timeout = tx;
            }
        }


        /**
         * @brief 根据波特率设置时序参数，接收和发送的帧间隔都会被设置为3.5T。
         *
         * @param baud
         */
        void set_frame_break_timeout_by_baudrate(uint32_t baud) {
            if (baud > 19200) {
                _tx_frame_break_timeout = 2;  // 1750us ≈ 2ms
            }
            else {
                auto timeout_us = 38500000UL / baud;  // 1T * 3.5 = T3.5, 1T = 11 bits
                _tx_frame_break_timeout = timeout_us / 1000;
                if ((timeout_us % 1000) >= 500) {
                    // 四舍五入
                    _tx_frame_break_timeout += 1;
                }
            }
            _rx_frame_break_timeout = _tx_frame_break_timeout;
        }


        /**
         * @brief 检查当前是否可能无阻塞的发送帧
         *
         * 发送帧时，期望的工作方式是：如果不能发送，就原地阻塞，或通知上层调用者；一旦开始发送，就无阻塞的把整个帧发送完；
         *
         * 返回1，表示可以发；返回0 表示可以稍等片刻；返回-1，表示无法判断。
         *
         * @return
         */
        int tx_available(uint16_t adu_len = 5) {
            // 软串口一定是阻塞的，没必要判断
#if _MODPASH_USE_SOFT_SERIAL > 0
            return 1;
#endif
            // 如果返回-1，表示serial_instance 不支持这个功能，那就只能直接发了再说
            int n = _serial_instance.availableForWrite();
            if (n < 0) {
                return 1;
            }

            auto pdu_len = adu_len + 3;
            if (static_cast<size_t>(n) >= pdu_len) {
                return 1;
            }
            else {
#ifdef SERIAL_TX_BUFFER_SIZE
                // 如果总的发送缓冲区大小大于pdu_len，就可以稍等
                if (SERIAL_TX_BUFFER_SIZE >= pdu_len) {
                    return 0;
                }
                else {
                    // 否则无论如何都会阻塞，就发了再说
                    return 1;
                }
#else
                // 如果不知道总的发送缓冲区大小，就无法判断
                return -1;

#endif
            }
        }


        /**
         * @brief 前一帧数据是否已发送完毕。
         *
         * 返回0 表示没发完；返回1 表示发完了；返回-1 表示无法判断。
         *
         * 如果使用485 收发器通讯，必须等待一帧发完，然后才能调用rx_available 切换485 收发器的方向。
         *
         * @param wait_until_complete 为true 时，等待串口发完
         * @return int
         */
        int tx_complete(bool wait_until_complete = false) {
            rx_clear();
            if (wait_until_complete) {
                _serial_instance.flush();
                return 1;
            }
            else {
#ifdef SERIAL_TX_BUFFER_SIZE
                int n = _serial_instance.availableForWrite();
                if (n < 0) {
                    return -1;
                }
                else {
                    // 如果知道发送缓冲区的总量，只要空余空间等于总量，表示发完了，
                    // 否则无法判断
                    if (static_cast<size_t>(n) == SERIAL_TX_BUFFER_SIZE) {
                        return 1;
                    }
                }
#else
                return -1;
#endif
            }

            return 0;
        }


        /**
         * @brief 设置485 收发器为输入模式
         *
         */
        void set_transceiver_input() {
            _de_re.set(0);
        }


        /**
         * @brief 设置485 收发器为输出模式
         *
         */
        void set_transceiver_output() {
            _de_re.set(1);
        }


        bool begin_tx(uint8_t addr) {
            if (is_broadcast_address(addr)) {
                return false;
            }

            // 控制485 收发器
            set_transceiver_output();
            _tx_current_crc = 0xffff;
            // 先发送地址
            _requested_addr = addr;
            _tx_current_crc = incremental_crc16(addr, _tx_current_crc);
            _serial_instance.write(addr);
            return true;
        }


        /**
         * @brief 结束一帧数据的构造，添加CRC16 校验码，并发送。
         *
         *  如果使用485 收发器通讯，必须等待一帧发完，然后才能切换收发器的方向。
         *
         * @return true
         * @return false
         */
        bool end_tx() {
            // 如果是广播地址，就不发
            if (is_broadcast_address(_addr)) {
                return false;
            }
            // 发送CRC
            _serial_instance.write(static_cast<uint8_t>(_tx_current_crc >> 8));
            _serial_instance.write(static_cast<uint8_t>(_tx_current_crc));
            rx_clear();
            // TODO: 发送超时或异常
            return true;
        }


        bool tx(uint8_t d) {
            // 如果是广播地址，就不发
            if (is_broadcast_address(_addr)) {
                return false;
            }
            // 再发送ADU 部分
            _tx_current_crc = incremental_crc16(d, _tx_current_crc);
            _serial_instance.write(d);
            return true;
        }


        bool tx(const uint8_t *adu, uint16_t adu_len) {
            // 再发送ADU 部分
            for (uint16_t i = 0; i < adu_len; ++i) {
                uint8_t d = adu[i];
                tx(d);
            }
            return true;
        }


        /**
         * @brief 用当前收到的请求地址发回响应
         *
         * @param adu
         * @param adu_len
         * @return true
         * @return false
         */
        bool tx_response(const uint8_t *adu, uint16_t adu_len) {
            auto addr = rx_frame_address();
            if (addr == 0xff) {
                return false;
            }

            begin_tx(addr);
            tx(addr, adu, adu_len);
            return end_tx();
        }


        /**
         * @brief 向当前目标地址发出请求
         *
         * @param adu
         * @param adu_len
         * @return true
         * @return false
         */
        bool tx_request(const uint8_t *adu, uint16_t adu_len) {
            begin_tx(_addr);
            tx(_addr, adu, adu_len);
            return end_tx();
        }


        /**
         * @brief 将接收到的请求帧原样返回
         *
         * @return true
         * @return false
         */
        bool tx_echo() {
            auto addr = rx_frame_address();
            if (addr == 0xff) {
                return false;
            }

            begin_tx(_addr);
            tx(&_rx_buffer[_rx_pdu_start_index + 1], _rx_frame_len);
            return end_tx();
        }


        /**
         * @brief 当前是否有未接收的串口数据
         *
         * @return true
         * @return false
         */
        bool serial_empty() {
            return _serial_instance.available() > 0;
        }


        /**
         * @brief 清空接收缓冲区
         *
         * 可能有必要设置Serial 的超时时间，让它足够小，比如设置为rx_frame_timeout。
         * 默认的超时时间很长，可能影响通讯实时性。
         *
         */
        void rx_clear() {
            auto i = _serial_instance.available();
            for (; i > 0; --i) {
                _serial_instance.read();
            }
        }


        /**
         * @brief 检查当前是否接收到有效数据
         *
         * 如果检测到疑似帧头，就将之后的所有数据写入缓冲区。
         * 如果接收到了数据，但与帧头不匹配，则将这些数据从缓冲区删除，直到遇到帧头。
         *
         * 使用者必须不停调用rx_available，轮询串口数据。当串口一段时间没有接收到数据后，
         * 表示可能遇到了帧间隔，缓冲区内可能已有一帧数据，此时返回true，用户接着调用rx 解析数据。
         *
         * rx 只将缓冲区内的数据解析完，不会从串口获取新数据。
         * 如果rx 接收到的帧不完整，将其视为损坏的数据，不会接着等待后续数据。
         *
         * 如果返回true，表示有一帧数据正在等待处理。如果返回false，表示其他情况，可以调用last_error 读取详情，
         * 包括：
         *
         * - no_data:     没有接收到数据，也没有在等待数据；
         * - no_frame:    接收到了数据，但不是有效的帧；
         * - waiting:     正在接收数据帧，等待后续数据或帧间隔；
         * - rx_overflow: 正在接收数据，但缓冲区溢出；
         *
         * 如果发生了缓冲区溢出，必须在接收完毕前处理，否则rx_available 会继续等待，在遇到帧间隔后返回true，
         * 并用ok 覆盖掉last_error。
         *
         * 有时不需要处理缓冲区溢出，因为溢出可能是因为沾包，有效数据帧后沾了很多异常数据。此时不必处理溢出，
         * 照常调用rx 提取有效帧。
         *
         * @return true
         * @return false
         */
        bool rx_available() {
            // 将de_re 引脚拉低，切换为接收模式
            set_transceiver_input();
            _last_error = error_level::waiting;
            auto n = static_cast<unsigned>(_serial_instance.available());

            // 将接收到的数据全部送入环形缓冲区。一段时间没收到数据后，
            // 表示可能遇到了帧间隔，之后调用rx 解析数据.
            if (n > 0) {
                // DEBUG
                // Serial.printf("N == %d\n", n);
                if (!_waiting_for_frame_break) {
                    clear();

                    if (n >= RTU_MIN_FRAME_LEN) {
                        // 先查找疑似帧头。对于从机，就是查找自己的地址，主机则要查找目标从机的地址。
                        // 此外，主机还需要查找第二地址，或 supper_address；
                        // 从机除了要查找设备地址和super_address，还要查找两个广播地址
                        for (; n > 0; --n) {
                            uint8_t d = static_cast<uint8_t>(_serial_instance.peek());
                            if (is_address(d)) {
                                // 检测到了正确的地址，将剩余的数据写入缓冲区
                                break;
                            }
                            else {
                                _serial_instance.read();  // 把一字节从缓冲区删除
                            }
                        }

                        if (n > 0) {
                            _waiting_for_frame_break = true;
                            // DEBUG
                            // Serial.println("start waiting");
                        }
                        else {
                            _last_error = error_level::no_frame;
                        }
                    }
                }
                else {
                    // DEBUG
                    // Serial.println("buffer append");
                    for (; n > 0; --n) {
                        uint8_t d = static_cast<uint8_t>(_serial_instance.read());
                        if (!_buffer_push_back(d)) {
                            // DEBUG
                            // Serial.println("Overflow");
                            _last_error = error_level::rx_overflow;
                            return false;
                        }
                    }

                    _frame_break_timer.reset();
                }
            }
            else if (_waiting_for_frame_break) {
                // DEBUG
                // Serial.println("waiting");
                if (_frame_break_timer.diff() > _rx_frame_break_timeout) {
                    _waiting_for_frame_break = false;
                    _last_error = error_level::ok;
                    return true;
                }
            }
            else {
                _last_error = error_level::no_data;
            }

            return false;
        }


        /**
         * @brief 开始读取数据
         *
         * 只有完整接收并解析了一帧数据后，才会返回true。返回false 时，可调用last_error 获取详细信息。
         *
         * 【 识别数据帧 】
         *
         * RTU 协议没有明确的帧头、帧尾，只依靠时序控制。但是很多情况下，控制时序是比较困难的：
         *
         * - 在单片机上，如果不想靠延时实现时序，就只能依赖硬件定时器，这会限制库的可移植性；
         * - 如果依赖Arduino 框架，Serial 设计上并没有提供中断回调API，只能轮询，不便于靠中断控制时序；
         * - 使用平台提供的串口实现是最方便的，但是大部分类似Serial 的实现都难以控制时序；
         * - 如果用在PC 或RTOS 环境，时序控制会更加复杂；
         *
         * 因此更优的方法是非阻塞轮询：串口有数据时将数据全部塞进缓冲区，没数据时开始计时，一段时间后还是没数据，
         * 就认为遇到了帧间隔延时。这种方法把时序控制和协议逻辑解耦，由用户保证轮询的实时性，没有硬件依赖。
         * 虽然极限性能不如硬件时序控制，但对于MODBUS 协议也足够了。
         *
         * 这部分轮询的工作由rx_available 实现。
         *
         * 【 解析数据帧 】
         *
         * 不能保证一次接收到的数据一定就是完整的一帧，可能由于各种情况，导致缓冲区中包含了无效数据:
         *
         * - 有效帧前后有无效数据，这种情况可以处理；
         * - 有效帧中间有无效数据，这种情况无法处理，视为通信错误；
         *
         * 前后有无效数据可能是因为沾包。比如总线上有多个从机时，其他从机与主机的通信也会被本机接收；
         * 或者是因为主机多次重试，第一个请求包通信失败，主机重试，于是第一个包的碎片就可能和第二个包沾一起。
         *
         * 处理沾包的主要原理：从疑似帧头的位置向后读取帧头数据，用AduInspector 获取帧长度，然后计算CRC16 校验值；
         * 用计算的校验值和帧长度结尾处的疑似校验值对比，如果相等，表示正确接收到了一帧，否则向后移动帧头位置，重复尝试，
         * 直到帧头位置移动到缓冲区数据结尾。
         *
         * 【 error level 】
         *
         * 调用rx 得到的error_level 是比较随机的，只能作为参考，因为当一次匹配失败时，rx 会接着向后匹配，
         * 返回的error_level 就是最后一次尝试的结果。
         *
         * 【 重复请求 】
         *
         * 当从机响应超时，主机会重复发送请求，有几种出错的情况：
         *
         * - 从机恰好在主机重复发送请求时响应，这会导致主机没收到响应，而从机也没收到重复请求；
         * - 从机在主机发送重复请求后响应，可能把重复请求当作新的请求，然后再次响应，干扰总线通信；
         *
         * 所以理论上，从机发送响应时，应该先检查串口有没有收到数据；如果有数据，等这一帧响应发送后，
         * 应该清空串口接收缓冲区，这样就能解决第二种情况；第一种情况不可能实时解决，
         * 因为从机无法预测主机重复请求的时机，只能调节主机的超时设置，跟从机的响应速度匹配。
         *
         * @return true
         * @return false 发生了超时等错误，接受数据不完整
         */
        bool rx() {
            // set_transceiver_input();
            _rx_frame_len = 0;
            _last_rx_frame_crc = _rx_frame_crc;
            _rx_frame_crc = 0xffff;

            if (_rx_buffer_len < RTU_MIN_FRAME_LEN) {
                _last_error = error_level::no_data;
            }
            else {
                uint_fast16_t pdu_head_index = 0;
                _last_error = error_level::no_frame;

                for (; pdu_head_index < _rx_buffer_len; ++pdu_head_index) {
                    auto d = _rx_buffer[pdu_head_index];

                    // DEBUG
                    // Serial.printf("RX = %X\n", d);

                    // 第一步，查找疑似帧头。虽然经过rx_available 后已经找到了疑似帧头，
                    // 但是为了处理沾包，要反复循环遍历缓冲区
                    if (!is_address(d)) {
                        continue;
                    }

                    // 第二步，开始计算CRC，包括帧头的设备地址
                    uint16_t crc = 0xffff;
                    crc = incremental_crc16(d, crc);

                    // DEBUG
                    // Serial.printf("ADDRESS CRC = %X\n", crc);

                    // 第三步，从疑似指令码开始，调用Inspector，同时验证CRC
                    _inspector.reset(!_is_master);
                    uint_fast16_t adu_remains_count = -1;  // ADC 剩余长度
                    uint_fast16_t adu_size = 0;

                    for (auto read_index = pdu_head_index + 1; read_index < _rx_buffer_len; ++read_index) {
                        // 有几种退出循环的情况：
                        // 1. inspector 返回 -1，表示没找到有效的ADU 帧头；
                        // 2. 读到了缓冲区末尾；
                        // 3. 比较CRC16 需要缓冲区里还有两字节，但是不够;
                        // 4. 成功找到了有效帧；
                        // 前两种情况的结果都是重新回去找疑似帧头

                        if (adu_remains_count == 0) {
                            // 读完了ADU，应该比较CRC
                            // 如果缓冲区剩下的字节数小于2，就报错
                            if ((_rx_buffer_len - read_index) < 2) {
                                _last_error = error_level::frame_incomplete;
                                break;
                            }
                            else {
                                uint8_t crc_h = _rx_buffer[read_index];
                                uint8_t crc_l = _rx_buffer[read_index + 1];
                                uint16_t frame_crc = (crc_h << 8) + crc_l;

                                // DEBUG
                                // Serial.printf("CHECK CRC = %X\n", frame_crc);
                                // Serial.printf("CRC = %X\n", crc);

                                if (frame_crc != crc) {
                                    _last_error = error_level::sum_mismatch;
                                    break;
                                }
                                else {
                                    // 找到了有效帧
                                    _last_error = error_level::ok;
                                    _last_rx_frame_crc = crc;
                                    // 返回的数据帧应该是PDU 去掉CRC 两字节，
                                    // 因为应用层需要根据请求地址判断响应方式
                                    _rx_frame_len = adu_size;
                                    _rx_frame_read_index = pdu_head_index + 1;
                                    _rx_pdu_start_index = pdu_head_index;
                                    ++_rx_counter;
                                    // 清空缓冲
                                    _clear_rx_buffer();
                                    return true;
                                }
                            }
                        }

                        d = _rx_buffer[read_index];
                        crc = incremental_crc16(d, crc);

                        // DEBUG
                        // Serial.printf("D = %X\n", d);

                        if (adu_size == 0) {
                            auto r = _inspector.feed(d);
                            if (r < 0) {
                                // DEBUG
                                // Serial.printf("INVALID FUNCTION = %X\n", d);
                                _last_error = error_level::invalid_function;
                                break;
                            }
                            else if (r > 0) {
                                // 成功获取ADU 长度，减去已经读了的长度，就是剩余的
                                adu_size = static_cast<uint_fast16_t>(r);
                                // DEBUG
                                // Serial.printf("ADU SIZE = %d\n", adu_size);
                                adu_remains_count = adu_size - (read_index - pdu_head_index - 1);
                            }
                        }

                        --adu_remains_count;
                    }
                }

                ++_rx_miss_counter;
            }

            // 处理完一帧就清空缓冲区，Modbus 是一问一答的形式，缓冲区里应该只有一个有效帧
            clear();
            return false;
        }


        bool rx_buffer_overflow() const {
            return _rx_buffer_len >= max_rx_frame_size();
        }


        void clear() {
            _clear_rx_buffer();
            _rx_frame_len = 0;
        }


        uint8_t address() const {
            return _addr;
        }


        void set_address(uint8_t addr) {
            _addr = addr;
        }


        uint8_t super_address() const {
            return _supper_addr;
        }


        void set_super_address(uint8_t addr) {
            _supper_addr = addr;
        }


        uint8_t secondary_broadcast_address() const {
            return _secondary_broadcast;
        }


        void set_secondary_broadcast_address(uint8_t addr) {
            _secondary_broadcast = addr;
        }


        bool is_secondary_broadcast(uint8_t d) {
            return (d != 0) && (d == _secondary_broadcast);
        }


        bool is_broadcast_address(uint8_t d) {
            return (d == 0) || is_secondary_broadcast(d);
        }


        bool is_supper_address(uint8_t d) {
            return (d != 0) && (d == _supper_addr);
        }


        bool is_address(uint8_t d) {
            if (_is_master) {
                return (d == _addr) || is_supper_address(d);
            }
            else {
                return (d == _addr) || is_supper_address(d) || is_broadcast_address(d);
            }
        }


        bool is_device_address(uint8_t d) {
            return (d == _addr) || is_supper_address(d);
        }


        size_t max_rx_frame_size() const {
            return RTU_MAX_RX_FRAME_SIZE;
        }


        /**
         * @brief 依次读取接收到的数据帧ADU 部分。无数据时返回-1
         *
         * @return int
         */
        int read() {
            auto read_count = _rx_frame_read_index - _rx_pdu_start_index - 1;
            if (read_count >= _rx_frame_len) {
                return -1;
            }
            else {
                auto d = _rx_buffer[_rx_frame_read_index];
                ++_rx_frame_read_index;
                return d;
            }
        }


        void rewind() {
            _rx_frame_read_index = _rx_pdu_start_index + 1;
        }


        uint16_t rx_frame_len() const {
            return _rx_frame_len;
        }


        uint16_t rx_frame_crc() const {
            return _rx_frame_crc;
        }


        uint16_t last_rx_frame_crc() const {
            return _last_rx_frame_crc;
        }


        /**
         * @brief 返回当前接收的帧的地址，若没接收到有效帧，返回0xff
         *
         * @return uint8_t
         */
        uint8_t rx_frame_address() const {
            if (_rx_frame_len > 0) {
                return _rx_buffer[_rx_pdu_start_index];
            }
            else {
                return 0xff;
            }
        }


        /**
         * @brief 返回当前接收的帧的功能码，若没接收到有效帧，返回0x00
         *
         * @return uint8_t
         */
        uint8_t rx_frame_function() const {
            if (_rx_frame_len > 0) {
                return _rx_buffer[_rx_pdu_start_index + 1];
            }
            else {
                return 0x00;
            }
        }


        address_type rx_address_type() const {
            auto addr = rx_frame_address();
            if (addr == 0) {
                return address_type::broadcast;
            }
            else if (is_secondary_broadcast(addr)) {
                return address_type::secondary_broadcast;
            }
            else if (is_supper_address(addr)) {
                return address_type::super;
            }
            else if (is_device_address(addr)) {
                return address_type::normal;
            }
            else {
                return address_type::none;
            }
        }


        uint16_t rx_miss_count() const {
            return _rx_miss_counter;
        }


        uint16_t rx_frame_count() const {
            return _rx_counter;
        }


        uint8_t requested_address() const {
            return _requested_addr;
        }


        void reset_counter() {
            _rx_miss_counter = 0;
            _rx_counter = 0;
        }
    };


    template <typename TimeSource, typename SerialType, typename PinType = DummyPinType>
    class NonBlockSerialFrameHandler : public NonBlockSerialFrameHandlerBase<TimeSource, SerialType, PinType, RtuFrameInspectorType> {
       public:
        /**
         * @brief 为了能自动推导所有模板参数，必须传入一个TimeSource 对象作为placeholder。
         *
         * @param t
         * @param s
         * @param p
         */
        NonBlockSerialFrameHandler(TimeSource t, SerialType &s, PinType p = DummyPinType{}) :
            NonBlockSerialFrameHandlerBase<TimeSource, SerialType, PinType, RtuFrameInspectorType>(s, p) {}

        NonBlockSerialFrameHandler(SerialType &s, PinType p = DummyPinType{}) :
            NonBlockSerialFrameHandlerBase<TimeSource, SerialType, PinType, RtuFrameInspectorType>(s, p) {}
    };
}  // namespace modpash