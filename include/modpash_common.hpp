#pragma once

#ifndef _MAKE__OBJECT__NOT__COPYABLE

    #define _MAKE__OBJECT__NOT__COPYABLE(Type) \
        Type(const Type&) = delete;            \
        Type(const Type&&) = delete;           \
        Type& operator=(const Type&) = delete; \
        Type& operator=(const Type&&) = delete;

#endif


namespace modpash {

    // * 1:  读取线圈输出状态     REQ 5 REP N
    // * 2:  读取线圈输入状态     REQ 5 REP N
    // * 3:  读取保持寄存器       REQ 5 REP N
    // * 4:  读取输入寄存器       REQ 5 REP N
    // * 5:  写入单个线圈         REQ 5 REP 5
    // * 6:  写单个保持寄存器     REQ 5 REP 5
    // * 8:  诊断                 REQ 5 REP 5
    // * 11: 获取通信事件计数     REQ 1 REP 5
    // * 12: 获取通信事件记录     REQ 1 REP N
    // * 15: 写多个线圈           REQ N REP 5
    // * 16: 写多个保持寄存器     REQ N REP 5
    // * 17: 获取从站ID           REQ 1 REP N
    enum class function_code {
        read_coil_output = 1,
        read_coil_input = 2,
        read_holding_registers = 3,
        read_input_registers = 4,
        write_coil = 5,
        write_register = 6,

        diagnose = 8,

        get_event_counter = 11,
        get_event_log = 12,

        write_multiple_coils = 15,
        write_multiple_registers = 16,

        report_device_id = 17,
    };


    enum class error_level {
        none = 0,
        ok = 1,

        invalid_function,  //
        sum_mismatch,      // 校验和不匹配
        frame_incomplete,  // 缺失校验和

        waiting,      // 正在等待帧间隔，或者正在等待从机响应，且并没有超时
        no_data,      // 没有接收到任何数据
        no_frame,     // 没有检测到帧头
        rx_timeout,   // 读取超时
        rx_overflow,  // rx 缓冲区溢出

        response_timeout,         // 从机响应超时
        response_mismatch,        // 从机响应不匹配主机的请求
        response_count_mismatch,  // 从机响应的寄存器数量与主机请求的数量不匹配
        broadcast_no_response,    // 向广播地址发送的请求不会收到响应

        data_insufficient,  // 获取的数据不足，无法完成操作
    };


    constexpr const char* error_level_text(error_level e) {
        switch (e) {
            case error_level::none:
                return "none";

            case error_level::ok:
                return "ok";

            case error_level::invalid_function:
                return "invalid_function";

            case error_level::sum_mismatch:
                return "sum_mismatch";

            case error_level::frame_incomplete:
                return "frame_incomplete";

            case error_level::waiting:
                return "waiting";

            case error_level::no_data:
                return "no_data";

            case error_level::no_frame:
                return "no_frame";

            case error_level::rx_timeout:
                return "rx_timeout";

            case error_level::rx_overflow:
                return "rx_overflow";

            case error_level::response_timeout:
                return "response_timeout";

            case error_level::response_mismatch:
                return "response_mismatch";

            case error_level::response_count_mismatch:
                return "response_count_mismatch";

            case error_level::broadcast_no_response:
                return "broadcast_no_response";

            case error_level::data_insufficient:
                return "data_insufficient";

            default:
                return "unknown";
        }
    }
}  // namespace modpash