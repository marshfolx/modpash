#pragma once

#include <stdint.h>
#include <stdlib.h>

#include "rtu/inspector.hpp"

#if __has_include("modpash_app_config.hpp")

    #include "modpash_app_config.hpp"

#else

namespace modpash {

    #ifdef _MODPASH_RTU_MAX_RX_FRAME_SIZE
    // 可接收的最大帧长度
    constexpr size_t RTU_MAX_RX_FRAME_SIZE = _MODPASH_RTU_MAX_RX_FRAME_SIZE;
    #else
    constexpr size_t RTU_MAX_RX_FRAME_SIZE = 64;
    #endif

    #ifdef _MODPASH_RTU_RESPONSE_TIMEOUT
    // 响应超时时间，单位毫秒
    constexpr uint16_t RTU_RESPONSE_TIMEOUT = _MODPASH_RTU_RESPONSE_TIMEOUT;
    #else
    constexpr uint16_t RTU_RESPONSE_TIMEOUT = 1000;  // 默认1000毫秒
    #endif

    using RtuFrameInspectorType = DefaultAduInspector;

}  // namespace modpash

#endif

namespace modpash {

    constexpr size_t RTU_MIN_FRAME_LEN = 4;                                         // 最小的RTU 帧长度, 包括地址、指令码、两字节CRC16 校验码
    constexpr size_t RTU_MAX_REQUEST_FRAME_SIZE = RTU_MIN_FRAME_LEN + 4 + 1 + 255;  // MODBUS 最大请求帧长度
    constexpr size_t RTU_MAX_RESPONSE_FRAME_SIZE = RTU_MIN_FRAME_LEN + 1 + 255;     // MODBUS 最大响应帧长度

    constexpr size_t RTU_PRACTICAL_FRAME_LEN = 8;  // 最常用的03 和06 指令的最小帧长度是8

    static_assert(RTU_MAX_RX_FRAME_SIZE > RTU_PRACTICAL_FRAME_LEN);

    constexpr size_t RTU_RX_BUFFER_SIZE = RTU_MAX_RX_FRAME_SIZE;

    // 收发器不处理发送帧时的3.5T 延时，如果有必要，应该由上层应用代码实现。
    // 如果是主机，发送帧后应该等待从机响应，不需要专门等待3.5T；
    // 只有主机连续发送广播请求时才要注意延时。
    // 如果是从机，应该在接收请求后等待3.5T 再发送响应。但是非阻塞轮询的工作方式无法确定帧具体是什么时间接收的，
    // 不如在接收时统一等待3.5T，从而保证响应一定在3.5T 后发送。
    constexpr size_t RTU_TX_FRAME_BREAK_TIMEOUT = 5;  // 对应波特率大于9600，3.5T > 3.9 毫秒
    constexpr size_t RTU_RX_FRAME_BREAK_TIMEOUT = RTU_TX_FRAME_BREAK_TIMEOUT;

#ifndef _MODPASH_USE_SOFT_SERIAL
    #define _MODPASH_USE_SOFT_SERIAL (0)
#endif

}  // namespace modpash