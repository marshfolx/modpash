#pragma once

#include <stdint.h>

namespace modpash {

    constexpr int8_t RESPONSE_ERROR_READ_ONLY = -8;  // 请求写入只读寄存器
    constexpr int8_t RESPONSE_ERROR_INVALID_COUNT = -7;  // 请求的寄存器数量不合法
    constexpr int8_t RESPONSE_ERROR_BUSY = -6;  // 从机忙，不能响应请求
    constexpr int8_t RESPONSE_ERROR_ACKNOWLEDGE = -5;
    constexpr int8_t RESPONSE_ERROR_DEVICE_ERROR = -4;
    constexpr int8_t RESPONSE_ERROR_INVALID_VALUE = -3;
    constexpr int8_t RESPONSE_ERROR_INVALID_ADDRESS = -2;
    constexpr int8_t RESPONSE_ERROR_INVALID_FUNCTION = -1;

    constexpr int8_t RESPONSE_ERROR_USER = -16;  // 用户可以使用小于等于这个编号的错误码表示自定义错误

    constexpr int8_t RESPONSE_TYPE_NONE = 0;
    constexpr int8_t RESPONSE_TYPE_OK = 1;


    enum class request_type {
        none = 0,
        read,   // 读取保持寄存器
        write,  // 写入保持寄存器

        check,  // 诊断指令，包括7、8、11、12、17

        report_id,  // 获取从站ID
        read_counter, // 获取通信事件计数

        invalid_function,
    };

    
    namespace hide {

        template <uint8_t Size>
        struct any_type_size_to_uint {};


        template <>
        struct any_type_size_to_uint<8> {
            using type = uint64_t;
        };


        template <>
        struct any_type_size_to_uint<4> {
            using type = uint32_t;
        };


        template <>
        struct any_type_size_to_uint<2> {
            using type = uint16_t;
        };


        template <>
        struct any_type_size_to_uint<1> {
            using type = uint8_t;
        };


        template <typename T>
        struct any_type_to_uint {
            using type = typename any_type_size_to_uint<sizeof(T)>::type;
        };

        template <typename T>
        using any_type_to_uint_type = typename any_type_to_uint<T>::type;

        // 使用union 来实现类型转换，避免strict-aliasing 规则的警告
        // 这个union 可以将任意类型的值转换为对应大小的整数类型，或者反过来。        
        template <typename T>
        union any_type_to_uint_converter {
            T d;
            any_type_to_uint_type<T> dd;
        };


        template <typename TransceiverType, typename T>
        inline void send_data(TransceiverType *rtx, T v) {
            // 一次最大可以发送一个八字节数据
            static_assert(sizeof(T) == 8 || sizeof(T) == 4 || sizeof(T) == 2 || sizeof(T) == 1);

            // 必须先用reinterpret_cast 把T 转换成整数类型，
            // 否则，如果T 是float，用static_cast 转换，
            // 结果是将浮点转化成整数，而不是把原始二进制数据变成整数
            //
            // 换成用union 实现转换，因为reinterpret_cast 会触发编译器警告，违反了strict-aliasing 规则。
            hide::any_type_to_uint_converter<T> vv{.d = v};

            if constexpr (sizeof(T) > 4) {
                uint64_t ddd = static_cast<uint64_t>(vv.dd);
                rtx->tx(static_cast<uint8_t>(ddd >> 56));
                rtx->tx(static_cast<uint8_t>(ddd >> 48));
                rtx->tx(static_cast<uint8_t>(ddd >> 40));
                rtx->tx(static_cast<uint8_t>(ddd >> 32));
            }

            if constexpr (sizeof(T) > 2) {
                uint32_t ddd = static_cast<uint32_t>(vv.dd);
                rtx->tx(static_cast<uint8_t>(ddd >> 24));
                rtx->tx(static_cast<uint8_t>(ddd >> 16));
            }

            if constexpr (sizeof(T) > 1) {
                uint16_t ddd = static_cast<uint16_t>(vv.dd);
                rtx->tx(static_cast<uint8_t>(ddd >> 8));
            }

            rtx->tx(static_cast<uint8_t>(vv.dd));
        }

        template <typename TransceiverType, typename T>
        inline T read_data(TransceiverType *rtx) {
            // 一次最大可以读取一个八字节数据
            static_assert(sizeof(T) == 8 || sizeof(T) == 4 || sizeof(T) == 2 || sizeof(T) == 1);

            hide::any_type_to_uint_converter<T> vv;
            vv.dd = 0;

            for (auto i = sizeof(T); i > 0; --i) {
                vv.dd <<= 8;
                vv.dd += static_cast<uint8_t>(rtx->read());
            }

            return vv.d;
        }

    }  // namespace hide

}  // namespace modpash