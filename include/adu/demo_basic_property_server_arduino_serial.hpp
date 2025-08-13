#pragma once

#include <Arduino.h>

// #include <scheduler_tick.hpp>

#include "basic_property_server.hpp"
#include "rtu/non_block_serial.hpp"

namespace modpash {

    template <typename ModpashTransceiverType>
    class DemoServer : public BasicPropertyServer<ModpashTransceiverType> {
        static_assert(sizeof(float) == 4, "Float size must be 4 bytes");

       private:
        union {
            struct {
                uint16_t value_0;
                uint16_t value_1;
                uint32_t value_2;
                float value_3;
            } v{0};

            uint16_t d[6];  // 两个uint16 加一个uint32_t、float == 6 个 uint16
        } _value;

       public:
        DemoServer(ModpashTransceiverType &rtx) noexcept :
            BasicPropertyServer<ModpashTransceiverType>(rtx, 3) {
            // 给三个属性分配个初始值
            _value.v.value_0 = 0xaabb;
            _value.v.value_1 = 0xccdd;
            _value.v.value_2 = 0xaabbccdd;
            _value.v.value_3 = 3.14f;
        }

        virtual int_fast8_t on_read(uint16_t address, uint8_t count) override {
            // DEBUG
            // Serial.printf("Addr= %d, ", address);
            // Serial.printf("Count= %d\n", count);

            if (count > (sizeof(_value.d) / 2)) {
                return RESPONSE_ERROR_INVALID_VALUE;
            }

            // 异常响应必须在begin_response 之前全部处理掉，
            // 否则会导致帧结构错误
            this->begin_response();

            switch (address) {
                case 0:
                    for (uint_fast8_t i = 0; i < count; ++i) {
                        this->send_property(_value.d[i]);
                    }
                    break;

                case 1:
                    this->send_property(_value.v.value_1);
                    break;

                case 2:
                    this->send_property(_value.v.value_2, count == 1);
                    break;

                case 3:
                    this->send_property(_value.v.value_3, count == 1);
                    break;
            }

            this->end_response();
            return RESPONSE_TYPE_OK;
        }

        virtual int_fast8_t on_write(uint16_t address, uint8_t count) override {
            // 处理写入请求时，响应帧是根据返回值生成的，所以异常响应可以随便在中间哪里处理
            switch (address) {
                case 0:
                    if (count > (sizeof(_value.d) / 2)) {
                        return RESPONSE_ERROR_INVALID_VALUE;
                    }

                    for (uint_fast8_t i = 0; i < count; ++i) {
                        _value.d[i] = this->template read_property<uint16_t>();
                    }
                    break;

                case 1:
                    if (count != 1) {
                        return RESPONSE_ERROR_INVALID_VALUE;
                    }

                    _value.v.value_1 = this->template read_property<uint16_t>();
                    break;

                case 2:
                    if (count != 2) {
                        return RESPONSE_ERROR_INVALID_VALUE;
                    }

                    _value.v.value_2 = this->template read_property<uint32_t>();
                    break;

                case 3:
                    if (count != 2) {
                        return RESPONSE_ERROR_INVALID_VALUE;
                    }

                    _value.v.value_3 = this->template read_property<float>();
                    break;
            }


            return RESPONSE_TYPE_OK;
        }
    };

}  // namespace modpash