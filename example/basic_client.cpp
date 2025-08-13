// Copyright (c) 2025 刻BITTER
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.


// 配合另一个开发板从机使用，也可以用Modbus Slave 模拟从机
// 从机需要有3 个保持寄存器


#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>


// scheduler_basic
#include <scheduler_basic.hpp>
#include <scheduler_tick.hpp>

using TimeSource = scheduler_basic::ArduinoMsSource;
using TimeType = typename TimeSource::TimeType;

// oled_basic_v2
#include "oled_backend.hpp"
#include "oled_basic_font.hpp"
#include "oled_basic_v2.hpp"
#include "oled_ssd13xx.hpp"

namespace ob = oled_basic;

// modpash
#include "rtu/non_block_serial.hpp"
#include "adu/basic_client.hpp"


/////////////////////// TOP LEVEL GLOBALS /////////////////////////

// U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

scheduler_basic::DelayCallback3<TimeSource, 10> dcb;

ob::BackendWireSSD13xx oled_backend;
constexpr auto &OLED_DISPLAY = ob::SSD1306_12864_WIRE;
ob::MonoBasic oled{oled_backend, OLED_DISPLAY, ob::font::ascii_8x16_no_lower_case};

modpash::NonBlockSerialFrameHandler mp{TimeSource{}, Serial};
modpash::BasicClient mp_client{mp};



///////////////////////  MAIN    ////////////////////////////


TimeType task_blink() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // 切换LED 状态
    return 800;
}


void setup() {
    Wire.begin();
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    oled_backend.init(true);
    oled_backend.init_display(OLED_DISPLAY);

    oled.clear_screen();

    mp_client.begin(0x97);
    mp_client.set_response_timeout(1000);  // 设置响应超时时间为1000毫秒

    dcb.add_task(task_blink, 0);
}


void display_flag(int flag) {
    oled.set_cursor(10, 6);

    if(flag < 0) {
        if(mp_client.last_error() == modpash::error_level::response_timeout) {
            oled.put_str("TIMEOUT ");
        }
        else if(mp_client.last_error() == modpash::error_level::response_mismatch) {
            oled.put_str("MISMATCH");
        }
        else {
            oled.put_str("ERROR   ");
        }
    }
    else {
        oled.put_str("OK      ");
    }
}

int wait_flag() {
    int flag;
    while((flag = mp_client.response_available()) == 0) {
        dcb.tick();  // 处理延时任务，避免阻塞
    }
    return flag;
}

uint16_t counter = 0;

uint32_t list[] = {0xaabbccdd, 0xbbccddee, 0xccddeeff, 0x11223344, 0x55667788, 0x99aabbcc, 0xddeeff00};
uint8_t index = 0;

constexpr size_t LIST_SIZE = sizeof(list) / sizeof(list[0]);

void inc_index() {
    ++index;
    if(index >= LIST_SIZE) {
        index = 0;
    }
}


void loop() {
    dcb.tick();

    // 写入计数器
    oled.set_cursor(10, 0);
    oled.put_str("CNT ");

    auto start_at = millis();
    mp_client.request_write_u16(0, counter);
    ++counter;

    int flag = wait_flag();
    auto elapsed = millis() - start_at;

    oled.set_num_base(ob::number_base::n10);
    oled.put_unsigned(counter);
    oled.set_cursor(80, 0);
    oled.put_str("     ");
    oled.set_cursor(80, 0);
    oled.put_unsigned(elapsed);

    display_flag(flag);

    // 写入u32
    uint32_t d = list[index];
    oled.set_cursor(10, 2);
    oled.put_str("U32 ");
    oled.set_num_base(ob::number_base::n16);
    oled.put_unsigned(d);

    mp_client.request_write_u32(1, list[index]);
    inc_index();
    flag = wait_flag();
    display_flag(flag);

    // 读取寄存器2
    mp_client.request_read(2, 1);
    flag = wait_flag();
    display_flag(flag);
    if(flag > 0) {
        mp_client.check_response();
        uint16_t value = mp_client.read_u16();
        oled.set_cursor(10, 4);
        oled.put_str("GET ");
        oled.set_num_base(ob::number_base::n16);
        oled.put_unsigned(value);

        if(value == static_cast<uint16_t>(d)) {
            oled.put_str(" OK");
        }
        else {
            oled.put_str(" ERR ");
        }
    }
    else {
        oled.set_cursor(10, 4);
        oled.put_str("GET ERR ");
    }  

    delay(200);
}
