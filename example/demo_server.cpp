// Copyright (c) 2025 刻BITTER
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <Arduino.h>
#include <stdlib.h>

// scheduler_basic
#include <scheduler_basic.hpp>
#include <scheduler_tick.hpp>

using TimeSource = scheduler_basic::ArduinoMsSource;
using TimeType = typename TimeSource::TimeType;

// modpash
#include "rtu/non_block_serial.hpp"
#include "adu/demo_basic_property_server_arduino_serial.hpp"


modpash::NonBlockSerialFrameHandler mp{TimeSource{}, Serial}; 
modpash::DemoServer mp_server{mp};

void setup() {
    Serial.begin(115200);
    mp.set_frame_break_timeout_by_baudrate(115200);
    
    // 初始化服务器
    mp_server.begin(0x97);
}

void loop() {
    // modpash::request_type rt;
    // if ((rt = mp_server.request_available()) !=  modpash::request_type::none) {
    //     mp_server.respond(rt);

    //     mp.clear_serial();
    // }
    mp_server.listen();
}