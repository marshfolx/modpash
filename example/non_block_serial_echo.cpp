// Copyright (c) 2025 刻BITTER
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

// scheduler_basic
#include <scheduler_basic.hpp>
#include <scheduler_tick.hpp>

using TimeSource = scheduler_basic::ArduinoMsSource;
using TimeType = typename TimeSource::TimeType;

// modpash
#include "rtu/non_block_serial.hpp"


modpash::NonBlockSerialFrameHandler mp{TimeSource{}, Serial}; 


void setup() {
    Serial.begin(115200);
    mp.set_frame_break_timeout_by_baudrate(115200);
    mp.begin_session(0x97, true);
}


void loop() {
    if (mp.rx_available()) {
        if (mp.rx()) {
            mp.tx_echo();
            mp.rx_clear();
        }
        else {
            Serial.print("ERR = ");
            Serial.println(mp.last_error_text());
        }
    }
}