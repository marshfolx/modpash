#pragma once

#include <stdint.h>

namespace modpash {

    struct DummyPinType {
        void set(bool level) {}
    };


    enum class address_type {
        none = 0,
        normal = 1,
        super,
        
        broadcast,
        secondary_broadcast,
    };

}  // namespace modpash