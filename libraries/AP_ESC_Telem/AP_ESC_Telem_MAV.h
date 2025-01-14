#pragma once

#include "AP_ESC_Telem_Backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ESC_Telem_MAV : public AP_ESC_Telem_Backend {
public:
    AP_ESC_Telem_MAV(const AP_ESC_Telem_MAV&) = delete;

    static AP_ESC_Telem_MAV& get_singleton(void) {
        static AP_ESC_Telem_MAV _singleton;
        return _singleton;
    }
    
    void update();

    void handle_ESC_message(const mavlink_message_t &msg, const uint8_t index);

private:
    AP_ESC_Telem_MAV();

};

namespace AP {
    AP_ESC_Telem_MAV& ESC_Telem_MAV();
};