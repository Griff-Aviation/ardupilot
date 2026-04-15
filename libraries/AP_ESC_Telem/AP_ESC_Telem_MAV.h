#pragma once

#include "AP_ESC_Telem.h"
#include "AP_ESC_Telem_Backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>

#if HAL_WITH_ESC_TELEM

class AP_ESC_Telem_MAV : public AP_ESC_Telem_Backend {
public:
    AP_ESC_Telem_MAV(const AP_ESC_Telem_MAV&) = delete;

    static AP_ESC_Telem_MAV& get_singleton(void) {
        static AP_ESC_Telem_MAV _singleton;
        return _singleton;
    }

    void handle_message(const mavlink_message_t &msg);

private:
    AP_ESC_Telem_MAV();

    bool identity_is_configured() const;
    bool matches_identity(const mavlink_message_t &msg) const;
    float scaled_rpm(uint16_t rpm) const;
    void handle_decoded_packet(const mavlink_esc_telemetry_1_to_4_t &packet, uint8_t base_index);

};

namespace AP {
    AP_ESC_Telem_MAV& ESC_Telem_MAV();
};

#endif // HAL_WITH_ESC_TELEM
