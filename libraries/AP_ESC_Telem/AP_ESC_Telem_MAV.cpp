#include "AP_ESC_Telem_MAV.h"
#include <GCS_MAVLink/GCS.h>

AP_ESC_Telem_MAV::AP_ESC_Telem_MAV()
{
}

void AP_ESC_Telem_MAV::handle_ESC_message(const mavlink_message_t &msg, const uint8_t index)
{
    if (index == 0)
    {
        mavlink_esc_telemetry_1_to_4_t packet;
        mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &packet);
        for (size_t i = 0; i < 4; i++)
        {
            update_rpm(i, packet.rpm[i]);
            TelemetryData t {
                .temperature_cdeg = int16_t(packet.temperature[i]*100),
                .voltage = (packet.voltage[i]/100.0f),
                .current = (packet.current[i]/100.0f),
            };
            update_telem_data(i, t,
                AP_ESC_Telem_Backend::TelemetryType::CURRENT
                    | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                    | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);            
        }
    }
    else if (index == 1)
    {
        mavlink_esc_telemetry_5_to_8_t packet;
        mavlink_msg_esc_telemetry_5_to_8_decode(&msg, &packet);
        for (size_t i = 0; i < 4; i++)
        {
            update_rpm(i+4, packet.rpm[i]);
            TelemetryData t {
                .temperature_cdeg = int16_t(packet.temperature[i]*100),
                .voltage = (packet.voltage[i]/100.0f),
                .current = (packet.current[i]/100.0f),
            };
            update_telem_data(i+4, t,
                AP_ESC_Telem_Backend::TelemetryType::CURRENT
                    | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                    | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
        }
    }
}

namespace AP {
    AP_ESC_Telem_MAV& ESC_Telem_MAV()
    {
        return AP_ESC_Telem_MAV::get_singleton();
    }
};