#include "AP_ESC_Telem_MAV.h"

#if HAL_WITH_ESC_TELEM

AP_ESC_Telem_MAV::AP_ESC_Telem_MAV()
{
}

bool AP_ESC_Telem_MAV::identity_is_configured() const
{
    return AP::esc_telem().get_mavlink_source_system() >= 0 &&
           AP::esc_telem().get_mavlink_source_component() >= 0;
}

bool AP_ESC_Telem_MAV::matches_identity(const mavlink_message_t &msg) const
{
    if (!identity_is_configured()) {
        return false;
    }

    return msg.sysid == uint8_t(AP::esc_telem().get_mavlink_source_system()) &&
           msg.compid == uint8_t(AP::esc_telem().get_mavlink_source_component());
}

float AP_ESC_Telem_MAV::scaled_rpm(uint16_t rpm) const
{
    return rpm * AP::esc_telem().get_mavlink_rpm_scale();
}

void AP_ESC_Telem_MAV::handle_decoded_packet(const mavlink_esc_telemetry_1_to_4_t &packet, uint8_t base_index)
{
    for (uint8_t i = 0; i < 4; i++) {
        const uint8_t esc_index = base_index + i;
        TelemetryData t {
            .temperature_cdeg = int16_t(packet.temperature[i] * 100),
            .voltage = packet.voltage[i] * 0.01f,
            .current = packet.current[i] * 0.01f,
            .consumption_mah = float(packet.totalcurrent[i]),
        };
        update_telem_data(esc_index, t,
            AP_ESC_Telem_Backend::TelemetryType::CURRENT |
            AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
            AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
            AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
        update_rpm(esc_index, scaled_rpm(packet.rpm[i]));
    }
}

void AP_ESC_Telem_MAV::handle_message(const mavlink_message_t &msg)
{
    if (!matches_identity(msg)) {
        return;
    }

    // Handle the full current ESC telemetry family because the packet layout is
    // uniform across 1..32 and the frontend already supports up to 32 ESCs.
    mavlink_esc_telemetry_1_to_4_t packet {};
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4:
        mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &packet);
        handle_decoded_packet(packet, 0);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8:
        mavlink_msg_esc_telemetry_5_to_8_decode(&msg, (mavlink_esc_telemetry_5_to_8_t *)&packet);
        handle_decoded_packet(packet, 4);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12:
        mavlink_msg_esc_telemetry_9_to_12_decode(&msg, (mavlink_esc_telemetry_9_to_12_t *)&packet);
        handle_decoded_packet(packet, 8);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16:
        mavlink_msg_esc_telemetry_13_to_16_decode(&msg, (mavlink_esc_telemetry_13_to_16_t *)&packet);
        handle_decoded_packet(packet, 12);
        break;
#if ESC_TELEM_MAX_ESCS > 16
    case MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20:
        mavlink_msg_esc_telemetry_17_to_20_decode(&msg, (mavlink_esc_telemetry_17_to_20_t *)&packet);
        handle_decoded_packet(packet, 16);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24:
        mavlink_msg_esc_telemetry_21_to_24_decode(&msg, (mavlink_esc_telemetry_21_to_24_t *)&packet);
        handle_decoded_packet(packet, 20);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28:
        mavlink_msg_esc_telemetry_25_to_28_decode(&msg, (mavlink_esc_telemetry_25_to_28_t *)&packet);
        handle_decoded_packet(packet, 24);
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32:
        mavlink_msg_esc_telemetry_29_to_32_decode(&msg, (mavlink_esc_telemetry_29_to_32_t *)&packet);
        handle_decoded_packet(packet, 28);
        break;
#endif
    default:
        break;
    }
}

namespace AP {
    AP_ESC_Telem_MAV& ESC_Telem_MAV()
    {
        return AP_ESC_Telem_MAV::get_singleton();
    }
};

#endif // HAL_WITH_ESC_TELEM
