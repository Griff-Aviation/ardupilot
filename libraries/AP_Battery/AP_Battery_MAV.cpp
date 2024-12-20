#include "AP_Battery_MAV.h"


#if AP_BATTERY_MAV_ENABLED

#include <GCS_MAVLink/GCS.h>

//AP_Battery_MAV AP_Battery_MAV::singleton;

AP_Battery_MAV::AP_Battery_MAV()
{
}

void AP_Battery_MAV::get_state(BatteryState &_state, uint8_t inst)
{
    WITH_SEMAPHORE(sem);
    _state = _batteries[inst];
}

void AP_Battery_MAV::handle_BATTERY_message(const mavlink_message_t &msg)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Mavlink Battery status handle");

    mavlink_battery_status_t battery_status;
    mavlink_msg_battery_status_decode(&msg, &battery_status);

    int battery_index = battery_status.id;
    if (battery_index < 0 || battery_index >= MAX_BATTERIES) {
        return;
    }

    _batteries[battery_index].voltage = battery_status.voltages[0] / 1000.0f; // Convert mV to V
    _batteries[battery_index].current = battery_status.current_battery / 100.0f; // Convert cA to A
    _batteries[battery_index].consumed_mah = battery_status.battery_remaining * 10; // Assume percentage to mAh

    _received_new_data[battery_index] = true;
}

namespace AP {
    AP_Battery_MAV& Battery_MAV()
    {
        return AP_Battery_MAV::get_singleton();
    }
};

#endif