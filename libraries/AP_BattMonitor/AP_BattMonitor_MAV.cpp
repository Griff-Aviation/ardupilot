/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "AP_BattMonitor_config.h"

#if AP_BATTERY_MAV_ENABLED

#include "AP_BattMonitor_MAV.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AP_BattMonitor_MAV::var_info[] = {
    // @Param: MAV_SYS
    // @DisplayName: MAVLink battery source system id
    // @Description: Exact MAVLink system id this battery monitor accepts BATTERY_STATUS messages from. Set to -1 to leave the backend unconfigured.
    // @Range: -1 255
    // @User: Advanced
    AP_GROUPINFO("MAV_SYS", 31, AP_BattMonitor_MAV, _sysid, -1),

    // @Param: MAV_CMP
    // @DisplayName: MAVLink battery source component id
    // @Description: Exact MAVLink component id this battery monitor accepts BATTERY_STATUS messages from. Set to -1 to leave the backend unconfigured.
    // @Range: -1 255
    // @User: Advanced
    AP_GROUPINFO("MAV_CMP", 32, AP_BattMonitor_MAV, _compid, -1),

    // @Param: MAV_ID
    // @DisplayName: MAVLink battery source battery id
    // @Description: Exact BATTERY_STATUS id this battery monitor accepts. Set to -1 to leave the backend unconfigured.
    // @Range: -1 255
    // @User: Advanced
    AP_GROUPINFO("MAV_ID", 33, AP_BattMonitor_MAV, _battery_id, -1),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_BattMonitor_MAV::AP_BattMonitor_MAV(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
    _state.healthy = false;
}

bool AP_BattMonitor_MAV::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!_soc_valid || !_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

bool AP_BattMonitor_MAV::identity_is_configured() const
{
    return _sysid >= 0 && _compid >= 0 && _battery_id >= 0;
}

bool AP_BattMonitor_MAV::matches_identity(const mavlink_message_t &msg, const mavlink_battery_status_t &battery_status) const
{
    if (!identity_is_configured()) {
        return false;
    }

    return msg.sysid == uint8_t(_sysid.get()) &&
           msg.compid == uint8_t(_compid.get()) &&
           battery_status.id == uint8_t(_battery_id.get());
}

bool AP_BattMonitor_MAV::handle_battery_status(const mavlink_message_t &msg, const mavlink_battery_status_t &battery_status)
{
    if (!matches_identity(msg, battery_status)) {
        return false;
    }

    WITH_SEMAPHORE(_sem);

    _have_received = true;
    _last_update_ms = AP_HAL::millis();

    _voltage_valid = battery_status.voltages[0] != UINT16_MAX;
    _current_valid = battery_status.current_battery != -1;
    _temperature_valid = battery_status.temperature != INT16_MAX;
    _soc_valid = battery_status.battery_remaining != -1;

    if (_voltage_valid) {
        _voltage = battery_status.voltages[0] * 0.001f;
    }
    if (_current_valid) {
        _current_amps = battery_status.current_battery * 0.01f;
    }
    if (_temperature_valid) {
        _temperature = battery_status.temperature * 0.01f;
    }
    if (_soc_valid) {
        _soc = battery_status.battery_remaining;
    }

    return true;
}

void AP_BattMonitor_MAV::read()
{
    uint32_t last_update_ms = 0;
    bool have_received = false;
    bool voltage_valid = false;
    bool current_valid = false;
    bool temperature_valid = false;
    bool soc_valid = false;
    float voltage = 0.0f;
    float current_amps = 0.0f;
    float temperature = 0.0f;
    uint8_t soc = 0;

    {
        WITH_SEMAPHORE(_sem);
        last_update_ms = _last_update_ms;
        have_received = _have_received;
        voltage_valid = _voltage_valid;
        current_valid = _current_valid;
        temperature_valid = _temperature_valid;
        soc_valid = _soc_valid;
        voltage = _voltage;
        current_amps = _current_amps;
        temperature = _temperature;
        soc = _soc;
    }

    if (!have_received) {
        _state.healthy = false;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const bool fresh = (now_ms - last_update_ms) <= AP_BATT_MONITOR_TIMEOUT;
    _state.healthy = fresh;

    if (!fresh) {
        WITH_SEMAPHORE(_sem);
        _current_valid = false;
        _temperature_valid = false;
        _soc_valid = false;
        return;
    }

    if (voltage_valid) {
        _state.voltage = voltage;
    }
    if (current_valid) {
        _state.current_amps = current_amps;
    }
    if (temperature_valid) {
        _state.temperature = temperature;
        _state.temperature_time = now_ms;
    }
    if (soc_valid) {
        _soc = soc;
    }

    _state.last_time_micros = AP_HAL::micros();
}

AP_BattMonitor::Failsafe AP_BattMonitor_MAV::update_failsafes()
{
    if (!_state.healthy) {
        return AP_BattMonitor::Failsafe::Unhealthy;
    }
    return AP_BattMonitor_Backend::update_failsafes();
}

#endif // AP_BATTERY_MAV_ENABLED
