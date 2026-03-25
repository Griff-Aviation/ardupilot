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

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_MAV_ENABLED

class AP_BattMonitor_MAV : public AP_BattMonitor_Backend {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_BattMonitor_MAV(AP_BattMonitor &mon,
                       AP_BattMonitor::BattMonitor_State &mon_state,
                       AP_BattMonitor_Params &params);

    void read() override;
    AP_BattMonitor::Failsafe update_failsafes() override;

    bool handle_battery_status(const mavlink_message_t &msg, const mavlink_battery_status_t &battery_status);

    bool has_current(void) const override {
        return _state.healthy && _current_valid;
    }

    bool has_temperature(void) const override {
        return _state.healthy && _temperature_valid;
    }

    bool capacity_remaining_pct(uint8_t &percentage) const override;

    int16_t configured_sysid() const { return _sysid.get(); }
    int16_t configured_compid() const { return _compid.get(); }
    int16_t configured_battery_id() const { return _battery_id.get(); }

private:
    bool matches_identity(const mavlink_message_t &msg, const mavlink_battery_status_t &battery_status) const;
    bool identity_is_configured() const;

    AP_Int16 _sysid;
    AP_Int16 _compid;
    AP_Int16 _battery_id;

    HAL_Semaphore _sem;
    float _voltage = 0.0f;
    float _current_amps = 0.0f;
    float _temperature = 0.0f;
    uint8_t _soc = 0;
    uint32_t _last_update_ms = 0;
    bool _have_received = false;
    bool _voltage_valid = false;
    bool _current_valid = false;
    bool _temperature_valid = false;
    bool _soc_valid = false;
};

#endif // AP_BATTERY_MAV_ENABLED
