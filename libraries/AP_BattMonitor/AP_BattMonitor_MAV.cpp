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
#include <AP_Battery/AP_Battery_MAV.h>
#include <GCS_MAVLink/GCS.h>

uint8_t foo = 0;

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_MAV::capacity_remaining_pct(uint8_t &percentage) const
{
    /*
    if (foo == 0){
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Battery soc: %d", _soc);
        foo = 10;
    }
    foo--;
    */
   
    // the monitor must have current readings in order to estimate consumed_mah and be healthy
    if (!has_current() || !_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

void AP_BattMonitor_MAV::read()
{
            
    uint8_t instance = _params._serial_number;
    if (instance > MAX_BATTERIES){
        return;
    }

    struct BatteryState batt_state;
    AP::Battery_MAV().get_state(batt_state,instance);

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Battery Voltage: %5.3f", batt_state.voltage);
    _state.voltage = batt_state.voltage;
    _state.current_amps = batt_state.current;
    _state.temperature = batt_state.temperature;
    _soc = batt_state.state_of_charge;
    _state.healthy = true;

}

#endif // AP_BATTERY_MAV_ENABLED
