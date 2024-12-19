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

void AP_BattMonitor_MAV::read()
{
    AP_Battery_MAV *batt = AP::Battery_MAV();
    if (batt == nullptr) {
        return;
    }
    uint8_t instance = _params._serial_number;
    if (instance > MAX_BATTERIES){
        return;
    }

    struct BatteryState batt_state;
    batt->get_state(batt_state,instance);

    _state.voltage = batt_state.voltage;
    _state.current_amps = batt_state.current;
    _state.consumed_mah = batt_state.consumed_mah;
}

#endif // AP_BATTERY_MAV_ENABLED
