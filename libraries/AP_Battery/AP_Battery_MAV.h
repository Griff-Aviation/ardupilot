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

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>

#if AP_BATTERY_MAV_ENABLED

#define MAX_BATTERIES 9

struct BatteryState {
    float voltage;     // in Volts
    float current;     // in Amperes
    float consumed_mah; // in mAh
};

class AP_Battery_MAV
{
public:
    AP_Battery_MAV(const AP_Battery_MAV&) = delete;

    static AP_Battery_MAV& get_singleton(void){
        static AP_Battery_MAV singleton;
        return singleton;
    }

    void get_state(BatteryState &_state,uint8_t inst);

    void handle_BATTERY_message(const mavlink_message_t &msg);

private:
    AP_Battery_MAV();

    BatteryState _batteries[MAX_BATTERIES];
    bool _received_new_data[MAX_BATTERIES] = {false};

    // Semaphore for access to shared frontend data
    HAL_Semaphore sem;

};

namespace AP {
    AP_Battery_MAV& Battery_MAV();
};

#endif