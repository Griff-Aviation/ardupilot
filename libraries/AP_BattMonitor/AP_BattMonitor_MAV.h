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
#include <GCS_MAVLink/GCS_MAVLink.h>

#if AP_BATTERY_MAV_ENABLED

class AP_BattMonitor_MAV : public AP_BattMonitor_Backend {
public:

    // Constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    // update state
    void read() override;

    // handle MAVLink battery status messages
    void handle_BATTERY_message(const mavlink_message_t &msg);

    bool has_current(void) const override {
        return true;
    }

    bool has_temperature(void) const override {
        return true;
    }

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

private:
    uint8_t _soc = 0;
};

#endif // AP_BATTERY_MAV_ENABLED
