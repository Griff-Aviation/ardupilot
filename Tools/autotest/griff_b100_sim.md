# Griff states in SITL: what they are and how to trigger them

Reference for `griff_b100_sim.py`, which plays the B100 bridge board (and a
modelled EMU) alongside SITL. Run the sim in its own terminal and type the
trigger commands into it; `help` lists them and `scenario list` the timelines.

## Running it

Pick a Griff frame and everything is wired up for you:

    sim_vehicle.py -v ArduCopter -f Griff-Hybrid --console --map
    sim_vehicle.py -v ArduCopter -f Griff30 --console --map

sim_vehicle.py gives the autopilot a SERIAL5 on TCP 5765 (+10 per instance),
starts this simulator against it in its own window, and picks the airframe
profile from the frame. `default_params/griff*.parm` sets `SERIAL5_PROTOCOL 2`,
so no manual param set or restart is needed. `--no-griff-b100` suppresses the
simulator; `--griff-b100-args` passes extra arguments through.

Griff30 is all-electric and publishes the B100 catalog only. Griff-Hybrid adds
the engine/generator block and the HybridStatus state machine.

On Griff-Hybrid, `MAV_CMD 6200 param1=1` (main power ON) runs the engine up
through `READY` and `STARTING` to `NORMAL`. That trigger is a simulator
convenience: the hybrid drafts define no engine-start command, and
GRIFF-HYB-DDR-001 has `READY` merely *permit* a start commanded elsewhere.

To drive it by hand instead, put it behind the autopilot yourself:

    term 1:  sim_vehicle.py -v ArduCopter --console --map -A "--serial5 tcp:5765"
             # once, then restart SITL:  param set SERIAL5_PROTOCOL 2
    term 2:  python3 Tools/autotest/griff_b100_sim.py --connect tcp:127.0.0.1:5765 -v

The autopilot learns a route to sysid 1 / compid 25 from the sim's first packet,
then forwards Griff commands out to it and relays its NAMED_VALUE telemetry back
to every GCS. Verified end to end: `6200 param1=1` aimed at 1/25 comes back
`MAV_RESULT_ACCEPTED` from 1/25, and 41 B100 fields reach a GCS on 5760.

The GCS must address **1/25**. Aim a Griff command at the autopilot's own
component and the autopilot answers it instead:

    ACK cmd=6200 result=3 (MAV_RESULT_UNSUPPORTED) from 1/1

The same thing happens to a correctly addressed command when no component 1/25
has ever been seen: `MAVLink_routing.cpp` rule 1e makes the flight controller
process anything for its own system id whose target component it cannot route,
and ArduPilot has no handler for 6200. An UNSUPPORTED ack from 1/1 therefore
means "the B100 is not on this link", not "the command is wrong".

In MAVProxy: `set target_component 25`, then `long 6200 1`.

Two sources of truth, and they are not the same maturity:

| Family | Spec | Status |
| --- | --- | --- |
| B100 payload/power board | `G135-B100-2022/docs/MavlinkNamedValueFloatFields.md` | shipping firmware |
| Hybrid / EMU | `Griff_Hybrid_Telemetry_Draft/GRIFF-HYB-{ICD,SM,VER}-001` | Draft 0.1 |

## Debugging ACKs when GGC says nothing came back

GGC files a command under the component it addressed, but matches the ACK on the
component that **answers** (`MavCommandQueue.cc` `handleCommandAck`). So an ACK
from the autopilot for a command aimed at the B100 is discarded, and "answered
UNSUPPORTED" and "never answered" look identical in the UI.

Watch the link with the sniffer, on a spare MAVProxy output:

    mavproxy.py --master tcp:127.0.0.1:5760 \
        --out udp:127.0.0.1:14550 \
        --out udp:127.0.0.1:14551
    python3 Tools/autotest/griff_mav_sniff.py --connect udpin:127.0.0.1:14551

Three outcomes, and each means something different:

| Sniffer shows | Meaning |
| --- | --- |
| `ACK ... ACCEPTED from 1/25` | the B100 answered; GGC accepts it |
| `ACK ... UNSUPPORTED from 1/1` | the autopilot answered for it; the B100 is not on the link, and GGC silently discards the ACK |
| nothing at all | the command never reached anything that answers |

To see it from GGC's own side, start it with the command-queue log enabled:

    ./GriffGroundControl --logging:Vehicle.MavCommandQueue

`handleCommandAck Ack not in list` means the ACK arrived and was dropped on the
component mismatch. No line at all means it never arrived.

## HYBSTAT — hybrid status (GRIFF-HYB-ICD-001 s4)

Resolved by the priority table in GRIFF-HYB-SM-001 s2, not by order of arrival:
INVALID > EMERGENCY_POWER_ONLY > GENERATOR_LOST > RESERVE_LOW > LAND_SOON >
DEGRADED_CONTINUE > STARTING > NOT_READY > READY > NORMAL.

| Value | Name | Trigger | Verifies |
| --- | --- | --- | --- |
| 0 | UNKNOWN | `inject boot` | HST-EMU-001 |
| 1 | NOT_READY | `power off` | |
| 2 | READY | `power on` (first 2 s) | HST-EMU-003 |
| 3 | STARTING | `power on` (2-9 s), or reserve < 120 s | HST-EMU-004 |
| 4 | NORMAL | `power on`, reserve >= 120 s | HST-EMU-005 |
| 5 | DEGRADED_CONTINUE | `degrade continue` | no Draft 0.1 stimulus |
| 6 | DEGRADED_LAND_SOON | `degrade soon` | no Draft 0.1 stimulus |
| 7 | GENERATOR_LOST | `arm on` then `inject gen-lost` | HST-EMU-007/009 |
| 8 | FORCED_LANDING_RESERVE_LOW | `reserve 85` with generator online | HST-EMU-012 |
| 9 | EMERGENCY_POWER_ONLY | `inject gen-lost` + `reserve 85` | HST-EMU-013/024 |
| 10 | INVALID | any `inject emu-*` / `can-integrity` / `gx-fault` / `src-invalid` | HST-EMU-014/025 |

Behaviour the triggers exercise, not just the value: generator loss is debounced
500 ms (`inject gen-lost 0.3` must NOT move the status), latches while armed, and
releases on `arm off`; NORMAL is entered only above 120 s reserve and left below
90 s.

## HYBREASON — reason code (GRIFF-HYB-ICD-001 s5)

| Code | Name | Trigger |
| --- | --- | --- |
| 0 | NONE | nominal |
| 1 | EMU_DECLARED_INVALID | `inject emu-invalid` |
| 2 | EMU_STATUS_STALE | `inject emu-stale` |
| 3 | EMU_STATUS_MISSING | `inject emu-missing` |
| 4 | CAN_INTEGRITY_FAIL | `inject can-integrity` |
| 5 | GX_BRIDGE_FAULT | `inject gx-fault` |
| 6 | MAVLINK_STATUS_STALE | `inject mav-stale` (receiver-side; publication simply stops) |
| 7 | SOURCE_ID_INVALID | `inject src-invalid` |

## B100 coded values (shipping firmware)

| Field | Codes | Trigger |
| --- | --- | --- |
| `GFCMDRES` | 0 accepted, 2 denied, 3 unsupported, -1 wrong target | send 6200 `param1=2` (denied), 6241 (unsupported), or aim a 62xx at another compid |
| `GFSTATE` | `100*MAIN + 10*NAV + PLD`, 9 unknown | `power`/`nav`/`payload on|off` |
| `PWRSTAT` | 1 both ON, 0 both OFF, 9 mixed/stale | `inject pwr-mixed` |
| `CUBEHBTO` | 1 = no Cube heartbeat for 5 s | `inject cube-loss` |
| `MVPARSETO` | 1 = bytes arriving, nothing parsing | `inject parse-timeout` |
| `CLAWLDKG`/`CLAWSTAT` | -999 = stale > 5 s | `inject claw-stale` |
| `U7ERR`/`U7RXSTA` | HAL error / 32 READY, 34 BUSY_RX | `inject uart7-err` |
| `CANBO`/`CANEV`/`CANLEC` | bus-off and last-error counters | `can-busoff` |

Any published field can also be pinned outright with `force <FIELD> <value>`, and
`clear all` returns everything to the plant.

## Scenarios

`scenario <name>` plays a timeline with narration, for repeatable demos:

`nominal`, `gen-loss`, `transient`, `latch`, `reserve-low`, `priority`,
`invalid`, `stale-link`, `all-states`, `b100-faults`.

They are scriptable too — stdin is read line by line:

    echo "scenario gen-loss" | python3 Tools/autotest/griff_b100_sim.py -v
    python3 Tools/autotest/griff_b100_sim.py --script "scenario all-states"
