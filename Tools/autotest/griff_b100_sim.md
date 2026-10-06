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
the engine/generator block, the HybridStatus state machine, and the two MAVLink
`TUNNEL` payloads the board uses to carry Freja's telemetry.

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

Three sources of truth, and they are not the same maturity:

| Family | Spec | Status |
| --- | --- | --- |
| B100 payload/power board | `G135-B100-2022/docs/MavlinkNamedValueFloatFields.md` | shipping firmware |
| Freja hybrid over `TUNNEL` | `G135-B100-2022/docs/FREJA_MAVLINK_TUNNEL_V1.md` | shipping firmware |
| Hybrid / EMU state machine | `Griff_Hybrid_Telemetry_Draft/GRIFF-HYB-{ICD,SM,VER}-001` | Draft 0.1 |

The middle row is how hybrid data actually leaves the board, so it is what the
simulator publishes by default. The `HYBSTAT`/`HYBREASON`/`ENGRPM` NAMED_VALUE
block from the bottom row is *not* in B100 firmware and never was; it is kept
behind `--hybrid-transport` for the GCS code that still reads it:

| `--hybrid-transport` | Publishes |
| --- | --- |
| `tunnel` (default) | `TUNNEL` 0x8001 + 0x8002 at 5 Hz |
| `nval` | the Draft 0.1 NAMED_VALUE hybrid block at 10 Hz |
| `both` | both, for migrating a GCS from one to the other |

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

## Hybrid data over MAVLink TUNNEL (FREJA_MAVLINK_TUNNEL_V1)

B100 reads Freja's telemetry off CAN and republishes it on UART7 as two private
`TUNNEL` payloads every 200 ms, broadcast to `0/0` from sysid 1 / `MAV_COMP_ID_USER1`
(25). Broadcast is what makes an unchanged ArduPilot forward them to every link
rather than consuming them, and it stands until the CU's MAVLink address is
assigned. Multibyte fields are unsigned **big-endian**; every byte past the
declared length is zero.

| Type | Length | Carries |
| --- | --- | --- |
| `0x8001` | 22 | HybridStatus: status, reason, reserve, criteria and latch flags |
| `0x8002` | 29 | Operator: bus, generator, SOC, pack voltages, rpm, fuel, temperatures |

Byte 0 is the format version (`1`) and byte 1 is the **bridge state** — B100's own
view of the CAN link, in its own namespace. It never replaces a Freja ReasonCode:

| State | Meaning | Trigger | What the payload carries |
| --- | --- | --- | --- |
| 0 | a cycle was accepted within 500 ms | nominal | Freja's real status and readings |
| 1 | no cycle has ever been accepted | `inject emu-missing` | status 10, reason 0, age `0xFFFF`, sentinels |
| 2 | the last accepted cycle aged out | `inject emu-stale`, `gx-fault` | status 10, reason 0, real age, sentinels |

A **rejected** frame never refreshes the sample, so an integrity or source-id
failure (`inject can-integrity`, `inject src-invalid`) ages out to state 2 exactly
as a silent bus does — it gets no state of its own. States 1 and 2 publish INVALID
rather than the last plausible reading, which is the entire point of the byte.

Unavailability is per field, not per message: when a validity bit is clear the
value is written as `0xFF` / `0xFFFF`, never as a measurement of zero. `inject
op-unavail` clears the bits for fuel 2, battery 3/4 and the generator-controller
temperature; `inject soc-estimated` marks SOC estimated rather than measured.

The CU is still responsible for its own missing-message timeout: bridge state
cannot report a message that never arrives. `inject mav-stale` stops publication
outright to exercise exactly that.

To read the payloads by eye, type `tunnel` at the console — it hex-dumps both as
they go on the wire, so a capture can be checked against the byte tables in
`FREJA_MAVLINK_TUNNEL_V1.md`:

    TUNNEL type=0x8001 len=22  01 00 01 01 00 40 00 01 2C 00 01 00 0F 07 01 00 ...
    TUNNEL type=0x8002 len=29  01 00 01 FF FF FF FF 00 03 E8 6D 6C 0D FF FF 64 ...

`status` prints the bridge state, the accepted-cycle age and the rejected-frame
count alongside the EMU status, because the two can legitimately disagree: a
fresh bridge (0) can carry `HYBSTAT=10` when Freja itself declares invalid
(`inject emu-invalid`), and that is a different fault from a dead bridge.

## HYBSTAT — hybrid status (GRIFF-HYB-ICD-001 s4)

Carried in `TUNNEL` 0x8001 byte 3, and as the `HYBSTAT` NAMED_VALUE under
`--hybrid-transport nval|both`.

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

Carried in `TUNNEL` 0x8001 byte 4. B100's frame validator accepts reason `0..12`,
so Freja's own enumeration is wider than the Draft 0.1 set below; 8..12 are
unnamed here because no document in either repo names them yet.

Codes 2, 3, 4, 5 and 7 are bridge faults. Over `TUNNEL` they appear as a nonzero
bridge state with reason **0**, not as the reason code — only `emu-invalid`
(reason 1) reaches the CU as a reason, because only it is Freja's own verdict.
The reason codes below are what the NAMED_VALUE block reports.

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
`invalid`, `freja-bridge`, `op-unavail`, `stale-link`, `all-states`,
`b100-faults`.

`freja-bridge` walks the `TUNNEL` bridge state through 1 → 0 → 2 and then through
the rejected-frame path, dumping the payloads at each step. `op-unavail` shows
fields going unavailable rather than reading zero.

They are scriptable too — stdin is read line by line:

    echo "scenario gen-loss" | python3 Tools/autotest/griff_b100_sim.py -v
    python3 Tools/autotest/griff_b100_sim.py --script "scenario all-states"
