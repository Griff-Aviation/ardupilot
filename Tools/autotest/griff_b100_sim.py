#!/usr/bin/env python3
"""Simulate the Griff B100 payload/power module as MAVLink component 25.

Runs as a separate process alongside SITL -- it touches neither sim_vehicle.py
nor the firmware. SITL flies the airframe; this plays the part of the B100
bridge board that SITL has no model of, so GriffGroundControl sees a vehicle
that behaves like a real hybrid Griff.

Behaviour is taken from the real firmware (G135-B100-2022):
  Project_files/GA_mavlink.cpp                -- command dispatch and ACK rules
  docs/MavlinkNamedValueFloatFields.md        -- field catalog, rates, packing
  docs/GriffControlsCommandDebugAndAck.md     -- debug/ACK contract

which means, faithfully:
  * NAMED_VALUE telemetry from sysid 1 / compid 25, normal set at 1 Hz, claw at
    5 Hz, CAN diagnostics every 10 s, version every 60 s
  * the verbose field set stays silent until Griff command 6250 param1=1
  * Griff commands are the range 6200..6251; 6200/6210/6220 are explicit
    set-state on param1, NOT toggles, and DENY any other param1
  * COMMAND_ACK is sent only for commands in the Griff range -- anything else
    aimed at compid 25 is answered with silence
  * a Griff command whose target is not sysid 1 / compid 25 is counted in the
    debug block (GFCMDTGT=0, GFCMDRES=-1) and never acked
  * COMMAND_INT is ignored outright, as handle_command_int() is a no-op
  * PWRSTAT is battery transistor feedback, not the commanded state, so it
    passes through 9 (mixed/stale) before settling to 1 or 0
  * GFSTATE packs as 100*MAIN + 10*NAV + PLD with 9 for unknown (999 at boot)

The engine and hybrid fields (ENGRPM, ENGTEMP, GENTEMP, BATTRSV, HYBSTAT,
HYBREASON, HYBFLTIME) are NOT in the B100 firmware -- per
GriffCustomTelemetry.json they come from the separate Griff_Hybrid_Telemetry
draft, so they are modelled here as a plausible EMU/generator plant rather than
copied from a reference implementation. Use --no-hybrid to omit them.

Usage -- pick the topology that matches what you are testing:

  1. Behind the autopilot, the way the real B100 sits on the Cube's UART7.
     The autopilot learns a route to 1/25, forwards Griff commands out to us
     and relays our telemetry back, so ANY GCS reaches us and the autopilot
     stops answering 6200 itself:

        term 1:  sim_vehicle.py -v ArduCopter --console --map \
                     -A "--serial5 tcp:5765"
                 (once, then restart: param set SERIAL5_PROTOCOL 2)
        term 2:  python3 Tools/autotest/griff_b100_sim.py \
                     --connect tcp:127.0.0.1:5765 -v

  2. Straight at QGC, for a quick telemetry-only demo. We only ever send, so
     QGC's UDP link adds us as a session target on our first packet and its
     commands come back to us -- but nothing else can reach us, and the
     autopilot answers Griff commands itself with MAV_RESULT_UNSUPPORTED:

        term 1:  sim_vehicle.py -v ArduCopter --console --map
        term 2:  python3 Tools/autotest/griff_b100_sim.py -v

Either way the GCS must address 1/25. A Griff command sent to the autopilot's
own component is answered by the autopilot, not by us: MAVLink_routing.cpp
rule 1e has it process anything for its system id whose target component it
has never seen, and no handler for 6200 means MAV_RESULT_UNSUPPORTED.

Modes:
    --mode sim    (default) coupled plant, real command semantics
    --mode sweep  ramp every ranged value through its range and cycle every
                  coded value, to check that dials and enum readouts render
    --faults      sim mode: lose the generator once running, to exercise the
                  degraded HYBSTAT/HYBREASON states

While it runs, the terminal is a console: type "help" for the command list.
Two ways to reach a state, because they prove different things --

    inject <stimulus>   feed the underlying condition (generator lost, EMU
                        stale, CAN integrity fail) and let the priority table
                        in GRIFF-HYB-SM-001 s2 pick the status, which is what
                        the HST-EMU cases in GRIFF-HYB-VER-001 actually verify
    force <FIELD> <v>   pin a published value outright, for the states Draft
                        0.1 defines but gives no stimulus for (DEGRADED_*,
                        UNKNOWN) and for showing a GCS every enum in turn

    scenario <name>     play a canned timeline of the above; "scenario list"

Commands are read line by line, so stdin can also be piped or scripted:

    echo "scenario gen-loss" | python3 Tools/autotest/griff_b100_sim.py -v

The default target (14550) is the port QGC listens on for SITL. This process
only sends -- it never binds a port -- so it does not fight MAVProxy's link,
and QGC's UDP link adds it as a session target on our first packet, which is
how the COMMAND_LONGs reach us.
"""

import argparse
import math
import os
import random
import select
import sys
import time

# COMMAND_ACK's target_system/target_component/progress fields are MAVLink 2
# extensions, and the real board acks with them populated, so ask pymavlink for
# the v2 dialect before it builds the MAVLink class.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402  (must follow the MAVLINK20 setenv)
from pymavlink.mavutil import mavlink as mav1  # noqa: E402

# ------------------------------------------------------------ Griff MAV_CMDs

CMD_MAIN_POWER = 6200
CMD_NAV_LIGHTS = 6210
CMD_PAYLOAD = 6220
CMD_GIMBAL_HORIZON = 6230
CMD_GIMBAL_NADIR = 6231
CMD_GIMBAL_OBLIQUE = 6232
CMD_CAN_RESET = 6240
CMD_ETH_RESET = 6241        # in range, unimplemented -> UNSUPPORTED
CMD_VERBOSE_NVAL = 6250
CMD_UART7_RESTART_TEST = 6251

GRIFF_CMD_MIN = CMD_MAIN_POWER
GRIFF_CMD_MAX = CMD_UART7_RESTART_TEST
GIMBAL_CMDS = (CMD_GIMBAL_HORIZON, CMD_GIMBAL_NADIR, CMD_GIMBAL_OBLIQUE)

B100_SYSID = 1
B100_COMPID = 25            # MAV_COMP_ID_USER1
CUBE_COMPID = 1

UART7_TEST_MAX_COUNT = 100
UART7_TEST_MIN_INTERVAL_MS = 500
UART7_TEST_MAX_INTERVAL_MS = 60000
UART7_TEST_DEFAULT_INTERVAL_MS = 2000

# firmware packs GXVER as major*1000000 + minor*10000 + patch*100 + build
GX_VERSION = 1 * 1000000 + 6 * 10000 + 1 * 100 + 0   # 1.6.1+0 -> 1060100

STALE_S = 5.0               # Cube heartbeat / parser / claw staleness threshold

# ------------------------------------------------------------- hybrid states

HYB_UNKNOWN, HYB_NOT_READY, HYB_READY, HYB_STARTING, HYB_NORMAL = 0, 1, 2, 3, 4
HYB_DEGRADED, HYB_LAND_SOON, HYB_GEN_LOST, HYB_RSV_LOW, HYB_EMERGENCY = 5, 6, 7, 8, 9
HYB_INVALID = 10

RSN_NONE, RSN_EMU_INVALID, RSN_EMU_STALE, RSN_EMU_MISSING = 0, 1, 2, 3
RSN_CAN_FAIL, RSN_GX_FAULT, RSN_MAV_STALE, RSN_SRC_INVALID = 4, 5, 6, 7

HYB_NAMES = {
    HYB_UNKNOWN: "UNKNOWN", HYB_NOT_READY: "NOT_READY", HYB_READY: "READY",
    HYB_STARTING: "STARTING", HYB_NORMAL: "NORMAL",
    HYB_DEGRADED: "DEGRADED_CONTINUE", HYB_LAND_SOON: "DEGRADED_LAND_SOON",
    HYB_GEN_LOST: "GENERATOR_LOST", HYB_RSV_LOW: "FORCED_LANDING_RESERVE_LOW",
    HYB_EMERGENCY: "EMERGENCY_POWER_ONLY", HYB_INVALID: "INVALID",
}
RSN_NAMES = {
    RSN_NONE: "NONE", RSN_EMU_INVALID: "EMU_DECLARED_INVALID",
    RSN_EMU_STALE: "EMU_STATUS_STALE", RSN_EMU_MISSING: "EMU_STATUS_MISSING",
    RSN_CAN_FAIL: "CAN_INTEGRITY_FAIL", RSN_GX_FAULT: "GX_BRIDGE_FAULT",
    RSN_MAV_STALE: "MAVLINK_STATUS_STALE", RSN_SRC_INVALID: "SOURCE_ID_INVALID",
}

# GRIFF-HYB-VER-001 s2 assumptions and GRIFF-HYB-SM-001 s4 thresholds
GEN_LOSS_DEBOUNCE_S = 0.5   # loss must persist this long before it may publish
RESERVE_LOW_S = 90.0        # below this -> RESERVE_LOW, or EMERGENCY if gen lost
RESERVE_NORMAL_S = 120.0    # needed to enter NORMAL out of STARTING
# GRIFF-HYB-ICD-001 s6 specifies 10 Hz nominal on all three segments, including
# autopilot -> CU, with a 1.0 s staleness timeout at the CU. That is the spec
# default here. It costs 7 msg/s per field on the GCS link, so --hybrid-rate
# lowers it when the extra traffic is not wanted.
HYB_PUBLISH_HZ = 10.0

AMBIENT_C = 15.0
IDLE_RPM = 2400.0
CRUISE_RPM = 6200.0
ENG_RUN_C = 96.0
GEN_RUN_C = 74.0
RESERVE_FULL_S = 300.0      # BATTRSV is seconds of reserve, not capacity
FLT_TIME_NOMINAL_S = 180.0  # HYBFLTIME is "Forced Landing Time"
CLAW_LOAD_KG = 12.4
STALE_SENTINEL = -999.0     # CLAWLDKG / CLAWSTAT stale marker

# --------------------------------------------------------------- field sets
# Grouped by publish schedule, exactly as documented for the real board.

NORMAL_1HZ = (
    "CUBEHBTO", "HBPKTCNT", "CUBEHBCN", "HBID", "HBAUTO",
    "U7RXCNT", "U7RXAGE", "U7BACKLG",
    "MVPKTCNT", "MVMSGID", "MVSTATE", "MVPERR", "MVOVR", "MVPARSETO",
    "U7ERR", "U7RXSTA", "U7TXSTA", "U7HALRC", "U7ARMCNT",
    "U7RSTCNT", "U7TSTCNT", "U7TSTREM",
    "CMDLGCNT", "CMDLGID", "CMDLGTGT",
    "GFCMDCNT", "GFCMDID", "GFCMDRES", "GFSTATE",
    "PWRSTAT", "PDINSTAT",
)
VERBOSE_1HZ = (
    "GFCMDP1", "GFCMDTGT", "GFCMDSYS", "GFCMDCMP", "GFCMDSEQ", "GFCMDAGE",
    "GFMAIN", "GFNAV", "GFPLD", "GFGIMBAL", "GFRESET",
    "U7DRNTO", "MVPKTAGE",
)
CLAW_5HZ = ("CLAWLDKG", "CLAWSTAT")
CAN_10S_NORMAL = ("CANBO",)
CAN_10S_VERBOSE = ("CANERRHI", "CANERRLO", "CANRA", "CANRS", "CANEV", "CANLEC")
VERSION_60S = ("GXVER",)
HYBRID_FIELDS = ("ENGRPM", "ENGTEMP", "GENTEMP", "BATTRSV",
                 "HYBSTAT", "HYBREASON", "HYBFLTIME")

# Sent as NAMED_VALUE_INT; everything else goes as NAMED_VALUE_FLOAT.
# The real board sends all of these as NAMED_VALUE_FLOAT, but QGC's fact group
# handles both and the shipped JSON types many of them "int", so integral
# fields go out as INT here to exercise the 252 path too. --all-float matches
# the firmware byte-for-byte instead.
INT_VALUES = frozenset((
    "CUBEHBTO", "HBPKTCNT", "CUBEHBCN", "HBID", "HBAUTO",
    "U7RXCNT", "U7BACKLG", "U7ERR", "U7RXSTA", "U7TXSTA", "U7HALRC",
    "U7ARMCNT", "U7RSTCNT", "U7TSTCNT", "U7TSTREM", "U7DRNTO",
    "MVPKTCNT", "MVMSGID", "MVSTATE", "MVPERR", "MVOVR", "MVPARSETO",
    "CMDLGCNT", "CMDLGID", "CMDLGTGT",
    "GFCMDCNT", "GFCMDID", "GFCMDRES", "GFCMDTGT", "GFCMDSYS", "GFCMDCMP",
    "GFCMDSEQ", "GFSTATE", "GFMAIN", "GFNAV", "GFPLD", "GFGIMBAL", "GFRESET",
    "PWRSTAT", "PDINSTAT", "CLAWSTAT",
    "CANBO", "CANERRHI", "CANERRLO", "CANRA", "CANRS", "CANEV", "CANLEC",
    "GXVER", "HYBSTAT", "HYBREASON",
))

SWEEP_RANGES = (
    ("ENGRPM", 0.0, 8000.0),
    ("ENGTEMP", -20.0, 150.0),
    ("GENTEMP", -20.0, 150.0),
    ("BATTRSV", 0.0, 300.0),
    ("HYBFLTIME", 0.0, 180.0),
    ("CLAWLDKG", 0.0, 40.0),
)
SWEEP_ENUMS = (
    ("PWRSTAT", (0, 1, 9)),
    ("HYBSTAT", tuple(range(11))),
    ("HYBREASON", tuple(range(8))),
    ("GFCMDRES", (-1, 0, 2, 3)),
    ("GFSTATE", (999, 111, 101, 900, 0)),
    ("GFMAIN", (-1, 0, 1)),
    ("GFNAV", (-1, 0, 1)),
    ("GFPLD", (-1, 0, 1)),
    ("GFGIMBAL", (-1, 6230, 6231, 6232)),
    ("CUBEHBTO", (0, 1)),
    ("CMDLGTGT", (-1, 0, 1)),
    ("PDINSTAT", (0, 1, 10, 11)),
    ("U7RXSTA", (32, 34)),
    ("U7TXSTA", (32, 33)),
    ("U7HALRC", (0, 1, 2, 3)),
    ("CLAWSTAT", (0, 1, -999)),
)


def lag(current, target, dt, tau):
    """First-order approach of current toward target with time constant tau."""
    return current + (target - current) * (1.0 - math.exp(-dt / tau))


def state_digit(latched):
    """GFSTATE digit: 1 ON, 0 OFF, 9 unknown."""
    return 9 if latched < 0 else int(latched)


class B100:
    """State of the simulated payload/power module."""

    def __init__(self, faults=False, fault_after=45.0, fault_duration=60.0,
                 hybrid=True, reserve_s=RESERVE_FULL_S):
        self.t = 0.0
        self.faults = faults
        self.fault_after = fault_after
        self.fault_duration = fault_duration
        self.hybrid = hybrid

        # latched command state; -1 is "unknown", as the board reports before
        # it has seen a command for that function
        self.main = -1
        self.nav = -1
        self.payload = -1
        self.gimbal = -1
        self.reset = -1
        self.verbose = False

        # COMMAND_LONG / Griff command debug block
        self.cmd_long_count = 0
        self.last_cmd_long = 0
        self.last_cmd_target = -1       # -1 before any command is received
        self.griff_cmd_count = 0
        self.last_griff_cmd = 0
        self.last_griff_res = 0
        self.last_griff_p1 = 0.0
        self.last_griff_tgt = 0
        self.last_griff_sys = 0
        self.last_griff_comp = 0
        self.last_griff_seq = 0
        self.last_griff_time = None

        # battery transistor feedback behind PWRSTAT
        self.transistors = None         # None -> stale/unknown -> PWRSTAT 9
        self.transistor_settle_at = None

        # engine / hybrid plant
        self.rpm = 0.0
        self.eng_c = AMBIENT_C
        self.gen_c = AMBIENT_C
        self.reserve_s = reserve_s
        self.flt_time_s = FLT_TIME_NOMINAL_S
        self.hyb = HYB_NOT_READY
        self.reason = RSN_NONE
        self.powered_at = None
        self.normal_entered = False

        # live fault injection: name -> None (until cleared) or expiry time
        self.inj = {}
        self.forced = {}            # field -> pinned published value
        self.degrade = None         # None | "continue" | "land-soon"
        self.gen_off_since = None   # start of the current generator dropout
        self.gen_latched = False    # GENERATOR_LOST latches in flight
        self.fault_fired = False    # legacy --faults one-shot
        self.reserve_hold = False   # stop the generator charging the reserve back
        self.hb_armed = False       # armed bit from the Cube heartbeat
        self.armed_forced = None    # console override for the armed bit
        self.can_lec = 0

        # UART7 / parser / CAN counters
        self.rx_bytes = 0
        self.last_rx = None
        self.mav_pkts = 0
        self.last_msgid = 0
        self.last_pkt = None
        self.hb_pkts = 0
        self.cube_hbs = 0
        self.last_cube_hb = None
        self.last_hb_id = -1
        self.cube_autopilot = -1
        self.uart7_restarts = 0
        self.uart7_test_count = 0
        self.uart7_test_remaining = 0
        self.uart7_test_interval_ms = UART7_TEST_DEFAULT_INTERVAL_MS
        self.uart7_test_next = None
        self.can_bus_off = 0
        self.can_restart_attempts = 0
        self.can_restart_ok = 0
        self.can_last_event = -1

    # -------------------------------------------------------------- injection

    @property
    def armed(self):
        """In flight, as far as the latching rules are concerned."""
        if self.armed_forced is not None:
            return self.armed_forced
        return self.hb_armed

    def set_inj(self, name, duration=None):
        self.inj[name] = None if duration is None else self.t + duration

    def clear_inj(self, name=None):
        if name is None:
            self.inj.clear()
        else:
            self.inj.pop(name, None)

    def injected(self, name):
        if name not in self.inj:
            return False
        expiry = self.inj[name]
        return expiry is None or self.t < expiry

    def _expire_injections(self):
        for name in [n for n, e in self.inj.items() if e is not None and self.t >= e]:
            del self.inj[name]

    # ----------------------------------------------------------- rx / commands

    def note_packet(self, msg):
        """Count a MAVLink packet the way the UART7 parser would."""
        self.rx_bytes = (self.rx_bytes + len(msg.get_msgbuf())) & 0x7FFFFFFF
        self.last_rx = self.t
        self.mav_pkts += 1
        self.last_msgid = msg.get_msgId()
        self.last_pkt = self.t
        if msg.get_type() == "HEARTBEAT":
            self.hb_pkts += 1
            self.last_hb_id = msg.get_srcSystem() * 1000 + msg.get_srcComponent()
            if msg.get_srcSystem() == B100_SYSID and msg.get_srcComponent() == CUBE_COMPID:
                self.cube_hbs += 1
                self.last_cube_hb = self.t
                self.cube_autopilot = msg.autopilot
                # the armed bit decides whether GENERATOR_LOST latches, so take
                # it from SITL rather than guessing
                self.hb_armed = bool(msg.base_mode & mav1.MAV_MODE_FLAG_SAFETY_ARMED)

    def handle_command_long(self, msg, compid):
        """Mirror GA_mavlink.cpp handle_command_long.

        Returns the MAV_RESULT to ack, or None to stay silent.
        """
        cmd = msg.command
        griff = GRIFF_CMD_MIN <= cmd <= GRIFF_CMD_MAX
        target_match = (msg.target_system == B100_SYSID
                        and msg.target_component == compid)

        # update_command_long_rx_debug runs for every COMMAND_LONG parsed
        self.cmd_long_count += 1
        self.last_cmd_long = cmd
        self.last_cmd_target = 1 if target_match else 0

        if not target_match:
            if griff:
                self._note_griff(msg, target_match=0.0, result=-1)
            return None

        if cmd == CMD_MAIN_POWER:
            result = self._cmd_main_power(msg)
        elif cmd == CMD_NAV_LIGHTS:
            result = self._cmd_set_state(msg, "nav")
        elif cmd == CMD_PAYLOAD:
            result = self._cmd_set_state(msg, "payload")
        elif cmd in GIMBAL_CMDS:
            self.gimbal = cmd
            result = mav1.MAV_RESULT_ACCEPTED
        elif cmd == CMD_CAN_RESET:
            # firmware acks 6240 before performing the reset
            self.reset = cmd
            result = mav1.MAV_RESULT_ACCEPTED
            self._note_griff(msg, 1.0, result)
            self.can_restart_attempts += 1
            self.can_restart_ok += 1
            self.can_last_event = 1
            return result
        elif cmd == CMD_VERBOSE_NVAL:
            result = self._cmd_verbose(msg)
        elif cmd == CMD_UART7_RESTART_TEST:
            result = self._cmd_uart7_test(msg)
        else:
            # in the Griff range but not implemented (e.g. 6241 Ethernet reset)
            result = mav1.MAV_RESULT_UNSUPPORTED

        if griff:
            self._note_griff(msg, 1.0, result)
            return result
        # Non-Griff command aimed at us: firmware computes a result but never
        # sends an ACK.
        return None

    def _note_griff(self, msg, target_match, result):
        self.griff_cmd_count += 1
        self.last_griff_cmd = msg.command
        self.last_griff_res = int(result)
        self.last_griff_p1 = msg.param1
        self.last_griff_tgt = int(target_match)
        self.last_griff_sys = msg.get_srcSystem()
        self.last_griff_comp = msg.get_srcComponent()
        self.last_griff_seq = msg.get_seq()
        self.last_griff_time = self.t

    def _cmd_main_power(self, msg):
        if msg.param1 == 1:
            self._set_transistors(True)
            self.main = 1
            return mav1.MAV_RESULT_ACCEPTED
        if msg.param1 == 0:
            self._set_transistors(False)
            self.main = 0
            return mav1.MAV_RESULT_ACCEPTED
        if msg.param1 == 3 and msg.param2 == 0:      # "Power On Freja"
            return mav1.MAV_RESULT_ACCEPTED
        if msg.param1 == 4 and msg.param2 == 0:      # "Precharge Controllers"
            return mav1.MAV_RESULT_ACCEPTED
        return mav1.MAV_RESULT_DENIED

    def _cmd_set_state(self, msg, which):
        """6210 / 6220: explicit set-state on param1, deny anything else."""
        if msg.param1 == 1:
            setattr(self, which, 1)
            return mav1.MAV_RESULT_ACCEPTED
        if msg.param1 == 0:
            setattr(self, which, 0)
            return mav1.MAV_RESULT_ACCEPTED
        return mav1.MAV_RESULT_DENIED

    def _cmd_verbose(self, msg):
        if msg.param1 == 1:
            self.verbose = True
            return mav1.MAV_RESULT_ACCEPTED
        if msg.param1 == 0:
            self.verbose = False
            return mav1.MAV_RESULT_ACCEPTED
        return mav1.MAV_RESULT_DENIED

    def _cmd_uart7_test(self, msg):
        if msg.param1 < 0 or msg.param1 > UART7_TEST_MAX_COUNT:
            return mav1.MAV_RESULT_DENIED
        count = int(msg.param1)
        if count == 0:
            self.uart7_test_remaining = 0
            self.uart7_test_next = None
            return mav1.MAV_RESULT_ACCEPTED
        interval_ms = UART7_TEST_DEFAULT_INTERVAL_MS
        if msg.param2 > 0:
            interval_ms = int(msg.param2 * 1000.0)
        if not (UART7_TEST_MIN_INTERVAL_MS <= interval_ms <= UART7_TEST_MAX_INTERVAL_MS):
            return mav1.MAV_RESULT_DENIED
        self.uart7_test_remaining = count
        self.uart7_test_interval_ms = interval_ms
        self.uart7_test_next = self.t
        return mav1.MAV_RESULT_ACCEPTED

    # ------------------------------------------------------------------ plant

    def _set_transistors(self, on):
        """Commanding power does not move PWRSTAT instantly; the battery CAN
        status goes stale/mixed first, which the board reports as 9."""
        self.transistors = None
        self.transistor_settle_at = self.t + 1.5
        self._pending_transistors = on

    def step(self, dt):
        self.t += dt

        if (self.transistor_settle_at is not None
                and self.t >= self.transistor_settle_at):
            self.transistors = self._pending_transistors
            self.transistor_settle_at = None
            if self.transistors:
                self.powered_at = self.t
            else:
                self.powered_at = None
                self.normal_entered = False

        # command-driven UART7 restart test
        if self.uart7_test_remaining > 0 and self.uart7_test_next is not None:
            if self.t >= self.uart7_test_next:
                self.uart7_test_remaining -= 1
                self.uart7_test_count += 1
                self.uart7_restarts += 1
                self.uart7_test_next = self.t + self.uart7_test_interval_ms / 1000.0

        if self.hybrid:
            self._step_hybrid(dt)

    def _step_hybrid(self, dt):
        """Step the EMU plant, then resolve the published status."""
        self._expire_injections()

        # --faults stays as shorthand for one scheduled generator loss
        if (self.faults and not self.fault_fired and self.powered_at is not None
                and self.t - self.powered_at > self.fault_after):
            self.fault_fired = True
            self.set_inj("gen-lost", self.fault_duration)

        powered = bool(self.transistors)
        gen_lost = self._generator_lost(powered)

        if not powered:
            phase = "off"
            self.rpm = lag(self.rpm, 0.0, dt, 1.5)
            self.eng_c = lag(self.eng_c, AMBIENT_C, dt, 90.0)
            self.gen_c = lag(self.gen_c, AMBIENT_C, dt, 70.0)
            self.flt_time_s = FLT_TIME_NOMINAL_S
            self.normal_entered = False
        elif gen_lost:
            # engine down, flying on the battery reserve; the forced-landing
            # clock is however many seconds of reserve are left
            phase = "gen_lost"
            self.rpm = lag(self.rpm, 0.0, dt, 2.5)
            self.eng_c = lag(self.eng_c, AMBIENT_C, dt, 120.0)
            self.gen_c = lag(self.gen_c, AMBIENT_C, dt, 80.0)
            self.reserve_s = max(0.0, self.reserve_s - dt)
            self.flt_time_s = self.reserve_s
        else:
            since_on = self.t - self.powered_at
            if since_on < 2.0:
                phase, target_rpm = "ready", 0.0
            elif since_on < 9.0:
                phase = "starting"
                target_rpm = IDLE_RPM if since_on < 5.0 else CRUISE_RPM
            else:
                # NORMAL is only entered with reserve >= 120 s (HST-EMU-005);
                # once entered it stands until reserve falls below 90 s, which
                # RESERVE_LOW then covers
                if self.reserve_s >= RESERVE_NORMAL_S:
                    self.normal_entered = True
                phase = "running" if self.normal_entered else "starting"
                target_rpm = (CRUISE_RPM
                              + 140.0 * math.sin(self.t / 7.0)
                              + 60.0 * math.sin(self.t / 2.3))
            self.rpm = max(0.0, lag(self.rpm, target_rpm, dt, 1.2)
                           + random.uniform(-12.0, 12.0))
            load = self.rpm / CRUISE_RPM
            self.eng_c = lag(self.eng_c, AMBIENT_C + (ENG_RUN_C - AMBIENT_C) * load, dt, 25.0)
            self.gen_c = lag(self.gen_c, AMBIENT_C + (GEN_RUN_C - AMBIENT_C) * load, dt, 35.0)
            if self.rpm > IDLE_RPM and not self.reserve_hold:
                self.reserve_s = min(RESERVE_FULL_S, self.reserve_s + 5.0 * dt)
            self.flt_time_s = FLT_TIME_NOMINAL_S

        self.hyb, self.reason = self._resolve_status(phase, gen_lost)

    def _generator_lost(self, powered):
        """Debounced, in-flight-latched generator loss.

        A dropout shorter than GEN_LOSS_DEBOUNCE_S must not move the status
        (HST-EMU-008), and once the loss is confirmed in flight it stays
        latched even if the generator comes back (HST-EMU-009).
        """
        if self.injected("gen-lost"):
            if self.gen_off_since is None:
                self.gen_off_since = self.t
        else:
            self.gen_off_since = None

        confirmed = (self.gen_off_since is not None
                     and self.t - self.gen_off_since >= GEN_LOSS_DEBOUNCE_S)
        if confirmed and self.armed:
            self.gen_latched = True
        if self.gen_latched and not confirmed and not self.armed:
            self.gen_latched = False        # "latched until landing"
        return powered and (confirmed or self.gen_latched)

    def _resolve_status(self, phase, gen_lost):
        """Pick the published status by GRIFF-HYB-SM-001 s2 priority.

        Highest priority first, so that concurrent conditions resolve the way
        HST-EMU-024 and HST-EMU-025 require rather than by order of arrival.
        """
        if self.injected("boot"):
            # EMU boot: outside the priority table, it simply precedes it
            return HYB_UNKNOWN, RSN_NONE

        for name, reason in (("emu-invalid", RSN_EMU_INVALID),
                             ("emu-stale", RSN_EMU_STALE),
                             ("emu-missing", RSN_EMU_MISSING),
                             ("can-integrity", RSN_CAN_FAIL),
                             ("gx-fault", RSN_GX_FAULT),
                             ("src-invalid", RSN_SRC_INVALID)):
            if self.injected(name):
                return HYB_INVALID, reason                  # priority 1

        running = phase in ("starting", "running")
        reserve_low = self.reserve_s < RESERVE_LOW_S
        if gen_lost and reserve_low:
            return HYB_EMERGENCY, RSN_NONE                  # priority 2
        if gen_lost:
            return HYB_GEN_LOST, RSN_NONE                   # priority 3
        if reserve_low and running:
            return HYB_RSV_LOW, RSN_NONE                    # priority 4
        if self.degrade == "land-soon":
            return HYB_LAND_SOON, RSN_NONE                  # priority 5
        if self.degrade == "continue":
            return HYB_DEGRADED, RSN_NONE                   # priority 6
        # NOTE: main power ON runs NOT_READY -> READY -> STARTING -> NORMAL
        # straight through. That is a simulator convenience, not the spec: the
        # hybrid document set defines no engine-start command at all, and
        # GRIFF-HYB-DDR-001 has READY only *permit* a start that some other
        # actor commands. Accepted deliberately so the sim has one trigger.
        if phase == "starting":
            return HYB_STARTING, RSN_NONE                   # priority 7
        if phase == "off":
            return HYB_NOT_READY, RSN_NONE                  # priority 8
        if phase == "ready":
            return HYB_READY, RSN_NONE                      # priority 9
        return HYB_NORMAL, RSN_NONE                         # priority 10

    # -------------------------------------------------------------- telemetry

    def pwrstat(self):
        """1 both transistors ON, 0 both OFF, 9 mixed/fault/unknown/stale."""
        if self.transistors is None:
            return 9
        return 1 if self.transistors else 0

    def values(self):
        cube_ok = (self.last_cube_hb is not None
                   and self.t - self.last_cube_hb < STALE_S
                   and not self.injected("cube-loss"))
        rx_age = -1.0 if self.last_rx is None else self.t - self.last_rx
        pkt_age = -1.0 if self.last_pkt is None else self.t - self.last_pkt
        cmd_age = -1.0 if self.last_griff_time is None else self.t - self.last_griff_time
        parse_timeout = 1 if (self.injected("parse-timeout")
                              or (rx_age >= 0.0 and pkt_age > STALE_S)) else 0
        # PD11 mirrors the payload line (last-change-wins), PD12 idles low
        pd11 = 1 if self.payload == 1 else 0
        claw_live = self.payload == 1
        claw_stale = self.injected("claw-stale")
        uart7_err = self.injected("uart7-err")

        values = {
            # --- Cube link over UART7
            "CUBEHBTO": 0 if cube_ok else 1,
            "HBPKTCNT": self.hb_pkts,
            "CUBEHBCN": self.cube_hbs,
            "HBID": self.last_hb_id,
            "HBAUTO": self.cube_autopilot,
            "U7RXCNT": self.rx_bytes,
            "U7RXAGE": rx_age,
            "U7BACKLG": 0,
            "U7ERR": 8 if uart7_err else 0,
            "U7RXSTA": 32 if uart7_err else (34 if 0.0 <= rx_age < 0.5 else 32),
            "U7TXSTA": 32,
            "U7HALRC": 0,
            "U7ARMCNT": 0,
            "U7RSTCNT": self.uart7_restarts,
            "U7TSTCNT": self.uart7_test_count,
            "U7TSTREM": self.uart7_test_remaining,
            "U7DRNTO": 0,

            # --- MAVLink parser
            "MVPKTCNT": self.mav_pkts,
            "MVMSGID": self.last_msgid,
            "MVSTATE": 0,
            "MVPERR": 0,
            "MVOVR": 0,
            "MVPARSETO": parse_timeout,
            "MVPKTAGE": pkt_age,

            # --- command debug
            "CMDLGCNT": self.cmd_long_count,
            "CMDLGID": self.last_cmd_long,
            "CMDLGTGT": self.last_cmd_target,
            "GFCMDCNT": self.griff_cmd_count,
            "GFCMDID": self.last_griff_cmd,
            "GFCMDRES": self.last_griff_res,
            "GFCMDP1": self.last_griff_p1,
            "GFCMDTGT": self.last_griff_tgt,
            "GFCMDSYS": self.last_griff_sys,
            "GFCMDCMP": self.last_griff_comp,
            "GFCMDSEQ": self.last_griff_seq,
            "GFCMDAGE": cmd_age,
            "GFMAIN": self.main,
            "GFNAV": self.nav,
            "GFPLD": self.payload,
            "GFGIMBAL": self.gimbal,
            "GFRESET": self.reset,
            "GFSTATE": (100 * state_digit(self.main)
                        + 10 * state_digit(self.nav)
                        + state_digit(self.payload)),

            # --- power / IO
            "PWRSTAT": 9 if self.injected("pwr-mixed") else self.pwrstat(),
            "PDINSTAT": 10 * pd11 + 0,
            "CLAWLDKG": STALE_SENTINEL if claw_stale else (
                CLAW_LOAD_KG + 0.05 * math.sin(self.t / 3.0) if claw_live else 0.0),
            "CLAWSTAT": STALE_SENTINEL if claw_stale else (1 if claw_live else 0),

            # --- CAN
            "CANBO": self.can_bus_off,
            "CANERRHI": 0,
            "CANERRLO": 0,
            "CANRA": self.can_restart_attempts,
            "CANRS": self.can_restart_ok,
            "CANEV": self.can_last_event,
            "CANLEC": self.can_lec,
            "GXVER": GX_VERSION,

            # --- hybrid draft (not B100 firmware)
            "ENGRPM": self.rpm,
            "ENGTEMP": self.eng_c,
            "GENTEMP": self.gen_c,
            "BATTRSV": self.reserve_s,
            "HYBSTAT": self.hyb,
            "HYBREASON": self.reason,
            "HYBFLTIME": self.flt_time_s,
        }

        # forced values win over the plant, so that states Draft 0.1 defines
        # without a stimulus can still be shown to a GCS
        values.update(self.forced)
        return values

    def summary(self):
        """One line of what is actually being published, pins included."""
        hyb = int(self.forced.get("HYBSTAT", self.hyb))
        reason = int(self.forced.get("HYBREASON", self.reason))
        return ("PWRSTAT=%d GFSTATE=%03d HYBSTAT=%d %s%s HYBREASON=%d %s%s "
                "rsv=%.0fs armed=%d%s"
                % (self.forced.get("PWRSTAT", self.pwrstat()),
                   self.forced.get("GFSTATE",
                                   100 * state_digit(self.main)
                                   + 10 * state_digit(self.nav)
                                   + state_digit(self.payload)),
                   hyb, HYB_NAMES.get(hyb, "?"),
                   " (pinned, plant says %d)" % self.hyb
                   if "HYBSTAT" in self.forced else "",
                   reason, RSN_NAMES.get(reason, "?"),
                   " (pinned)" if "HYBREASON" in self.forced else "",
                   self.reserve_s, int(self.armed),
                   "  latched" if self.gen_latched else ""))


# ------------------------------------------------------------------- console
# Everything below drives the sim while it runs: single stimuli, pinned values,
# and canned timelines. Commands are plain lines, so the same strings work typed
# at the terminal, piped in, or listed inside a scenario.

ALL_FIELDS = frozenset(NORMAL_1HZ + VERBOSE_1HZ + CLAW_5HZ + CAN_10S_NORMAL
                       + CAN_10S_VERBOSE + VERSION_60S + HYBRID_FIELDS)

# name -> (default duration in seconds or None for "until cleared", help)
INJECTIONS = {
    "boot":          (5.0,  "EMU boot -> HYBSTAT 0 UNKNOWN"),
    "gen-lost":      (None, "generator output lost -> 7, or 9 once reserve < 90 s"),
    "emu-invalid":   (None, "EMU declares invalid -> 10 reason 1"),
    "emu-stale":     (None, "no EMU status within the CAN timeout -> 10 reason 2"),
    "emu-missing":   (None, "no EMU status ever received -> 10 reason 3"),
    "can-integrity": (None, "CAN counter/CRC check fails -> 10 reason 4"),
    "gx-fault":      (None, "Gx cannot bridge status -> 10 reason 5"),
    "src-invalid":   (None, "source id does not match the EMU -> 10 reason 7"),
    "mav-stale":     (5.0,  "stop publishing the hybrid fields entirely"),
    "cube-loss":     (None, "no Cube heartbeat on UART7 -> CUBEHBTO 1"),
    "parse-timeout": (None, "bytes arrive but do not parse -> MVPARSETO 1"),
    "claw-stale":    (None, "claw data stops updating -> CLAWLDKG/CLAWSTAT -999"),
    "uart7-err":     (None, "UART7 error -> U7ERR 8, U7RXSTA 32"),
    "pwr-mixed":     (None, "battery transistor feedback mixed -> PWRSTAT 9"),
}

HELP = """commands (one per line, or several separated by ';')
  help                      this list
  status                    print the resolved state
  power on|off              as if Griff command 6200 param1=1/0 arrived
  nav|payload on|off        as if 6210 / 6220 arrived
  arm on|off|auto           armed bit: GENERATOR_LOST only latches in flight
                            ('auto' follows the Cube heartbeat again)
  reserve <seconds>         set BATTRSV and hold it there
  hyb <0-10> [reason]       pin HYBSTAT (and HYBREASON)
  force <FIELD> <value>     pin any published field, e.g. force CLAWSTAT -999
  clear [FIELD|all]         drop pinned values and injections
  inject <name> [seconds]   feed a stimulus; omit seconds to hold it
  recover                   end a generator loss (the status may stay latched)
  latch-reset               clear a latched GENERATOR_LOST
  degrade continue|soon|off declare DEGRADED_CONTINUE / DEGRADED_LAND_SOON
  can-busoff                bump the CAN bus-off counters once
  scenario <name>           play a timeline; 'scenario list', 'scenario stop'
  quit"""


def _all_states_steps():
    """Hold each HYBSTAT in turn, then each reason code: the demo reel."""
    steps = [(0.0, "clear all"), (0.0, "say holding every HYBSTAT for 4 s")]
    for i in range(HYB_INVALID + 1):
        t = 1.0 + 4.0 * i
        steps.append((t, "force HYBSTAT %d" % i))
        steps.append((t, "say HYBSTAT=%d %s" % (i, HYB_NAMES[i])))
    base = 1.0 + 4.0 * (HYB_INVALID + 1)
    steps.append((base, "say now every HYBREASON, held against HYBSTAT=10"))
    for code in sorted(RSN_NAMES):
        t = base + 2.0 * code
        steps.append((t, "force HYBREASON %d" % code))
        steps.append((t, "say HYBREASON=%d %s" % (code, RSN_NAMES[code])))
    steps.append((base + 2.0 * (max(RSN_NAMES) + 1), "clear all"))
    return steps


SCENARIOS = {
    "nominal": (
        "cold start through to NORMAL (HST-EMU-001/003/004/005)", [
            (0.0, "clear all"), (0.0, "power off"), (0.0, "inject boot 3"),
            (0.0, "say EMU boot: expect 0 UNKNOWN, then 1 NOT_READY"),
            (3.5, "status"),
            (4.0, "power on"),
            (4.5, "say expect 2 READY, then 3 STARTING"), (6.0, "status"),
            (9.0, "status"),
            (14.0, "say expect 4 NORMAL: engine running, reserve >= 120 s"),
            (14.0, "status"),
        ]),
    "gen-loss": (
        "generator lost in flight, reserve drains (HST-EMU-007, HST-EMU-013)", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "arm on"),
            (0.0, "say waiting for NORMAL before the generator is lost"),
            (12.0, "status"),
            (12.0, "reserve 95"), (12.0, "inject gen-lost"),
            (12.0, "say generator lost; 500 ms debounce, then expect 7"),
            (13.5, "status"),
            (17.0, "say reserve now below 90 s: expect 9 EMERGENCY_POWER_ONLY"),
            (19.0, "status"),
        ]),
    "transient": (
        "sub-debounce generator dropout, status must not move (HST-EMU-008)", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "arm on"),
            (12.0, "status"),
            (12.0, "say 300 ms dropout: expect HYBSTAT to stay 4 NORMAL"),
            (12.0, "inject gen-lost 0.3"),
            (12.6, "status"), (14.0, "status"),
        ]),
    "latch": (
        "generator recovers but the status stays latched (HST-EMU-009)", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "arm on"),
            (12.0, "inject gen-lost"), (12.0, "say expect 7 GENERATOR_LOST"),
            (14.0, "status"),
            (14.0, "recover"),
            (14.0, "say generator back in flight: expect 7 to stand"),
            (16.0, "status"),
            (16.0, "say landing (disarm) releases the latch"), (16.0, "arm off"),
            (18.0, "status"),
        ]),
    "reserve-low": (
        "reserve below 90 s with the generator online (HST-EMU-012)", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "arm on"),
            (12.0, "status"),
            (12.0, "say reserve to 85 s, generator still online: expect 8"),
            (12.0, "reserve 85"), (13.0, "status"),
        ]),
    "priority": (
        "concurrent conditions resolve by priority (HST-EMU-024/025)", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "arm on"),
            (12.0, "reserve 85"), (12.0, "inject gen-lost"),
            (12.0, "say generator lost + reserve low: expect 9, not 7"),
            (14.0, "status"),
            (14.0, "inject emu-invalid"),
            (14.0, "say plus EMU invalid: expect 10, highest priority"),
            (16.0, "status"),
        ]),
    "invalid": (
        "EMU stale, then CAN integrity failure (HST-GX-004/006)", [
            (0.0, "clear all"), (0.0, "power on"),
            (12.0, "say EMU status stale: expect 10 reason 2"),
            (12.0, "inject emu-stale 5"), (13.0, "status"),
            (18.0, "say CAN integrity failure: expect 10 reason 4"),
            (18.0, "inject can-integrity 5"), (19.0, "status"),
        ]),
    "stale-link": (
        "hybrid publication stops (HST-AP-015, HST-CU-012)", [
            (0.0, "clear all"), (0.0, "power on"),
            (12.0, "status"),
            (12.0, "say hybrid fields stop for 6 s: the GCS must show "
                   "stale/lost, not the last NORMAL"),
            (12.0, "inject mav-stale 6"),
            (19.0, "say publication resumed"), (19.0, "status"),
        ]),
    "all-states": ("hold every HYBSTAT and HYBREASON in turn", _all_states_steps()),
    "b100-faults": (
        "B100 link, claw, CAN and power faults; no hybrid involvement", [
            (0.0, "clear all"), (0.0, "power on"), (0.0, "payload on"),
            (2.0, "say Cube heartbeat lost: expect CUBEHBTO 1"),
            (2.0, "inject cube-loss 6"),
            (9.0, "say parser starved: expect MVPARSETO 1"),
            (9.0, "inject parse-timeout 6"),
            (16.0, "say claw stale: expect CLAWLDKG/CLAWSTAT -999"),
            (16.0, "inject claw-stale 6"),
            (23.0, "say UART7 error and CAN bus-off"),
            (23.0, "inject uart7-err 6"), (23.0, "can-busoff"),
            (30.0, "say transistor feedback mixed: expect PWRSTAT 9"),
            (30.0, "inject pwr-mixed 6"),
            (37.0, "clear all"),
        ]),
}


class Console:
    """Line-oriented control of a running sim."""

    def __init__(self, b100, log=print, enabled=True):
        self.b100 = b100
        self.log = log
        self.enabled = enabled and sys.stdin is not None and not sys.stdin.closed
        self.scenario = None        # (name, steps, next index, start time)
        self.quit = False

    # ------------------------------------------------------------- input

    def poll(self):
        """Consume any complete line waiting on stdin."""
        if not self.enabled:
            return
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
        except (ValueError, OSError):
            self.enabled = False
            return
        if not ready:
            return
        line = sys.stdin.readline()
        if line == "":
            self.enabled = False    # EOF: piped input is exhausted
            return
        for part in line.split(";"):
            if part.strip():
                self.execute(part.strip())

    def step(self):
        """Run any scenario steps that have come due."""
        if self.scenario is None:
            return
        name, steps, i, t0 = self.scenario
        while i < len(steps) and self.b100.t - t0 >= steps[i][0]:
            self.execute(steps[i][1], quiet=True)
            i += 1
        if i >= len(steps):
            self.log("scenario %s: end" % name)
            self.scenario = None
        else:
            self.scenario = (name, steps, i, t0)

    # ----------------------------------------------------------- commands

    def execute(self, line, quiet=False):
        b = self.b100
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ("help", "?"):
                self.log(HELP)
                self.log("injections: " + ", ".join(sorted(INJECTIONS)))
            elif cmd == "say":
                self.log("  * " + " ".join(args))
            elif cmd == "status":
                self.log("  t=%6.1f  %s" % (b.t, b.summary()))
            elif cmd == "power":
                on = self._onoff(args)
                b._set_transistors(on)
                b.main = 1 if on else 0
                self._ok(quiet, "main power %s" % ("ON" if on else "OFF"))
            elif cmd in ("nav", "payload"):
                setattr(b, cmd, 1 if self._onoff(args) else 0)
                self._ok(quiet, "%s %s" % (cmd, args[0]))
            elif cmd == "arm":
                if args and args[0].lower() == "auto":
                    b.armed_forced = None
                else:
                    b.armed_forced = self._onoff(args)
                self._ok(quiet, "armed=%s" % b.armed)
            elif cmd == "reserve":
                b.reserve_s = float(args[0])
                b.reserve_hold = True
                self._ok(quiet, "BATTRSV=%.0fs (held)" % b.reserve_s)
            elif cmd == "hyb":
                b.forced["HYBSTAT"] = int(args[0])
                if len(args) > 1:
                    b.forced["HYBREASON"] = int(args[1])
                self._ok(quiet, "HYBSTAT pinned to %d %s"
                         % (int(args[0]), HYB_NAMES.get(int(args[0]), "?")))
            elif cmd == "force":
                name = args[0].upper()
                if name not in ALL_FIELDS:
                    self.log("  ! unknown field %s" % name)
                    return
                value = float(args[1])
                b.forced[name] = int(value) if value == int(value) else value
                self._ok(quiet, "%s pinned to %s" % (name, args[1]))
            elif cmd == "clear":
                target = args[0].upper() if args else "ALL"
                if target == "ALL":
                    b.forced.clear()
                    b.clear_inj()
                    b.degrade = None
                    b.gen_latched = False
                    b.gen_off_since = None
                    b.reserve_hold = False
                    self._ok(quiet, "cleared all overrides")
                else:
                    b.forced.pop(target, None)
                    b.clear_inj(target.lower())
                    self._ok(quiet, "cleared %s" % target)
            elif cmd == "inject":
                name = args[0].lower()
                if name not in INJECTIONS:
                    self.log("  ! unknown injection %s" % name)
                    return
                duration = float(args[1]) if len(args) > 1 else INJECTIONS[name][0]
                b.set_inj(name, duration)
                self._ok(quiet, "inject %s%s -- %s"
                         % (name, "" if duration is None else " for %gs" % duration,
                            INJECTIONS[name][1]))
            elif cmd == "recover":
                b.clear_inj("gen-lost")
                self._ok(quiet, "generator restored%s"
                         % (" (status still latched)" if b.gen_latched else ""))
            elif cmd == "latch-reset":
                b.gen_latched = False
                self._ok(quiet, "generator-lost latch cleared")
            elif cmd == "degrade":
                which = args[0].lower() if args else "off"
                b.degrade = {"continue": "continue", "soon": "land-soon",
                             "land-soon": "land-soon", "off": None}[which]
                self._ok(quiet, "degrade=%s" % b.degrade)
            elif cmd == "can-busoff":
                b.can_bus_off += 1
                b.can_last_event = 2
                b.can_lec = 4
                self._ok(quiet, "CANBO=%d" % b.can_bus_off)
            elif cmd == "scenario":
                self._scenario(args)
            elif cmd in ("quit", "exit"):
                self.quit = True
            else:
                self.log("  ! unknown command %r -- try 'help'" % line)
        except (IndexError, KeyError, ValueError) as err:
            self.log("  ! bad command %r (%s)" % (line, err))

    def _scenario(self, args):
        if not args or args[0] == "list":
            self.log("scenarios:")
            for name in sorted(SCENARIOS):
                self.log("  %-12s %s" % (name, SCENARIOS[name][0]))
            return
        if args[0] == "stop":
            self.scenario = None
            self.log("scenario stopped")
            return
        name = args[0]
        if name not in SCENARIOS:
            self.log("  ! no scenario %r -- try 'scenario list'" % name)
            return
        description, steps = SCENARIOS[name]
        self.scenario = (name, steps, 0, self.b100.t)
        self.log("scenario %s: %s" % (name, description))

    def _ok(self, quiet, text):
        self.log("  %s %s" % (">" if quiet else "<", text))

    @staticmethod
    def _onoff(args):
        return {"on": True, "1": True, "off": False, "0": False}[args[0].lower()]


def sweep_values(t, period):
    """--mode sweep: ramp every ranged value, cycle every coded value."""
    out = {}
    for i, (name, lo, hi) in enumerate(SWEEP_RANGES):
        x = math.fmod(t / period + i * 0.1, 1.0)
        tri = 2.0 * x if x < 0.5 else 2.0 * (1.0 - x)
        out[name] = lo + (hi - lo) * tri
    for name, choices in SWEEP_ENUMS:
        out[name] = choices[int(t) % len(choices)]
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="udpout:127.0.0.1:14550",
                    help="MAVLink connection string (default: %(default)s)")
    ap.add_argument("--mode", choices=("sim", "sweep"), default="sim",
                    help="coupled plant, or ramp every value through its range")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="scheduler tick in Hz; bounds the 10 Hz hybrid rate and "
                         "the 5 Hz claw rate (default: %(default)s)")
    ap.add_argument("--period", type=float, default=6.0,
                    help="sweep-mode ramp period in seconds (default: %(default)s)")
    ap.add_argument("--verbose-nval", action="store_true",
                    help="start with the verbose field set on (as if 6250 param1=1)")
    ap.add_argument("--airframe", choices=("griff30", "griff-hybrid"),
                    default="griff-hybrid",
                    help="which airframe to simulate: griff30 is all-electric "
                         "and publishes the B100 catalog only; griff-hybrid adds "
                         "the engine/generator block and the HybridStatus state "
                         "machine (default: %(default)s)")
    ap.add_argument("--hybrid-rate", type=float, default=HYB_PUBLISH_HZ,
                    help="hybrid block downlink rate in Hz (default: %(default)s; "
                         "the onboard ICD rate is 10, which is too fast for a GCS link)")
    ap.add_argument("--minimal", action="store_true",
                    help="publish only the 8 instrument-tray values (the hybrid "
                         "block plus PWRSTAT) and nothing else; same footprint as "
                         "the original injector")
    ap.add_argument("--no-hybrid", action="store_true",
                    help="omit the engine/hybrid fields (they are not B100 fields)")
    ap.add_argument("--all-float", action="store_true",
                    help="send every value as NAMED_VALUE_FLOAT, as the real board does")
    ap.add_argument("--reserve", type=float, default=RESERVE_FULL_S,
                    help="starting BATTRSV in seconds; lower it to reach the "
                         "Reserve Low (<90) and Emergency Power (<30) states "
                         "quickly, since reserve drains in real time "
                         "(default: %(default)s)")
    ap.add_argument("--faults", action="store_true",
                    help="sim mode: lose the generator once running")
    ap.add_argument("--fault-after", type=float, default=45.0,
                    help="seconds after power ON to lose the generator (default: %(default)s)")
    ap.add_argument("--fault-duration", type=float, default=60.0,
                    help="seconds before the generator recovers (default: %(default)s)")
    ap.add_argument("--power-on", action="store_true",
                    help="boot already powered instead of waiting for 6200 param1=1")
    ap.add_argument("--heartbeat", action="store_true",
                    help="also emit a HEARTBEAT from compid 25")
    ap.add_argument("--sysid", type=int, default=B100_SYSID,
                    help="source system id (default: %(default)s)")
    ap.add_argument("--compid", type=int, default=B100_COMPID,
                    help="source component id (default: %(default)s)")
    ap.add_argument("--no-console", action="store_true",
                    help="do not read trigger commands from stdin")
    ap.add_argument("--script", action="append", default=[],
                    help="console command to run at startup; repeatable, e.g. "
                         "--script 'scenario gen-loss'")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="log state each second and every command handled")
    args = ap.parse_args()

    mav = mavutil.mavlink_connection(args.connect, source_system=args.sysid,
                                     source_component=args.compid)
    print("B100 sim: %s, sys %u comp %u on %s, mode=%s"
          % (args.airframe, args.sysid, args.compid, args.connect, args.mode))
    if args.mode == "sim" and not args.power_on:
        print("waiting for MAV_CMD 6200 param1=1 (Main Power ON) -- %s"
              % ("engine off until then" if args.airframe == "griff-hybrid"
                 else "main power off until then"))
    if not args.no_console:
        print("console ready: type 'help' for triggers, 'scenario list' for timelines")

    b100 = B100(faults=args.faults, fault_after=args.fault_after,
                fault_duration=args.fault_duration,
                hybrid=(args.airframe == "griff-hybrid") and not args.no_hybrid,
                reserve_s=args.reserve)
    b100.verbose = args.verbose_nval
    if args.power_on:
        b100.main = 1
        b100.transistors = True
        b100.powered_at = 0.0

    def send(name, value):
        key = name.encode("ascii")[:9]
        if name in INT_VALUES and not args.all_float:
            mav.mav.named_value_int_send(boot_ms, key, int(round(value)))
        else:
            mav.mav.named_value_float_send(boot_ms, key, float(value))

    console = Console(b100, enabled=not args.no_console)
    for line in args.script:
        console.execute(line)

    tick = 1.0 / args.rate
    start = time.time()
    prev = start
    boot_ms = 0
    due_1hz = due_can = due_ver = due_hyb = due_claw = due_hb = due_log = 0.0

    while True:
        cycle_start = time.time()
        now = cycle_start - start
        dt = max(1e-3, cycle_start - prev)
        prev = cycle_start
        boot_ms = int(now * 1000)

        # --- drain whatever QGC has sent us ------------------------------
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            b100.note_packet(msg)
            if msg.get_type() != "COMMAND_LONG":
                continue    # COMMAND_INT is a no-op on the real board
            result = b100.handle_command_long(msg, args.compid)
            if result is None:
                if args.verbose:
                    print("  <-- cmd %d from %d/%d targeting %d/%d  --> no ACK"
                          % (msg.command, msg.get_srcSystem(), msg.get_srcComponent(),
                             msg.target_system, msg.target_component))
                continue
            mav.mav.command_ack_send(msg.command, result, 0, 0,
                                     msg.get_srcSystem(), msg.get_srcComponent())
            if args.verbose:
                print("  <-- cmd %d p1=%g p2=%g from %d/%d  --> ACK %s"
                      % (msg.command, msg.param1, msg.param2,
                         msg.get_srcSystem(), msg.get_srcComponent(),
                         mav1.enums["MAV_RESULT"][result].name))

        console.poll()
        if console.quit:
            print("bye")
            return
        b100.step(dt)
        console.step()

        # --- publish on the documented schedule --------------------------
        if args.mode == "sweep":
            values = sweep_values(now, args.period)
            values.update(b100.forced)
            for name in values:
                send(name, values[name])
        else:
            values = b100.values()
            if args.minimal:
                if now >= due_hyb:
                    due_hyb = now + 1.0 / args.hybrid_rate
                    for name in HYBRID_FIELDS + ("PWRSTAT",):
                        send(name, values[name])
                _publish_done = True
            else:
                _publish_done = False
            if not _publish_done and now >= due_claw:   # 5 Hz
                due_claw = now + 0.2
                for name in CLAW_5HZ:
                    send(name, values[name])
            if not _publish_done and now >= due_1hz:    # 1 Hz
                due_1hz = now + 1.0
                for name in NORMAL_1HZ:
                    send(name, values[name])
                if b100.verbose:
                    for name in VERBOSE_1HZ:
                        send(name, values[name])
            # 10 Hz per GRIFF-HYB-ICD-001 s6; "mav-stale" stops it entirely so
            # that the receiver's own staleness timeout can be demonstrated
            if (not _publish_done and now >= due_hyb and b100.hybrid
                    and not b100.injected("mav-stale")):
                due_hyb = now + 1.0 / args.hybrid_rate
                for name in HYBRID_FIELDS:
                    send(name, values[name])
            if not _publish_done and now >= due_can:    # 10 s
                due_can = now + 10.0
                for name in CAN_10S_NORMAL:
                    send(name, values[name])
                if b100.verbose:
                    for name in CAN_10S_VERBOSE:
                        send(name, values[name])
            if not _publish_done and now >= due_ver:    # 60 s
                due_ver = now + 60.0
                for name in VERSION_60S:
                    send(name, values[name])

        if args.heartbeat and now >= due_hb:
            due_hb = now + 1.0
            mav.mav.heartbeat_send(mav1.MAV_TYPE_GENERIC, mav1.MAV_AUTOPILOT_INVALID,
                                   0, 0, mav1.MAV_STATE_ACTIVE)

        if args.verbose and args.mode == "sim" and now >= due_log:
            due_log = now + 1.0
            pinned = ("  [pinned: %s]" % ",".join(sorted(b100.forced))
                      if b100.forced else "")
            if b100.hybrid:
                print("t=%6.1f  PWRSTAT=%d GFSTATE=%03d HYB=%d %-26s rpm=%5.0f "
                      "rsv=%5.0fs fltT=%5.0fs claw=%6.1fkg cmds=%d%s"
                      % (now, values["PWRSTAT"], values["GFSTATE"],
                         values["HYBSTAT"],
                         HYB_NAMES.get(values["HYBSTAT"], "?")
                         + ("/" + RSN_NAMES.get(values["HYBREASON"], "?")
                            if values["HYBREASON"] else ""),
                         values["ENGRPM"], values["BATTRSV"],
                         values["HYBFLTIME"], values["CLAWLDKG"],
                         values["GFCMDCNT"], pinned))
            else:
                print("t=%6.1f  PWRSTAT=%d GFSTATE=%03d nav=%d pld=%d "
                      "claw=%6.1fkg cubeHB=%s cmds=%d%s"
                      % (now, values["PWRSTAT"], values["GFSTATE"],
                         values["GFNAV"], values["GFPLD"], values["CLAWLDKG"],
                         "timeout" if values["CUBEHBTO"] else "ok",
                         values["GFCMDCNT"], pinned))

        time.sleep(max(0.0, tick - (time.time() - cycle_start)))


if __name__ == "__main__":
    main()
