#!/usr/bin/env python3
"""Watch Griff commands and their ACKs on a link, and say what happened to each.

GriffGroundControl files a command under the component it addressed and matches
the ACK on the component that ANSWERS (MavCommandQueue.cc handleCommandAck).
An ACK from the autopilot for a command aimed at the B100 is therefore dropped
without a trace in the UI -- it looks exactly like no ACK at all. This prints
both, so the two cases stop looking alike.

The ACK alone is conclusive, which matters because MAVProxy does not fan
GCS->vehicle traffic out to its other outputs: a sniffer on a spare output sees
the answers, not the questions. Any Griff-range ACK from a component other than
25 is the autopilot answering for a board that is not on the link.

Run it beside GGC on a spare MAVProxy output:

    mavproxy.py --master tcp:127.0.0.1:5760 \\
        --out udp:127.0.0.1:14550 \\
        --out udp:127.0.0.1:14551
    python3 Tools/autotest/griff_mav_sniff.py --connect udpin:127.0.0.1:14551

or straight onto a SITL serial port (--connect tcp:127.0.0.1:5762).
"""

import argparse
import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mav1

GRIFF_MIN, GRIFF_MAX = 6200, 6251
B100_COMPID = 25

NAMES = {
    6200: "main power", 6210: "nav lights", 6220: "payload",
    6230: "gimbal horizon", 6231: "gimbal nadir", 6232: "gimbal oblique",
    6240: "CAN reset", 6241: "eth reset", 6250: "verbose nval",
    6251: "uart7 test",
}


def describe(cmd):
    return "%d %s" % (cmd, NAMES.get(cmd, ""))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="udpin:127.0.0.1:14551",
                    help="MAVLink connection string (default: %(default)s)")
    ap.add_argument("--timeout", type=float, default=1.2,
                    help="seconds to wait for an ACK before calling it lost; "
                         "GGC's own ack timeout is 1.2 s (default: %(default)s)")
    ap.add_argument("--all-commands", action="store_true",
                    help="also report commands outside the Griff range")
    args = ap.parse_args()

    mav = mavutil.mavlink_connection(args.connect)
    print("sniffing %s -- Griff commands %d..%d, B100 is component %d\n"
          % (args.connect, GRIFF_MIN, GRIFF_MAX, B100_COMPID))

    pending = {}        # (target_component, command) -> (time, source)
    start = time.time()

    while True:
        now = time.time() - start

        for key in [k for k, v in pending.items() if now - v[0] > args.timeout]:
            comp, cmd = key
            del pending[key]
            print("  %7.1f  LOST     %s -> comp %d: no ACK in %.1fs"
                  % (now, describe(cmd), comp, args.timeout))
            if comp == B100_COMPID:
                print("           the B100 is not answering on this link. If the "
                      "autopilot answered instead, look for an UNSUPPORTED line "
                      "above -- GGC discards those.")

        msg = mav.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        kind = msg.get_type()

        if kind == "COMMAND_LONG":
            if not (args.all_commands or GRIFF_MIN <= msg.command <= GRIFF_MAX):
                continue
            pending[(msg.target_component, msg.command)] = (now, msg.get_srcSystem())
            print("  %7.1f  SENT     %s  p1=%g p2=%g  from %d/%d -> %d/%d"
                  % (now, describe(msg.command), msg.param1, msg.param2,
                     msg.get_srcSystem(), msg.get_srcComponent(),
                     msg.target_system, msg.target_component))

        elif kind == "COMMAND_ACK":
            if not (args.all_commands or GRIFF_MIN <= msg.command <= GRIFF_MAX):
                continue
            src_comp = msg.get_srcComponent()
            result = mav1.enums["MAV_RESULT"][msg.result].name
            pending.pop((src_comp, msg.command), None)
            for key in [k for k in pending if k[1] == msg.command]:
                del pending[key]        # answered, whoever answered it
            print("  %7.1f  ACK      %s  %s  from %d/%d"
                  % (now, describe(msg.command), result,
                     msg.get_srcSystem(), src_comp))

            if src_comp == B100_COMPID:
                print("           GGC accepts this: the answering component is "
                      "25, which is what the command was queued under.")
                continue

            print("           GGC DISCARDS THIS: component %d answered a command "
                  "meant for component 25, and the queue is keyed on the "
                  "answering component (MavCommandQueue.cc handleCommandAck). "
                  "GGC logs 'Ack not in list' and, 1.2 s later, reports "
                  "'Vehicle did not respond to command'." % src_comp)
            if msg.result == mav1.MAV_RESULT_UNSUPPORTED:
                print("           UNSUPPORTED from the autopilot means the B100 "
                      "is not on this link at all: no component 25 is known, so "
                      "the autopilot handled the command itself. Put the sim on "
                      "a SITL serial port -- see griff_b100_sim.md.")


if __name__ == "__main__":
    main()
