# Automatic payload release on loss of control

Experimental Griff feature. Disabled by default. It is not approved for automatic
release in flight until SITL, the B100/hook bench test and warn-only flight review
have passed. Do not merge before hardware validation.

## Configuration

All eight parameters are captured on arming; modifications apply on the next arming.

| Parameter | Default | Units / values | Purpose |
| --- | --- | --- | --- |
| PLDR_ENABLE | 0 | 0 disabled, 1 enabled | Monitor loss of control. |
| PLDR_PWM | 1950 | PWM-equivalent microseconds, 1000–2200 | Inclusive per-motor saturation threshold. |
| PLDR_TIME | 0.5 | seconds, 0.1–2.0 | Required continuous saturation of the same motor. |
| PLDR_ANG_PCT | 110 | percent, 100–200 | Percentage of the effective Loiter angle limit, otherwise ANGLE_MAX. |
| PLDR_ACTION | 0 | 0 warn/log only, 1 release | Warn-only never transmits or changes flight mode. |
| PLDR_MODE | 0 | 0 unchanged, 2 AltHold | Request once at the first successful release transmission. |
| PLDR_SYSID | 1 | 1–255 | B100 system ID; no broadcast. |
| PLDR_COMPID | 25 | 1–255 | B100 component ID; no broadcast. |

English metadata is included in Parameters.cpp. Mission Planner's stock metadata
does not automatically acquire private-fork descriptions.

## Detection and action

Monitoring requires armed/airborne, not standby, and an eligible multicopter mode.
Acro, Flip and Turtle are excluded. Each enabled motor has its own timer and
remembered control-loss condition, reset immediately below the PWM threshold or
when monitoring becomes ineligible. The tilt must strictly exceed the threshold
while thrust-direction attitude error is growing. The latter excludes pure yaw
error, matching crash_check, and is compared against an interpolated 100 ms history.

The trigger latches until disarm, including in warn-only mode and after failed
communication. No subsequent automatic trigger is allowed in that arming period.
The detector runs before the unchanged crash_check; this is not a guarantee that
every loss-of-control event satisfies the configured release conditions first.

Release sends COMMAND_LONG command 6220, param1=1, param2–7=0, to the exact learned
target route. A transaction sends at most three times, 250 ms apart, ending 750 ms
after the trigger. Missing route or TX space does not count as a successful send.
ACK source, command and supplied target extensions must match. ACCEPTED stops
retries, IN_PROGRESS waits until the same deadline, other results terminate the
transaction. First-send AltHold failure is recorded and does not stop release retries.

Landing/disarm or an observed manual COMMAND_LONG close (6220, param1=0, same
target) stops remaining retries. Manual hook control and the external B100 input
remain available; the automatic latch is not a B100 lock against closing. Commands
on another link that bypasses the autopilot cannot be observed by the autopilot.

The reviewed B100 main at a5c067e accepts explicit desired state and returns ACK
before physical hook movement. Its payload handler periodically sends that state
over CAN. ACK is **not** confirmation that the payload has physically separated.

## Diagnostics

PLDR is logged at 20 Hz while enabled/armed and on observed state/event changes.
TimeUS is microseconds; Tilt/Lim/Err are degrees; Rate is degrees/second; PWM is
microseconds; Sat is the longest currently active saturation interval in seconds.
Mot is the one-based ArduPilot motor number, Ch its one-based output channel, and
TM the motor responsible for the latched trigger. These numbers can differ: in
the supplied log 167, output 3 is mapped to ArduPilot motor 2.

States St: 0 disabled, 1 monitoring, 2 saturating, 3 warn latched, 4 pending,
5 accepted, 6 rejected, 7 timeout, 8 cancelled, 9 invalid configuration.
Ev bits: 0 saturation start, 1 trigger, 2 successful send, 3 ACK accepted,
4 ACK rejected, 5 timeout, 6 cancelled, 7 invalid configuration, 8 AltHold failed.
Tx counts successful sends; Ack=255 means no matching ACK. Start notices are
limited to one per five seconds. Other transaction events receive explicit text.

## Reproducible tests

From the repository root in Ubuntu, after generating MAVLink headers:

```bash
python3 Tools/scripts/test_payload_release.py
python3 Tools/scripts/replay_payload_release.py /path/to/000001{53..67}.BIN --output replay.json
python3 Tools/scripts/replay_payload_release.py /path/to/00000167.BIN --angle-max-only
Tools/autotest/autotest.py build.Copter test.Copter.AutoPayloadRelease
```

The native suite compiles the real production core and inserts the unchanged
vehicle/route method bodies into a harness that replaces sensors, outputs, HAL,
logger and transport boundaries. It is not closed-loop SITL. Use
`--mavlink-include PATH` if headers are generated outside the standard directories.

Replay reads FMT definitions directly and feeds the same core, using held RCOU
outputs and ANG (ATT fallback). Thrust-direction error is reconstructed from
actual and desired Euler attitudes. It reports missing data rather than silently
assuming no trigger. Logs that start after arming can establish armed/airborne
state from NOT_LANDED; this inference is explicitly reported. Time is reported
both since boot and relative to the first timestamped record.

Replay cannot reproduce unlogged PWM dips or the exact internal controller timing.
The supplied log 167 records RCOU at about 10 Hz, so PWM crossing/continuity is
limited to approximately 0.1 s resolution. ANG is much faster. Confirm real-time
timing in SITL and hardware; do not interpret replay timestamps as millisecond
accuracy for physical release.

The SITL scenario uses a synthetic 77 kg quad, not a validated G30 model. It
removes one simulated motor's thrust and tests warn-only/configuration freeze,
accepted/rejected/no ACK, manual close, retries, latch and ordering versus crash
messages. The fake B100 peer exists only on the SITL link. Every scenario resets
SITL; native tests separately cover disarm/re-arm without reboot. Execute this
scenario before treating its model/test timings as validated.

## Hardware rollout

Build CubeOrangePlus from the reviewed tree. First bench-test with no suspended
payload: routing, ACK, explicit opening, manual closing, and reboot/disarm behavior.
First loaded flight settings are PLDR_ENABLE=1, PLDR_ACTION=0, PLDR_MODE=0.
Review PLDR inputs against known flight events before enabling automatic release.
CG and motor headroom investigation remain a separate work item.
