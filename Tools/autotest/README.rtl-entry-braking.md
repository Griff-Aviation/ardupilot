# RTL entry braking

Base: `8bee65b35bee567d467c301be22e5f8af0f06fcb`, the firmware identified in
flight log `00000045.BIN`. The implementation changes RTL entry, not normal
Loiter pilot-input generation or normal Auto waypoint navigation.

## Control behaviour

Airborne RTL begins with `ModeRTL::ENTRY_BRAKE`:

* Initialise the horizontal reference at measured position and velocity, without
  reducing entry velocity to `RTL_SPEED`.
* Shape the reference toward zero velocity with `WPNAV_ACCEL` and the lower of
  `WPNAV_JERK` and the position/attitude controller's computed jerk capability.
* Limit the complete horizontal acceleration command (PID, reference and offsets)
  by vector magnitude and by the per-update vector jerk. Feed the unapplied
  command back into the existing saturation/anti-windup path.
* Preserve the learned velocity-integrator disturbance estimate when the NE
  controller was active. Do not reinterpret the entire old pilot/feedforward
  command as a long-lived I bias. Output slew maintains command continuity.
* Climb concurrently using the normal RTL altitude, rally, cone, fence and
  terrain policy and waypoint vertical speed/acceleration settings. The entry
  climb uses PosControl U-axis input shaping rather than a waypoint S-curve.
* Wait for actual/reference horizontal velocity, reference acceleration and
  final climb altitude to settle before starting the
  return waypoint. The original return altitude is retained; `RTL_CLIMB_MIN`
  is not applied again.

The achieved stopping point becomes the start of the return leg. A residual
position error to the integrated braking reference does not require the vehicle
to creep back to an artificial point after stopping. WPNav's normal initialiser
preserves the total output across that position-reference handover.

The optional PosControl limits are arguments to each RTL control call. Calls
without them retain the original path. There is no persistent limit to clear
when returning to Loiter/Auto, including mission RTL running inside Auto.
SmartRTL does not enter the new phase. No parameters or parameter IDs are added.

### Boundaries

The acceleration bound concerns the commanded NE vector, not measured physical
deceleration. Aerodynamic drag and attitude/thrust tracking also affect motion.
The special A/J limit ends at the handover to ordinary return navigation.

When the previous command is already inside the requested acceleration circle,
the entry limiter guarantees both magnitude and per-update jerk, within float
rounding. If it is already outside, an instantaneous hard clamp cannot also
preserve jerk: the limiter recovers without increasing the old magnitude. A
sudden reduction of the normal tilt safety limit takes priority over jerk.

## Closed-loop SITL regression

Dependencies are the usual SITL toolchain and `pymavlink`. Build an unchanged
binary from the base above and preserve it separately, then build the patched
binary. Use the same test runner and fixture for both:

```sh
python Tools/autotest/rtl_entry_braking_sitl.py \
  --root . --binary /path/to/base/arducopter \
  --output /tmp/rtl-entry-baseline --label baseline --bearings 0,45

python Tools/autotest/rtl_entry_braking_sitl.py \
  --root . --binary build/sitl/bin/arducopter \
  --output /tmp/rtl-entry-fixed --label fixed --bearings 0,45 \
  --expect-fixed --compare /tmp/rtl-entry-baseline/summary.json

python Tools/autotest/rtl_entry_braking_sitl.py \
  --root . --binary build/sitl/bin/arducopter \
  --output /tmp/rtl-entry-exit --label exit --exercise-exit \
  --expect-fixed --compare /tmp/rtl-entry-baseline/summary.json
```

Each fresh process reaches 14.8-15.2 m/s for at least three seconds in Loiter or
Auto before RTL. `RTL_SPEED=500`, `WPNAV_ACCEL=250`, `WPNAV_JERK=1` and the
flight-log position/velocity gains are used. Loiter input is centred only after
RTL acknowledgement. The exit test aborts braking to the original mode, regains
15 m/s and re-enters RTL. All cases must return, land and disarm.

The explicit low-drag JSON model permits 15 m/s with an initial thrust-vector
acceleration below 2.5 m/s2. It is a synthetic 3 kg test fixture, not a validated
model of the flight-log aircraft. The stock model needs much greater tilt at
15 m/s and would exercise the already-over-limit recovery case instead.

### Evidence and acceptance

SITL builds log `RTLB` at every active output-limiter update:

`TimeUS,Dt,RawN,RawE,OutN,OutE,PrevN,PrevE,Accel,Jerk,Seq`

All acceleration fields use m/s2, jerk uses m/s3 and `Dt` uses seconds. `Seq`
increments before the logger write, so missing records are detectable. This
instrumentation is compiled out of hardware builds. PSCN/PSCE are only about
10 Hz and cannot establish the per-update bound on their own.

The runner checks finite output, `hypot(OutN,OutE) <= 2.5001`, vector step no
larger than `Jerk*Dt` plus 0.0001 m/s2 rounding tolerance, continuous sequence
starting at one and a continuous previous-output chain, zero dropped writes,
no trace outside explicit RTL in these scenarios, and no initial target-speed
drop below 14 m/s. The fixture's entry/climb phase must finish within 30 seconds.
Completion requires RTL mode, a near-home, near-ground stationary position and
logged disarming, rather than disarming alone.
It records normal pre-RTL speed/pitch/roll/altitude for
baseline comparison (0.25 m/s, 1 degree, 1 degree and 0.5 m tolerances).

Every case retains its launch parameters, console, telemetry and BIN log, plus
`result.json` and `trace_metrics.json`. `summary.json` records all cases and
baseline comparisons. `--inspect-folder` can recheck an existing case without
flying again. A failed flight still preserves available trace diagnostics.
