#!/usr/bin/env python3
"""
Check an OctaQuad X stability sweep run with MOT_OQ_BIAS against the same
sweep run at bias 0.

Usage:
  AP_Motors_test s frame_class=4 > base.csv
  AP_Motors_test s frame_class=4 oq_bias=0.2 > bias.csv
  check_octaquad_bias.py base.csv bias.csv 0.2

For every input row it checks that, compared with the unbiased row:
  - motors 1-4 moved by +d and motors 5-8 by -d (the same d for each),
  - d has the sign of the bias and is no larger than bias * collective,
  - d is the full request unless a motor is at 0 or 1,
  - every motor stays within 0..1 and the sum of the commands is unchanged,
  - the roll, pitch, yaw and throttle limit flags are unchanged.
The collective is the mean of the unbiased motor commands: on this frame
the roll, pitch and yaw factors sum to zero over the eight motors.
With thrust boost every row must equal the unbiased row (no bias during
boost, which may itself drive the lost motor above 1).
"""

import csv
import sys

TOL = 2e-4  # the sweep prints commands to 4 decimals


def rows(path):
    with open(path) as f:
        lines = f.read().splitlines()
    start = next((i for i, line in enumerate(lines) if line.startswith('Roll,')), None)
    if start is None:
        sys.exit('%s: no "Roll," header; not an AP_Motors_test stability sweep' % path)
    boost = None
    for line in lines[:start]:
        if line.startswith('Thrust boost:'):
            boost = line.split(':', 1)[1].strip() == 'True'
    if boost is None:
        sys.exit('%s: no "Thrust boost:" line before the header' % path)
    return boost, list(csv.DictReader(lines[start:]))


USAGE = 'usage: check_octaquad_bias.py <unbiased.csv> <biased.csv> <bias>'


def main():
    if len(sys.argv) != 4:
        sys.exit(USAGE)
    base_path, bias_path = sys.argv[1], sys.argv[2]
    try:
        bias = float(sys.argv[3])
    except ValueError:
        sys.exit('bias must be a number\n' + USAGE)
    base_boost, base = rows(base_path)
    bias_boost, biased = rows(bias_path)
    if base_boost != bias_boost or len(base) != len(biased):
        sys.exit('sweeps do not match')
    norm = ['Mot%d_norm' % i for i in range(1, 9)]
    flags = ['LimR', 'LimP', 'LimY', 'LimThD', 'LimThU']
    failures = 0
    full = limited = unchanged = 0
    for n, (b, r) in enumerate(zip(base, biased)):
        key = [r['Roll'], r['Pitch'], r['Yaw'], r['Thr']]
        if key != [b['Roll'], b['Pitch'], b['Yaw'], b['Thr']]:
            sys.exit('row %d inputs differ' % n)
        u = [float(b[k]) for k in norm]
        v = [float(r[k]) for k in norm]
        diff = [v[i] - u[i] for i in range(8)]
        d = diff[0]
        c = sum(u) / 8
        want = 0.0 if bias_boost else bias * c
        problems = []
        if bias_boost:
            # no bias during boost; boost itself may drive the lost motor above 1
            if any(abs(x) > TOL for x in diff) or [r[k] for k in flags] != [b[k] for k in flags]:
                problems.append('changed during thrust boost %s' % diff)
                failures += 1
                if failures <= 10:
                    print('row %d %s: %s' % (n, key, '; '.join(problems)))
            else:
                unchanged += 1
            continue
        if any(abs(diff[i] - d) > TOL for i in range(4)) or any(abs(diff[i] + d) > TOL for i in range(4, 8)):
            problems.append('layers moved unevenly %s' % diff)
        if any(x < -TOL or x > 1 + TOL for x in v):
            problems.append('command out of range %s' % v)
        if abs(sum(v) - sum(u)) > 8 * TOL:
            problems.append('collective changed %.4f -> %.4f' % (sum(u), sum(v)))
        if d * want < -TOL or abs(d) > abs(want) + TOL:
            problems.append('transfer %.4f outside 0..%.4f' % (d, want))
        at_bound = any(x < TOL or x > 1 - TOL for x in v)
        if abs(d - want) > TOL:
            if at_bound:
                limited += 1
            else:
                problems.append('transfer %.4f short of %.4f with no motor at a bound' % (d, want))
        else:
            full += 1
        if [r[k] for k in flags] != [b[k] for k in flags]:
            problems.append('limit flags changed')
        if problems:
            failures += 1
            if failures <= 10:
                print('row %d %s: %s' % (n, key, '; '.join(problems)))
    if bias_boost:
        print('%d rows (thrust boost): %d unchanged, %d failures' % (len(base), unchanged, failures))
    else:
        print('%d rows: %d full transfer, %d bound-limited, %d failures' % (len(base), full, limited, failures))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
