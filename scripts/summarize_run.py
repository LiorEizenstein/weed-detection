#!/usr/bin/env python3
"""
summarize_run.py — compact summary of a watermelon_demo run log.

Usage:
    python3 summarize_run.py run_logs/run_<timestamp>.log
    python3 summarize_run.py run_logs/run_<timestamp>.log --full
"""

import re
import sys

# Patterns to extract (node prefix stripped for brevity)
PATTERNS = [
    (re.compile(r'SCAN pose (\d+)/(\d+)\s+pan=([+-]?\d+\.\d+)'), 'SCAN'),
    (re.compile(r'Skip scan pos pan=([+-]?\d+\.\d+)'),             'SKIP'),
    (re.compile(r'Weed spotted at pixel \((\d+),(\d+)\)'),         'SPOT'),
    (re.compile(r'Skip detection px=\((\d+),(\d+)\): world pos'),  'SKIP_WORLD'),
    (re.compile(r'Centering: weed px=\((\d+),(\d+)\) '
                r'err=\(([+-]\d+\.\d+),([+-]\d+\.\d+)\) '
                r'w1=([+-]\d+\.\d+) w2=([+-]\d+\.\d+)'),          'CENTER'),
    (re.compile(r'Firing laser at weed'),                           'FIRE'),
    (re.compile(r'Treated weed world pos: \(([+-]?\d+\.\d+),([+-]?\d+\.\d+)\)'), 'WORLD'),
    (re.compile(r'Weed treated — pan=([+-]?\d+\.\d+) blacklisted '
                r'\(total treated: (\d+)\)'),                       'TREATED'),
    (re.compile(r'Weed \d+ marked as treated'),                     'FM_TREATED'),
    (re.compile(r'Lost the weed'),                                  'LOST'),
    (re.compile(r'INIT: moving to HOME'),                           'INIT'),
]


def parse_log(path, full=False):
    events = []
    with open(path) as f:
        for line in f:
            # Skip lines without node info or with only detection_node frame stats
            if 'frame#' in line and 'weed(s)' in line and 'Centering' not in line:
                continue
            ts_m = re.search(r'\[(\d+\.\d+)\]', line)
            ts = float(ts_m.group(1)) if ts_m else 0.0
            for pat, label in PATTERNS:
                m = pat.search(line)
                if m:
                    events.append((ts, label, m.groups()))
                    break
    return events


def format_events(events, full):
    t0 = events[0][0] if events else 0.0
    current_scan = None
    center_count = 0
    out = []

    for ts, label, groups in events:
        rel = ts - t0

        if label == 'INIT':
            out.append(f't+{rel:6.1f}s  INIT')

        elif label == 'SCAN':
            idx, total, pan = groups
            if current_scan != idx:
                current_scan = idx
                center_count = 0
                out.append(f't+{rel:6.1f}s  SCAN {idx}/{total}  pan={pan}')

        elif label == 'SKIP':
            out.append(f't+{rel:6.1f}s    skip pan={groups[0]}')

        elif label == 'SKIP_WORLD':
            cx, cy = groups
            out.append(f't+{rel:6.1f}s    skip world-pos match  px=({cx},{cy})')

        elif label == 'SPOT':
            cx, cy = groups
            out.append(f't+{rel:6.1f}s    WEED SPOTTED  px=({cx},{cy})')

        elif label == 'CENTER':
            cx, cy, ex, ey, w1, w2 = groups
            center_count += 1
            if full:
                out.append(f't+{rel:6.1f}s    center #{center_count}  '
                           f'px=({cx},{cy}) err=({ex},{ey})  w1={w1} w2={w2}')
            # in compact mode only show the first and last center step
            elif center_count == 1:
                out.append(f't+{rel:6.1f}s    center #1  px=({cx},{cy}) err=({ex},{ey})')

        elif label == 'FIRE':
            if not full and center_count > 1:
                # emit summary of centering steps we suppressed
                out.append(f'          ... ({center_count - 1} more centering steps)')
            out.append(f't+{rel:6.1f}s    >>> FIRE <<<')
            center_count = 0

        elif label == 'WORLD':
            x, y = groups
            out.append(f't+{rel:6.1f}s    treated world=({x},{y})')

        elif label == 'TREATED':
            pan, total = groups
            out.append(f't+{rel:6.1f}s    blacklisted pan={pan}  (total={total})')

        elif label == 'FM_TREATED':
            out.append(f't+{rel:6.1f}s    [field_mgr] {groups[0] if groups else "weed treated"}')

        elif label == 'LOST':
            out.append(f't+{rel:6.1f}s    !! LOST WEED — resuming scan')

    return out


def main():
    if len(sys.argv) < 2:
        print('Usage: summarize_run.py <logfile> [--full]')
        sys.exit(1)
    path = sys.argv[1]
    full = '--full' in sys.argv

    events = parse_log(path, full)
    lines = format_events(events, full)
    print(f'=== {path} ({len(events)} events) ===')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
