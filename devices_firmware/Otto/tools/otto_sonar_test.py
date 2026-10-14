#!/usr/bin/env python3
"""Otto HC-SR04 freeze test: polls 'a$' like Scratch, optionally swings legs fast.

usage: otto_sonar_test.py [port] [phase_seconds] [baud]  (v3/v4: 57600, v2: 115200)
Phases: idle -> legs fast (speed 0) -> idle. Reports value changes and freezes.
"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
PHASE = float(sys.argv[2]) if len(sys.argv) > 2 else 15

BAUD = int(sys.argv[3]) if len(sys.argv) > 3 else 57600
s = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(2.0)  # reset by DTR + ident spam window
s.reset_input_buffer()
s.write(b' ')
time.sleep(0.3)
print('ident:', s.read(200))
s.reset_input_buffer()


def poll():
    s.write(b'a$')
    r = s.read(3)
    if len(r) == 3 and r[0:1] == b'#':
        return r[1], r[2]
    s.reset_input_buffer()
    return None, None


def legs(a, b):
    # z: LL RL LF RF speed '$' ; speed%8*5 -> 0 = fastest
    s.write(bytes([ord('z'), a, b, a, b, 0, ord('$')]))
    s.read(3)


def run(name, moving):
    print(f'--- {name} ({PHASE:.0f}s) ---')
    t0 = time.time(); last = None; last_change = t0; n = 0; vals = set()
    swing = 0; next_swing = t0
    while time.time() - t0 < PHASE:
        if moving and time.time() >= next_swing:
            swing ^= 1
            legs(60, 120) if swing else legs(120, 60)
            next_swing = time.time() + 0.25
        d, _ = poll(); n += 1
        if d is None:
            continue
        vals.add(d)
        now = time.time()
        if d != last:
            if last is not None and now - last_change > 2:
                print(f'  [{now - t0:6.2f}s] UNFROZE after {now - last_change:.1f}s')
            print(f'  [{now - t0:6.2f}s] dist={d}')
            last = d; last_change = now
        time.sleep(0.03)
    stuck = time.time() - last_change
    print(f'  polls={n} distinct={len(vals)} last={last} unchanged_for={stuck:.1f}s'
          + ('  <<< FROZEN' if stuck > 3 else ''))


run('idle: move hand in front of the sensor', False)
run('legs FAST (speed 0)', True)
legs(90, 90)
run('idle again: move hand in front of the sensor', False)
s.close()
