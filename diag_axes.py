#!/usr/bin/env python3
"""Diagnostic: print raw sensor axes to determine orientation mapping.

Hold the Pi in each position and press Enter to sample.
"""
import time
from sense_hat import SenseHat

sense = SenseHat()
sense.set_rotation(270)

def sample(label):
    input(f"\n>>> {label} — press Enter to sample...")
    # Average 10 readings
    ax = ay = az = 0
    gx = gy = gz = 0
    mx = my = mz = 0
    N = 10
    for _ in range(N):
        a = sense.get_accelerometer_raw()
        g = sense.get_gyroscope_raw()
        m = sense.get_compass_raw()
        ax += a['x']; ay += a['y']; az += a['z']
        gx += g['x']; gy += g['y']; gz += g['z']
        mx += m['x']; my += m['y']; mz += m['z']
        time.sleep(0.05)
    print(f"  Accel: X={ax/N:+.3f}  Y={ay/N:+.3f}  Z={az/N:+.3f}")
    print(f"  Gyro:  X={gx/N:+.3f}  Y={gy/N:+.3f}  Z={gz/N:+.3f}")
    print(f"  Mag:   X={mx/N:+.1f}  Y={my/N:+.1f}  Z={mz/N:+.1f}")

print("=== AstroPi Axis Diagnostic ===")
print("Pi should be in the flight case.\n")

sample("1. UPRIGHT, display facing you (home position)")
sample("2. TILT FORWARD (top of case away from you, ~45°)")
sample("3. BACK TO UPRIGHT, then TILT RIGHT (~45°)")
sample("4. UPRIGHT, then ROTATE CW ~90° (viewed from above) and HOLD STILL")
sample("5. LAY FLAT, display facing UP (ceiling)")

print("\nDone! Copy-paste the output above.")
