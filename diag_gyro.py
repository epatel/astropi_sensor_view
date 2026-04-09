#!/usr/bin/env python3
"""Diagnostic: sample gyro DURING rotation to map gyro axes to physical motions."""
import time
from sense_hat import SenseHat

sense = SenseHat()
sense.set_rotation(270)

def sample_during(label, duration=3.0):
    input(f"\n>>> {label} — press Enter then IMMEDIATELY start the motion...")
    peaks = {"x": 0.0, "y": 0.0, "z": 0.0}
    t0 = time.time()
    while time.time() - t0 < duration:
        g = sense.get_gyroscope_raw()
        for axis in ("x", "y", "z"):
            if abs(g[axis]) > abs(peaks[axis]):
                peaks[axis] = g[axis]
        time.sleep(0.02)
    print(f"  Gyro peaks: X={peaks['x']:+.3f}  Y={peaks['y']:+.3f}  Z={peaks['z']:+.3f}")
    dominant = max(peaks, key=lambda a: abs(peaks[a]))
    print(f"  Dominant axis: {dominant.upper()} ({peaks[dominant]:+.3f})")

print("=== AstroPi Gyro Axis Diagnostic ===")
print("Start from UPRIGHT position each time.")
print("Perform a SLOW, STEADY rotation for ~2 seconds.\n")

sample_during("1. PITCH: tilt the TOP FORWARD (away from you)")
sample_during("2. PITCH BACK: tilt the TOP BACKWARD (toward you)")
sample_during("3. ROLL: tilt the RIGHT side DOWN")
sample_during("4. ROLL BACK: tilt the LEFT side DOWN")
sample_during("5. YAW CW: rotate clockwise viewed from ABOVE")
sample_during("6. YAW CCW: rotate counter-clockwise viewed from ABOVE")

print("\nDone!")
