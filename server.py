#!/usr/bin/env python3
"""AstroPi Sense HAT Dashboard — WebSocket server."""

import asyncio
import json
import logging
import math
import os
import random
import sys
import time
import threading
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import base64
import io as _io

import subprocess

import websockets
from PIL import Image
from sense_hat import SenseHat
from gpiozero import Button

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("astropi")

sense = SenseHat()
sense.low_light = True
sense.set_rotation(270)  # AstroPi case mounts the board vertically

CLIENTS: set = set()
SENSOR_INTERVAL = 0.05  # 50ms (20Hz for smooth filter updates)
WS_SEND_INTERVAL = 0.1  # Send to clients at 10Hz
PROJECT_DIR = Path(__file__).parent
MAG_CAL_FILE = PROJECT_DIR / "mag_cal.json"


# ── Madgwick AHRS filter ────────────────────────────────────────────


class MadgwickFilter:
    """Madgwick AHRS filter: fuses gyroscope + accelerometer (+ magnetometer).

    Sensor frame (board in flight case, display forward):
      +X = up, +Y = right, +Z = backward
    Gravity reference: [-1, 0, 0] (down = -X)
    Produces a quaternion — no gimbal lock.
    """

    def __init__(self, beta=0.3):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.beta = beta
        self._last_time = None
        self.gyro_bias = [0.0, 0.0, 0.0]
        self._mag_norm_avg = None  # Running average for interference detection

    def calibrate_gyro(self, samples=20):
        """Average gyro readings at startup to estimate bias."""
        log.info("Calibrating gyro (%d samples)...", samples)
        sums = [0.0, 0.0, 0.0]
        for _ in range(samples):
            g = sense.get_gyroscope_raw()
            sums[0] += g["x"]
            sums[1] += g["y"]
            sums[2] += g["z"]
            time.sleep(0.05)
        self.gyro_bias = [s / samples for s in sums]
        log.info("Gyro bias: %.4f, %.4f, %.4f", *self.gyro_bias)

    def update(self, gyro, accel, mag=None):
        """Update filter with gyro + accel, optionally + magnetometer (MARG).

        If mag is None or has abnormal norm, falls back to IMU-only update.
        """
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return self.q
        dt = now - self._last_time
        self._last_time = now
        if dt <= 0 or dt > 1.0:
            return self.q

        # Subtract gyro bias then negate Y/Z — gyro Y and Z axes are
        # sign-flipped relative to accelerometer frame on this hardware.
        gx = gyro["x"] - self.gyro_bias[0]
        gy = -(gyro["y"] - self.gyro_bias[1])
        gz = -(gyro["z"] - self.gyro_bias[2])
        q0, q1, q2, q3 = self.q

        # Normalize accelerometer
        ax, ay, az = accel["x"], accel["y"], accel["z"]
        anorm = math.sqrt(ax * ax + ay * ay + az * az)
        if anorm < 0.01:
            return self._integrate_gyro(gx, gy, gz, dt)
        ax, ay, az = ax / anorm, ay / anorm, az / anorm

        # Check if we can use magnetometer
        use_mag = False
        mx = my = mz = 0.0
        if mag is not None:
            mx, my, mz = mag["x"], mag["y"], mag["z"]
            mnorm = math.sqrt(mx * mx + my * my + mz * mz)
            if self._mag_norm_avg is not None and mnorm > 0.01:
                if 0.5 * self._mag_norm_avg < mnorm < 1.5 * self._mag_norm_avg:
                    mx, my, mz = mx / mnorm, my / mnorm, mz / mnorm
                    use_mag = True
                    self._mag_norm_avg = 0.95 * self._mag_norm_avg + 0.05 * mnorm
            elif mnorm > 0.01:
                self._mag_norm_avg = mnorm
                mx, my, mz = mx / mnorm, my / mnorm, mz / mnorm
                use_mag = True

        # IMU-only gradient for pitch/roll (gravity reference — proven correct)
        s0, s1, s2, s3 = self._gradient_imu(q0, q1, q2, q3, ax, ay, az)

        # Normalize gradient
        snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if snorm > 0:
            s0, s1, s2, s3 = s0 / snorm, s1 / snorm, s2 / snorm, s3 / snorm

        # Quaternion rate from gyroscope
        qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qdot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qdot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qdot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Integrate with gradient descent correction
        q0 += (qdot0 - self.beta * s0) * dt
        q1 += (qdot1 - self.beta * s1) * dt
        q2 += (qdot2 - self.beta * s2) * dt
        q3 += (qdot3 - self.beta * s3) * dt

        # Normalize quaternion
        qnorm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / qnorm, q1 / qnorm, q2 / qnorm, q3 / qnorm]

        # Yaw correction disabled — IMU-only gives correct pitch/roll
        # if use_mag and mag is not None:
        #     self._correct_yaw(mag, dt)

        return self.q

    def _correct_yaw(self, mag, dt, alpha=0.02):
        """Gently correct quaternion yaw toward magnetometer heading.

        Applies a small rotation around sensor +X (up axis) so that
        only yaw is affected — pitch and roll are untouched.
        """
        w, x, y, z = self.q
        mx, my, mz = mag["x"], mag["y"], mag["z"]

        # Current heading from quaternion: project mag to earth frame
        # using R(q)^T, then compute atan2 of horizontal components
        ex = (1 - 2*(y*y + z*z))*mx + 2*(x*y + w*z)*my + 2*(x*z - w*y)*mz
        ey = 2*(x*y - w*z)*mx + (1 - 2*(x*x + z*z))*my + 2*(y*z + w*x)*mz

        mag_heading = math.atan2(-ey, ex)

        # Quaternion's current yaw (rotation around X axis)
        quat_yaw = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))

        # Error (wrap to [-pi, pi])
        err = mag_heading - quat_yaw
        if err > math.pi:
            err -= 2 * math.pi
        elif err < -math.pi:
            err += 2 * math.pi

        # Small corrective rotation around sensor X (up): q_corr * q
        half = alpha * err * dt
        ch, sh = math.cos(half), math.sin(half)
        qw, qx, qy, qz = self.q
        self.q = [
            ch*qw - sh*qx,
            ch*qx + sh*qw,
            ch*qy - sh*qz,
            ch*qz + sh*qy,
        ]

    def _integrate_gyro(self, gx, gy, gz, dt):
        """Pure gyroscope integration (no correction)."""
        q0, q1, q2, q3 = self.q
        qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qdot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qdot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qdot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        q0 += qdot0 * dt
        q1 += qdot1 * dt
        q2 += qdot2 * dt
        q3 += qdot3 * dt
        qnorm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / qnorm, q1 / qnorm, q2 / qnorm, q3 / qnorm]
        return self.q

    def _gradient_imu(self, q0, q1, q2, q3, ax, ay, az):
        """Gradient for accel-only correction.

        Convention: R(q) = earth→sensor.  Accel ref = [+1, 0, 0] (up).
        Predicted sensor reading = R(q) * [1,0,0] = first column of R(q).
        """
        f1 = (1 - 2 * (q2 * q2 + q3 * q3)) - ax
        f2 = 2 * (q1 * q2 + q0 * q3) - ay
        f3 = 2 * (q1 * q3 - q0 * q2) - az
        s0 = 2 * q3 * f2 - 2 * q2 * f3
        s1 = 2 * q2 * f2 + 2 * q3 * f3
        s2 = -4 * q2 * f1 + 2 * q1 * f2 - 2 * q0 * f3
        s3 = -4 * q3 * f1 + 2 * q0 * f2 + 2 * q1 * f3
        return s0, s1, s2, s3

    def _gradient_marg(self, q0, q1, q2, q3, ax, ay, az, mx, my, mz):
        """Gradient for accel + mag correction (MARG).

        Convention: R(q) = earth→sensor.  Accel ref = [+1, 0, 0].
        Mag ref = [bx, 0, bz] derived from earth-frame projection via R^T.
        """
        # Accel gradient (same as IMU)
        sa0, sa1, sa2, sa3 = self._gradient_imu(q0, q1, q2, q3, ax, ay, az)

        # Earth-frame magnetic field: h = R(q)^T * m  (sensor→earth)
        hx = ((1 - 2 * (q2 * q2 + q3 * q3)) * mx
              + 2 * (q1 * q2 + q0 * q3) * my
              + 2 * (q1 * q3 - q0 * q2) * mz)
        hy = (2 * (q1 * q2 - q0 * q3) * mx
              + (1 - 2 * (q1 * q1 + q3 * q3)) * my
              + 2 * (q2 * q3 + q0 * q1) * mz)
        hz = (2 * (q1 * q3 + q0 * q2) * mx
              + 2 * (q2 * q3 - q0 * q1) * my
              + (1 - 2 * (q1 * q1 + q2 * q2)) * mz)

        bx = math.sqrt(hx * hx + hy * hy)
        bz = hz

        # Predicted mag in sensor frame: p = R(q) * [bx, 0, bz]
        p0 = ((1 - 2 * (q2 * q2 + q3 * q3)) * bx
              + 2 * (q1 * q3 + q0 * q2) * bz)
        p1 = (2 * (q1 * q2 + q0 * q3) * bx
              + 2 * (q2 * q3 - q0 * q1) * bz)
        p2 = (2 * (q1 * q3 - q0 * q2) * bx
              + (1 - 2 * (q1 * q1 + q2 * q2)) * bz)

        fm0 = p0 - mx
        fm1 = p1 - my
        fm2 = p2 - mz

        # Mag Jacobian^T * f_mag  (∂(R*b)/∂q)^T * f
        sm0 = (2 * q2 * bz) * fm0 + (2 * q3 * bx - 2 * q1 * bz) * fm1 + (-2 * q2 * bx) * fm2
        sm1 = (2 * q3 * bz) * fm0 + (2 * q2 * bx - 2 * q0 * bz) * fm1 + (2 * q3 * bx - 4 * q1 * bz) * fm2
        sm2 = (-4 * q2 * bx + 2 * q0 * bz) * fm0 + (2 * q1 * bx + 2 * q3 * bz) * fm1 + (-2 * q0 * bx - 4 * q2 * bz) * fm2
        sm3 = (-4 * q3 * bx + 2 * q1 * bz) * fm0 + (2 * q0 * bx + 2 * q2 * bz) * fm1 + (2 * q1 * bx) * fm2

        return sa0 + sm0, sa1 + sm1, sa2 + sm2, sa3 + sm3

    def css_matrix(self):
        """Convert quaternion to CSS rotation matrix.

        Sensor frame: +X=up, +Y=right, +Z=backward (in flight case)
        CSS frame:    +X=right, +Y=down, +Z=toward viewer
        Mapping: CSS_X=sensor_Y, CSS_Y=-sensor_X, CSS_Z=-sensor_Z
        R_css = M * R_sensor * M^T
        """
        w, x, y, z = self.q
        # R_sensor elements (standard quaternion rotation matrix)
        r00 = 1 - 2 * (y * y + z * z)
        r01 = 2 * (x * y - w * z)
        r02 = 2 * (x * z + w * y)
        r10 = 2 * (x * y + w * z)
        r11 = 1 - 2 * (x * x + z * z)
        r12 = 2 * (y * z - w * x)
        r20 = 2 * (x * z - w * y)
        r21 = 2 * (y * z + w * x)
        r22 = 1 - 2 * (x * x + y * y)

        # CSS rotation matrix from q_css = [w, -y, -x, -z].
        # Remaps left-handed sensor quaternion to right-handed CSS frame.
        # Proper rotation matrix (det=+1, no distortion).
        c00 = r11
        c01 = r10
        c02 = r12
        c10 = r01
        c11 = r00
        c12 = r02
        c20 = r21
        c21 = r20
        c22 = r22

        return [
            round(c00, 4), round(c01, 4), round(c02, 4),
            round(c10, 4), round(c11, 4), round(c12, 4),
            round(c20, 4), round(c21, 4), round(c22, 4),
        ]


# ── Magnetometer calibration ──────────────────────────────────────────


def load_mag_calibration() -> dict | None:
    """Load saved magnetometer calibration from JSON file."""
    if MAG_CAL_FILE.exists():
        try:
            with open(MAG_CAL_FILE) as f:
                cal = json.load(f)
            log.info(
                "Loaded mag calibration: offset=[%.1f, %.1f, %.1f]",
                cal["offset_x"], cal["offset_y"], cal["offset_z"],
            )
            return cal
        except (json.JSONDecodeError, KeyError) as e:
            log.warning("Bad mag calibration file, will recalibrate: %s", e)
    return None


def save_mag_calibration(cal: dict):
    """Save magnetometer calibration to JSON file."""
    with open(MAG_CAL_FILE, "w") as f:
        json.dump(cal, f, indent=2)
    log.info("Saved mag calibration to %s", MAG_CAL_FILE)


def calibrate_magnetometer(samples=100, interval=0.05) -> dict:
    """Collect mag samples and compute hard-iron offsets.

    Returns dict with offset_x/y/z and avg_norm.
    """
    log.info("Calibrating magnetometer (%d samples, rotate the device!)...", samples)
    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3
    for i in range(samples):
        m = sense.get_compass_raw()
        for j, axis in enumerate(("x", "y", "z")):
            val = m[axis]
            mins[j] = min(mins[j], val)
            maxs[j] = max(maxs[j], val)
        time.sleep(interval)
    offsets = [(mins[j] + maxs[j]) / 2 for j in range(3)]
    # Average norm from half-ranges (for interference detection baseline)
    ranges = [(maxs[j] - mins[j]) / 2 for j in range(3)]
    avg_norm = math.sqrt(sum(r * r for r in ranges))
    cal = {
        "offset_x": round(offsets[0], 2),
        "offset_y": round(offsets[1], 2),
        "offset_z": round(offsets[2], 2),
        "avg_norm": round(avg_norm, 2),
    }
    log.info(
        "Mag calibration: offset=[%.1f, %.1f, %.1f], norm=%.1f",
        cal["offset_x"], cal["offset_y"], cal["offset_z"], cal["avg_norm"],
    )
    return cal


def apply_mag_calibration(raw_mag: dict, cal: dict) -> dict:
    """Subtract hard-iron offsets from raw magnetometer reading."""
    return {
        "x": raw_mag["x"] - cal["offset_x"],
        "y": raw_mag["y"] - cal["offset_y"],
        "z": raw_mag["z"] - cal["offset_z"],
    }


ahrs = MadgwickFilter(beta=0.3)
for _attempt in range(5):
    try:
        ahrs.calibrate_gyro()
        break
    except OSError:
        log.warning("IMU init failed, retrying in 3s... (%d/5)", _attempt + 1)
        time.sleep(3)
        sense = SenseHat()
        sense.low_light = True
        sense.set_rotation(270)
else:
    log.error("IMU init failed after 5 attempts, exiting")
    sys.exit(1)

# Magnetometer calibration
if "--recalibrate" in sys.argv:
    mag_cal = calibrate_magnetometer()
    save_mag_calibration(mag_cal)
else:
    mag_cal = load_mag_calibration()
    if mag_cal is None:
        mag_cal = calibrate_magnetometer()
        save_mag_calibration(mag_cal)

# Seed filter with saved mag norm for interference detection
if mag_cal.get("avg_norm"):
    ahrs._mag_norm_avg = mag_cal["avg_norm"]

# ── Filter mode toggle ────────────────────────────────────────────────

filter_mode = "madgwick"  # "madgwick", "rtimu", or "test"
_rtimu_ref_q = None  # Reference quaternion for RTIMULib mounting correction

# ── Test mode: cycle through known quaternions ───────────────────────
import itertools

_c = math.cos(math.radians(22.5))
_s = math.sin(math.radians(22.5))
TEST_QUATERNIONS = [
    ("Home (identity)", [1.0, 0.0, 0.0, 0.0]),
    ("Pitch 45° (front down)", [_c, 0.0, -_s, 0.0]),
    ("Pitch -45° (front up)", [_c, 0.0, _s, 0.0]),
    ("Roll 45° CW (right down)", [_c, 0.0, 0.0, -_s]),
    ("Roll -45° CCW (left down)", [_c, 0.0, 0.0, _s]),
    ("Yaw 45° CW (from above)", [_c, _s, 0.0, 0.0]),
    ("Yaw -45° CCW (from above)", [_c, -_s, 0.0, 0.0]),
]
_test_cycle = itertools.cycle(TEST_QUATERNIONS)
_test_current = TEST_QUATERNIONS[0]
_test_last_switch = 0.0


def _quat_conjugate(q):
    """Quaternion conjugate (inverse for unit quaternion)."""
    return [q[0], -q[1], -q[2], -q[3]]


def _quat_multiply(a, b):
    """Hamilton product of two quaternions [w, x, y, z]."""
    return [
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ]


def _get_rtimu_quaternion():
    """Get raw quaternion from RTIMULib via sense_hat's internal IMU."""
    if sense._imu.IMURead():
        data = sense._imu.getIMUData()
        qp = data.get("fusionQPose")
        if qp:
            return list(qp)
    return None


def quaternion_to_css_matrix(q):
    """Convert quaternion to CSS rotation matrix (same mapping as MadgwickFilter.css_matrix)."""
    w, x, y, z = q
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - w * z)
    r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    # Sensor→CSS: same q_css=[w,-y,-x,-z] transform as MadgwickFilter.css_matrix
    return [
        round(r11, 4), round(r10, 4), round(r12, 4),
        round(r01, 4), round(r00, 4), round(r02, 4),
        round(r21, 4), round(r20, 4), round(r22, 4),
    ]


# ── Preset animations ────────────────────────────────────────────────

_cam_stream_task: asyncio.Task | None = None

_preset_task: asyncio.Task | None = None
_scroll_task: asyncio.Task | None = None
_scroll_gen = 0
_latest_heading = 0.0  # updated by sensor loop
_latest_mag = {"x": 0.0, "y": 0.0, "z": 0.0}  # calibrated mag from sensor loop
_imu_paused = False  # set True when compass preset owns the IMU
_scroll_lock = threading.Lock()


def _cancellable_show_message(text, text_colour, back_colour, scroll_speed, gen):
    """Like sense.show_message but checks generation counter between frames."""
    with _scroll_lock:
        previous_rotation = sense._rotation
        sense._rotation -= 90
        if sense._rotation < 0:
            sense._rotation = 270
        try:
            dummy_colour = [None, None, None]
            string_padding = [dummy_colour] * 64
            letter_padding = [dummy_colour] * 8
            scroll_pixels = []
            scroll_pixels.extend(string_padding)
            for s in text:
                scroll_pixels.extend(sense._trim_whitespace(sense._get_char_pixels(s)))
                scroll_pixels.extend(letter_padding)
            scroll_pixels.extend(string_padding)
            coloured_pixels = [
                text_colour if p == [255, 255, 255] else back_colour
                for p in scroll_pixels
            ]
            scroll_length = len(coloured_pixels) // 8
            for i in range(scroll_length - 8):
                if _scroll_gen != gen:
                    return
                start = i * 8
                sense.set_pixels(coloured_pixels[start:start + 64])
                time.sleep(scroll_speed)
        finally:
            sense._rotation = previous_rotation


async def _cancel_scroll():
    global _scroll_task, _scroll_gen
    _scroll_gen += 1
    if _scroll_task and not _scroll_task.done():
        _scroll_task.cancel()
        try:
            await _scroll_task
        except asyncio.CancelledError:
            pass
    _scroll_task = None


async def _cancel_preset():
    global _preset_task
    if _preset_task and not _preset_task.done():
        _preset_task.cancel()
        try:
            await _preset_task
        except asyncio.CancelledError:
            pass
    _preset_task = None


def _hsv_to_rgb(h, s, v):
    """Convert HSV (0-360, 0-1, 0-1) to RGB (0-255)."""
    c = v * s
    x = c * (1 - abs((h / 60) % 2 - 1))
    m = v - c
    if h < 60:
        r, g, b = c, x, 0
    elif h < 120:
        r, g, b = x, c, 0
    elif h < 180:
        r, g, b = 0, c, x
    elif h < 240:
        r, g, b = 0, x, c
    elif h < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x
    return [int((r + m) * 255), int((g + m) * 255), int((b + m) * 255)]


async def preset_rainbow():
    """Cycle rainbow colors across the matrix."""
    offset = 0
    while True:
        pixels = []
        for y in range(8):
            for x in range(8):
                hue = ((x + y) * 40 + offset) % 360
                pixels.append(_hsv_to_rgb(hue, 1.0, 0.6))
        sense.set_pixels(pixels)
        offset = (offset + 10) % 360
        await asyncio.sleep(0.08)


async def preset_heatmap():
    """Map current temperature to a heatmap on the LED matrix."""
    while True:
        temp = sense.get_temperature()
        # Map 15-40C to blue-red
        t = max(0.0, min(1.0, (temp - 15) / 25))
        r = int(t * 255)
        b = int((1 - t) * 255)
        g = int((1 - abs(t - 0.5) * 2) * 180)
        base = [r, g, b]
        pixels = []
        for y in range(8):
            for x in range(8):
                # Add slight variation
                variation = random.randint(-20, 20)
                pixels.append([
                    max(0, min(255, base[0] + variation)),
                    max(0, min(255, base[1] + variation)),
                    max(0, min(255, base[2] + variation)),
                ])
        sense.set_pixels(pixels)
        await asyncio.sleep(0.5)


async def preset_snake():
    """Simple snake animation across the matrix."""
    snake = [(0, 0)]
    dx, dy = 1, 0
    while True:
        hx, hy = snake[-1]
        nx, ny = hx + dx, hy + dy
        # Bounce off walls
        if nx < 0 or nx > 7:
            dx = -dx
            nx = hx + dx
        if ny < 0 or ny > 7:
            dy = -dy
            ny = hy + dy
        # Random turn
        if random.random() < 0.3:
            dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            nx, ny = hx + dx, hy + dy
            nx = max(0, min(7, nx))
            ny = max(0, min(7, ny))
        snake.append((nx, ny))
        if len(snake) > 10:
            snake.pop(0)
        sense.clear()
        for i, (sx, sy) in enumerate(snake):
            brightness = int((i + 1) / len(snake) * 255)
            sense.set_pixel(sx, sy, 0, brightness, 0)
        await asyncio.sleep(0.15)


async def preset_sparkle():
    """Random sparkle effect."""
    while True:
        pixels = [[0, 0, 0]] * 64
        for _ in range(random.randint(3, 10)):
            idx = random.randint(0, 63)
            pixels[idx] = [
                random.randint(100, 255),
                random.randint(100, 255),
                random.randint(100, 255),
            ]
        sense.set_pixels(pixels)
        await asyncio.sleep(0.1)


async def preset_compass():
    """Show compass needle pointing north on LED matrix.

    Hold the case flat with display/LEDs facing up, like a handheld compass.
    Board axes when flat: +X = away from HDMI, +Y = right.
    Display: row 0 = top (toward +X), col 7 = right (toward +Y).
    """
    # Cardinal markers at edges (dim)
    markers = {
        (3, 0): [0, 0, 80], (4, 0): [0, 0, 80],   # N
        (3, 7): [40, 40, 40], (4, 7): [40, 40, 40], # S
        (7, 3): [40, 40, 40], (7, 4): [40, 40, 40], # E
        (0, 3): [40, 40, 40], (0, 4): [40, 40, 40], # W
    }
    _compass_tick = 0
    while True:
        heading_rad = math.radians(_latest_heading)
        if _compass_tick % 10 == 0:
            log.info("compass heading: %.1f°", _latest_heading)
        _compass_tick += 1
        dx = math.sin(heading_rad)
        dy = math.cos(heading_rad)

        pixels = [[0, 0, 0]] * 64

        # Cardinal markers
        for (mx, my), col in markers.items():
            pixels[my * 8 + mx] = col

        # North needle (red)
        for t in range(1, 8):
            px = int(round(3.5 + dx * t * 0.5))
            py = int(round(3.5 + dy * t * 0.5))
            if 0 <= px < 8 and 0 <= py < 8:
                pixels[py * 8 + px] = [255, 0, 0]

        # South tail (dim white)
        for t in range(1, 6):
            px = int(round(3.5 - dx * t * 0.5))
            py = int(round(3.5 - dy * t * 0.5))
            if 0 <= px < 8 and 0 <= py < 8:
                pixels[py * 8 + px] = [100, 100, 100]

        # Center dot
        for cx, cy in [(3, 3), (3, 4), (4, 3), (4, 4)]:
            pixels[cy * 8 + cx] = [60, 60, 60]

        sense.set_pixels(pixels)
        await asyncio.sleep(0.1)


async def preset_camera():
    """Stream Pi camera to the 8x8 LED matrix."""
    from picamera2 import Picamera2
    loop = asyncio.get_event_loop()
    cam = Picamera2()
    try:
        config = cam.create_still_configuration(main={"size": (64, 48), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        await asyncio.sleep(0.5)  # warm-up
        while True:
            arr = await loop.run_in_executor(None, cam.capture_array)
            img = Image.fromarray(arr[:, :, ::-1])
            # Center-crop to square, then scale down to 8x8
            w, h = img.size
            side = min(w, h)
            left = (w - side) // 2
            top = (h - side) // 2
            img = img.crop((left, top, left + side, top + side)).resize((8, 8), Image.BOX).rotate(270)
            pixels = [list(img.getpixel((x, y))) for y in range(8) for x in range(8)]
            sense.set_pixels(pixels)
            # Broadcast to browser so LED grid updates
            msg = json.dumps({"type": "led_update", "pixels": pixels})
            for ws in list(CLIENTS):
                try:
                    await ws.send(msg)
                except Exception:
                    pass
            await asyncio.sleep(0.1)
    finally:
        cam.stop()
        cam.close()


PRESETS = {
    "rainbow": preset_rainbow,
    "heatmap": preset_heatmap,
    "snake": preset_snake,
    "sparkle": preset_sparkle,
    "compass": preset_compass,
    "camera": preset_camera,
}

# ── Sensor reading ───────────────────────────────────────────────────


def tilt_compensated_heading(q, mag):
    """Derive compass heading from quaternion and mag vector.

    Rotates mag into earth frame using R(q)^T (since q is earth→sensor,
    we need the transpose to go sensor→earth).
    """
    w, x, y, z = q
    mx, my, mz = mag["x"], mag["y"], mag["z"]
    # Rotate mag vector into earth frame: m_earth = R(q)^T * m
    # R(q)^T row 0 = R(q) column 0
    ex = (1 - 2 * (y * y + z * z)) * mx + 2 * (x * y + w * z) * my + 2 * (x * z - w * y) * mz
    # R(q)^T row 1 = R(q) column 1
    ey = 2 * (x * y - w * z) * mx + (1 - 2 * (x * x + z * z)) * my + 2 * (y * z + w * x) * mz
    # Heading from horizontal components
    heading = math.degrees(math.atan2(-ey, ex))
    if heading < 0:
        heading += 360
    return round(heading, 1)


# ── Accelerometer display smoothing ──────────────────────────────────

ACCEL_EMA_ALPHA = 0.3

_accel_smooth = {"x": 0.0, "y": 0.0, "z": 0.0}


def smooth_accel(raw: dict) -> dict:
    """Apply exponential moving average to accelerometer display values."""
    for axis in ("x", "y", "z"):
        _accel_smooth[axis] = (
            ACCEL_EMA_ALPHA * raw[axis]
            + (1 - ACCEL_EMA_ALPHA) * _accel_smooth[axis]
        )
    return {
        "x": round(_accel_smooth["x"], 3),
        "y": round(_accel_smooth["y"], 3),
        "z": round(_accel_smooth["z"], 3),
    }


def update_filter():
    """Read IMU and update the Madgwick AHRS filter."""
    accel = sense.get_accelerometer_raw()
    gyro = sense.get_gyroscope_raw()
    mag_raw = sense.get_compass_raw()
    mag = apply_mag_calibration(mag_raw, mag_cal)
    ahrs.update(gyro, accel, mag)
    return accel, gyro, mag


def read_sensors() -> dict:
    """Read all Sense HAT sensors and return as dict."""
    global _latest_heading, _latest_mag
    if filter_mode == "test":
        global _test_current, _test_last_switch
        now = time.time()
        if now - _test_last_switch > 3.0:
            _test_current = next(_test_cycle)
            _test_last_switch = now
            log.info("Test: %s  q=%s", _test_current[0], _test_current[1])
        label, q = _test_current
        w, x, y, z = q
        matrix = quaternion_to_css_matrix(q)
        accel = sense.get_accelerometer_raw()
        gyro = sense.get_gyroscope_raw()
        mag_raw = sense.get_compass_raw()
        mag = apply_mag_calibration(mag_raw, mag_cal)
        return {
            "type": "sensors",
            "timestamp": now,
            "temperature": round(sense.get_temperature(), 1),
            "humidity": round(sense.get_humidity(), 1),
            "pressure": round(sense.get_pressure(), 1),
            "orientation": {
                "pitch": label,
                "roll": 0,
                "yaw": 0,
                "matrix": matrix,
                "q": [round(w, 4), round(x, 4), round(y, 4), round(z, 4)],
            },
            "filter_mode": "test",
            "accelerometer": {"x": 0, "y": 0, "z": 0},
            "gyroscope": {"x": 0, "y": 0, "z": 0},
            "magnetometer": {"x": 0, "y": 0, "z": 0},
        }
    if filter_mode == "rtimu":
        # Use RTIMULib raw quaternion with mounting correction
        global _rtimu_ref_q
        q_raw = _get_rtimu_quaternion()
        accel = sense.get_accelerometer_raw()
        gyro = sense.get_gyroscope_raw()
        mag_raw = sense.get_compass_raw()
        mag = apply_mag_calibration(mag_raw, mag_cal)
        _latest_mag = mag
        if q_raw:
            if _rtimu_ref_q is None:
                # Capture current orientation as "home" (upright, display forward)
                _rtimu_ref_q = _quat_conjugate(q_raw)
                log.info("RTIMULib reference quaternion captured")
            # Relative rotation from home: q_rel = q_ref_inv * q_raw
            q_rel = _quat_multiply(_rtimu_ref_q, q_raw)
            # RTIMULib quaternion has pitch/roll (y,z) inverted vs our convention
            q_rel = [q_rel[0], q_rel[1], -q_rel[2], -q_rel[3]]
            matrix = quaternion_to_css_matrix(q_rel)
            # Euler from relative quaternion
            w, x, y, z = q_rel
        else:
            matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            w = x = y = z = 0.0
        # Extract Euler angles from quaternion
        # Negate pitch/roll so +pitch=front down, +roll=CW from front
        pitch = round(-math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))), 1)
        roll = round(-math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 1)
        yaw = round(math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))), 1)
        _latest_heading = yaw
    else:
        # Custom Madgwick MARG filter
        accel, gyro, mag = update_filter()
        _latest_mag = mag
        # Extract Euler angles from fused quaternion
        # Negate pitch/roll so +pitch=front down, +roll=CW from front
        w, x, y, z = ahrs.q
        pitch = round(-math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))), 1)
        roll = round(-math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 1)
        yaw = tilt_compensated_heading(ahrs.q, mag)
        _latest_heading = yaw
        matrix = ahrs.css_matrix()

    accel_display = smooth_accel(accel)
    _poll_joystick()

    return {
        "type": "sensors",
        "timestamp": time.time(),
        "temperature": round(sense.get_temperature(), 1),
        "humidity": round(sense.get_humidity(), 1),
        "pressure": round(sense.get_pressure(), 1),
        "orientation": {
            "pitch": pitch,
            "roll": roll,
            "yaw": yaw,
            "matrix": matrix,
            "q": [round(w, 4), round(x, 4), round(y, 4), round(z, 4)],
        },
        "filter_mode": filter_mode,
        "accelerometer": accel_display,
        "gyroscope": {
            "x": round(gyro["x"], 3),
            "y": round(gyro["y"], 3),
            "z": round(gyro["z"], 3),
        },
        "magnetometer": {
            "x": round(mag["x"], 1),
            "y": round(mag["y"], 1),
            "z": round(mag["z"], 1),
        },
        "buttons": {name: btn.is_pressed for name, btn in _gpio_buttons.items()},
        "joystick": dict(_joystick_state),
    }


# ── WebSocket handler ────────────────────────────────────────────────


async def broadcast_sensors():
    """Run filter at 20Hz, send to clients at 5Hz."""
    loop = asyncio.get_event_loop()
    ticks = 0
    send_every = max(1, int(WS_SEND_INTERVAL / SENSOR_INTERVAL))  # 4 ticks

    while True:
        # Always update the filter for smooth tracking
        if ticks % send_every == 0 and CLIENTS:
            # Full sensor read + send to clients
            data = await loop.run_in_executor(None, read_sensors)
            msg = json.dumps(data)
            dead = set()
            for ws in list(CLIENTS):
                try:
                    await ws.send(msg)
                except websockets.ConnectionClosed:
                    dead.add(ws)
            CLIENTS.difference_update(dead)
        else:
            # Just update the filter (fast, no env sensors) and keep mag fresh
            if filter_mode == "madgwick":
                def _update_and_mag():
                    global _latest_mag
                    _, _, mag = update_filter()
                    _latest_mag = mag
                await loop.run_in_executor(None, _update_and_mag)

        ticks += 1
        await asyncio.sleep(SENSOR_INTERVAL)


async def _camera_grab():
    """Capture one frame and return base64 JPEG."""
    from picamera2 import Picamera2
    loop = asyncio.get_event_loop()
    cam = Picamera2()
    try:
        config = cam.create_still_configuration(main={"size": (320, 240), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        await asyncio.sleep(0.5)
        arr = await loop.run_in_executor(None, cam.capture_array)
        img = Image.fromarray(arr[:, :, ::-1]).rotate(-90, expand=True)
        buf = _io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return base64.b64encode(buf.getvalue()).decode()
    finally:
        cam.stop()
        cam.close()


async def _camera_stream_loop():
    """Continuously capture frames and broadcast as base64 JPEG."""
    from picamera2 import Picamera2
    loop = asyncio.get_event_loop()
    cam = Picamera2()
    try:
        config = cam.create_still_configuration(main={"size": (320, 240), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        await asyncio.sleep(0.5)
        while True:
            arr = await loop.run_in_executor(None, cam.capture_array)
            img = Image.fromarray(arr[:, :, ::-1]).rotate(-90, expand=True)
            buf = _io.BytesIO()
            img.save(buf, format="JPEG", quality=70)
            b64 = base64.b64encode(buf.getvalue()).decode()
            msg = json.dumps({"type": "camera_frame", "data": b64})
            for ws in list(CLIENTS):
                try:
                    await ws.send(msg)
                except Exception:
                    pass
            await asyncio.sleep(0.15)  # ~7fps
    finally:
        cam.stop()
        cam.close()


async def _cancel_cam_stream():
    global _cam_stream_task
    if _cam_stream_task and not _cam_stream_task.done():
        _cam_stream_task.cancel()
        try:
            await _cam_stream_task
        except asyncio.CancelledError:
            pass
    _cam_stream_task = None


async def handle_command(data: dict):
    """Process a command from the browser."""
    global _preset_task, _scroll_task
    cmd = data.get("cmd")

    if cmd == "scroll_text":
        await _cancel_preset()
        await _cancel_scroll()
        text = data.get("text", "Hello!")
        color = data.get("color", [255, 255, 255])
        bg = data.get("bg", [0, 0, 0])
        speed = data.get("speed", 0.08)
        loop = data.get("loop", False)

        gen = _scroll_gen

        async def _do_scroll():
            while True:
                await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: _cancellable_show_message(text, color, bg, speed, gen),
                )
                if not loop or _scroll_gen != gen:
                    break

        _scroll_task = asyncio.create_task(_do_scroll())

    elif cmd == "set_pixel":
        await _cancel_preset()
        await _cancel_scroll()
        x = data.get("x", 0)
        y = data.get("y", 0)
        color = data.get("color", [255, 255, 255])
        sense.set_pixel(x, y, *color)

    elif cmd == "set_pixels":
        await _cancel_preset()
        await _cancel_scroll()
        pixels = data.get("pixels", [[0, 0, 0]] * 64)
        sense.set_pixels(pixels)

    elif cmd == "clear":
        await _cancel_preset()
        await _cancel_scroll()
        sense.clear()

    elif cmd == "preset":
        await _cancel_preset()
        await _cancel_scroll()
        name = data.get("name", "")
        if name in PRESETS:
            _preset_task = asyncio.create_task(PRESETS[name]())
            log.info("Started preset: %s", name)

    elif cmd == "stop_preset":
        await _cancel_preset()
        sense.clear()

    elif cmd == "set_filter":
        global filter_mode, _rtimu_ref_q
        mode = data.get("mode", "madgwick")
        if mode in ("madgwick", "rtimu", "test"):
            filter_mode = mode
            if mode == "rtimu":
                _rtimu_ref_q = None  # Re-capture reference on next read
            log.info("Filter mode set to: %s", filter_mode)

    elif cmd == "camera_grab":
        await _cancel_cam_stream()
        b64 = await _camera_grab()
        msg = json.dumps({"type": "camera_frame", "data": b64})
        for ws in list(CLIENTS):
            try:
                await ws.send(msg)
            except Exception:
                pass

    elif cmd == "camera_stream":
        global _cam_stream_task
        enable = data.get("enable", False)
        if enable:
            await _cancel_cam_stream()
            _cam_stream_task = asyncio.create_task(_camera_stream_loop())
            log.info("Camera stream started")
        else:
            await _cancel_cam_stream()
            log.info("Camera stream stopped")


async def ws_handler(ws):
    """Handle a single WebSocket connection."""
    CLIENTS.add(ws)
    remote = ws.remote_address
    log.info("Client connected: %s", remote)
    try:
        async for message in ws:
            try:
                data = json.loads(message)
                await handle_command(data)
            except json.JSONDecodeError:
                log.warning("Invalid JSON from %s", remote)
            except Exception as e:
                log.error("Command error: %s", e)
    except websockets.ConnectionClosed:
        pass
    finally:
        CLIENTS.discard(ws)
        log.info("Client disconnected: %s", remote)


# ── HTTP server (serves index.html) ──────────────────────────────────


def start_http_server(host, port):
    """Run a simple HTTP server in a background thread."""
    handler = partial(SimpleHTTPRequestHandler, directory=str(PROJECT_DIR))
    httpd = HTTPServer((host, port), handler)
    log.info("HTTP server at http://%s:%d", host, port)
    httpd.serve_forever()


# ── Joystick & GPIO Buttons ──────────────────────────────────────────

_joystick_state = {"up": False, "down": False, "left": False, "right": False, "middle": False}


# Joystick direction remapping for 270° board rotation
_joy_remap = {"up": "right", "down": "left", "left": "up", "right": "down", "middle": "middle"}


def _poll_joystick():
    """Drain joystick events and update state. Call from sensor loop."""
    for k in _joystick_state:
        _joystick_state[k] = False
    for event in sense.stick.get_events():
        mapped = _joy_remap.get(event.direction)
        if mapped:
            _joystick_state[mapped] = (event.action != sense.stick.STATE_RELEASE)

# AstroPi flight case: 6 buttons
# Top group: top=GPIO26, bottom=GPIO13, left=GPIO20, right=GPIO19
# Bottom pair: A=GPIO16, B=GPIO21
_gpio_buttons = {
    "top": Button(26),
    "bottom": Button(13),
    "left": Button(20),
    "right": Button(19),
    "a": Button(16, hold_time=2),
    "b": Button(21, hold_time=2),
}

_shutdown_triggered = False


def _button_shutdown(reboot=False):
    """Scroll a message on the LED matrix, then shutdown or reboot."""
    global _shutdown_triggered, _scroll_gen
    if _shutdown_triggered:
        return
    _shutdown_triggered = True

    action = "Rebooting..." if reboot else "Shutting down..."
    cmd = ["sudo", "shutdown", "-Fr", "now"] if reboot else ["sudo", "shutdown", "-Fh", "now"]

    log.info("Button pressed: %s", action)
    _scroll_gen += 1
    with _scroll_lock:
        sense.show_message(action, text_colour=[255, 100, 0], scroll_speed=0.06)
        sense.clear()

    subprocess.run(cmd)


_gpio_buttons["a"].when_held = lambda: _button_shutdown(reboot=False)
_gpio_buttons["b"].when_held = lambda: _button_shutdown(reboot=True)
log.info("GPIO buttons: 6 configured (A=shutdown, B=reboot)")


# ── Main ─────────────────────────────────────────────────────────────


async def main():
    host = "0.0.0.0"
    http_port = int(os.environ.get("HTTP_PORT", 8080))
    ws_port = int(os.environ.get("WS_PORT", 8081))

    # Start HTTP server in background thread
    http_thread = threading.Thread(target=start_http_server, args=(host, http_port), daemon=True)
    http_thread.start()

    # Start WebSocket server
    async with websockets.serve(ws_handler, host, ws_port):
        log.info("WebSocket server at ws://%s:%d", host, ws_port)
        log.info("Open http://<pi-address>:%d in your browser", http_port)
        await broadcast_sensors()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sense.clear()
        log.info("Shutting down.")
