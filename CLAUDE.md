# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real-time sensor fusion dashboard for Raspberry Pi Sense HAT. A Python WebSocket server reads 9-DOF IMU data, runs a Madgwick AHRS quaternion filter, and streams orientation/sensor data to a browser dashboard that renders a live 3D STL model via Three.js WebGL.

## Architecture

**Core files:**
- `server.py` — Python async server: sensor I/O → Madgwick filter → quaternion → CSS rotation matrix → WebSocket broadcast
- `index.html` — Vanilla JS dashboard: WebSocket → Three.js STL model rotation, sparkline charts, LED matrix painter, text scroller
- `astropi-case.stl` — 3D model of the AstroPi flight case, rendered via Three.js
- `run.sh` — Launcher script on Pi (activates venv, runs server.py)
- `test_grid.html` — Standalone LED grid test page
- `diag_axes.py` / `diag_gyro.py` — Sensor axis mapping diagnostics (run on Pi)

**Communication:** HTTP on port 8080 (serves static files via `SimpleHTTPRequestHandler` in a daemon thread), WebSocket on port 8081 (sensor data at 10Hz, commands from browser).

**Server threading:** `broadcast_sensors()` async loop reads IMU at 20Hz, runs the Madgwick filter, and sends JSON to all clients at 10Hz. HTTP server runs in a background `threading.Thread`. LED presets and text scrolling use cancellable async tasks.

**WebSocket commands** (browser→server): `set_pixel`, `set_pixels`, `clear`, `scroll_text`, `preset`, `stop_preset`, `set_filter` (mode: `madgwick`/`rtimu`/`test`), `camera_grab`, `camera_stream` (enable: true/false).

**WebSocket messages** (server→browser): `sensors` (10Hz sensor data), `led_update` (pixel array during camera preset), `camera_frame` (base64 JPEG for camera panel).

## Sensor Fusion Pipeline

1. **IMU read** (20Hz): accelerometer, gyroscope, magnetometer from Sense HAT
2. **Gyro bias subtraction** + **Y/Z axis negation** (LSM9DS1 hardware has gyro Y/Z sign-flipped relative to accelerometer)
3. **Madgwick IMU gradient** corrects pitch/roll via gravity reference `[+1,0,0]` (sensor +X = up in flight case)
4. **Quaternion** `[w,x,y,z]` in earth→sensor convention
5. **CSS matrix** derived via `q_css = [w, -y, -x, -z]` remapping (left-handed sensor frame → right-handed CSS frame), producing a proper rotation matrix (det=+1)
6. **Tilt-compensated heading** from `R(q)^T * mag` projected to earth horizontal plane

**Sensor frame (board in flight case):** +X=up, +Y=right, +Z=backward. This is left-handed (X×Y=+Z=backward).

**CSS frame:** +X=right, +Y=down, +Z=toward viewer. Right-handed.

**Three.js frame:** +X=right, +Y=up, +Z=toward viewer. Right-handed. CSS→Three.js conversion flips Y: `M_three = F*M_css*F` where `F=diag(1,-1,1)`.

## 3D Model Rendering

The orientation card uses Three.js (v0.170.0, loaded via importmap from CDN) with STLLoader. The CSS cube HTML is kept hidden for JS compatibility but not rendered. The existing `<script>` computes the CSS rotation matrix as before, then calls `window.updateModel(m)` which the Three.js `<script type="module">` implements. The model auto-fits via bounding sphere: camera distance is computed as `radius / sin(fov/2)`.

## Filter Modes

- **Madgwick** (default): Custom IMU-only gradient for pitch/roll. MARG magnetometer gradient exists but is disabled pending mag axis calibration. Yaw drifts slowly from gyro integration; compass heading display is always correct.
- **RTIMULib**: Raw quaternion from `sense._imu.getIMUData()['fusionQPose']` with mounting correction via reference quaternion capture.
- **Test**: Cycles through 7 known quaternions (identity, ±pitch, ±roll, ±yaw) for visual verification.

## Dependencies (on Pi)

Python packages (installed in `~/venv`): `websockets`, `sense-hat`, `picamera2`, `pillow`

## Deployment

```bash
# Deploy to Pi (include STL when model changes)
scp server.py index.html astropi-case.stl astropi:~/projects/sensor-view/

# Start on Pi (activates ~/venv, runs server.py)
ssh astropi '~/projects/sensor-view/run.sh'

# Or manually
ssh astropi 'pkill -f server.py; sleep 1; ~/projects/sensor-view/run.sh'
```

Pi uses externally-managed Python (PEP 668) — always use the venv at `~/venv` for pip/python3.

**Browser access:** `http://astropi:8080` (requires Pi hostname resolution or use its IP).

## Diagnostic Tools

```bash
# Map accelerometer axes to physical directions (static positions)
ssh astropi '~/venv/bin/python3 ~/projects/sensor-view/diag_axes.py'

# Map gyroscope axes during active rotation
ssh astropi '~/venv/bin/python3 ~/projects/sensor-view/diag_gyro.py'
```

## Key Technical Pitfalls

- **LSM9DS1 axis conventions**: Gyro Y/Z are sign-flipped relative to accel. Magnetometer Z is inverted relative to accel. These must be corrected before feeding to the filter.
- **Madgwick gradient reference**: Must use `[+1,0,0]` (accel "up" direction), not `[-1,0,0]` (gravity). The Jacobian was derived for the positive reference.
- **CSS rotation matrix**: Cannot use simple `M*R*M^T` conjugation because `det(M)=-1` (improper mapping between left/right-handed frames). Instead, remap the quaternion: `q_css = [w, -y, -x, -z]`, then build `R(q_css)` which is always a proper rotation. The Three.js model receives this same matrix with Y-axis conjugation applied.
- **Three.js module scope**: The dashboard uses `<script type="module">` for Three.js imports. Functions called from inline `onclick` handlers must be exposed via `window.fn = fn`.
- **picamera2 BGR output**: `capture_array()` returns BGR despite `RGB888` format string. Always swap channels: `arr[:, :, ::-1]`. The camera is mounted sideways in the flight case — apply `rotate(-90)` for the browser panel, `rotate(270)` for the 8x8 LED grid.
- **Camera resource contention**: Only one picamera2 instance can be open at a time. The camera preset (LEDs) and camera panel (stream/grab) each open and close their own instance. Starting one while the other is active will fail — cancel the preset before using the panel, and vice versa.
- **Magnetometer calibration**: Hard-iron offsets stored in `mag_cal.json` on Pi. Use `--recalibrate` flag to force fresh calibration.
