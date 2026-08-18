# Warehouse Inventory Drone Simulation

PX4 v1.16.2 + Gazebo Harmonic warehouse simulation. A three-camera drone
(`warehouse_scout`) flies a GPS-denied autonomous tour of two aisles, scanning
box QR codes and Code128 barcodes to build an inventory.

Localization is GPS-free: downward optical flow + rangefinder for dead
reckoning, AprilTag (tag36h11) floor and rack markers for absolute pose.

## Prerequisites

- Ubuntu with ROS 2 Jazzy at `/opt/ros/jazzy`
- PX4-Autopilot built at `~/PX4-Autopilot` (`make px4_sitl_default`)
- Gazebo Harmonic (`gz`)
- Project venv at `.venv`

One-time PX4 integration (also re-run if the project folder is moved or
renamed — it links the world and models into PX4 by absolute path):

```bash
cd "/home/coban/Desktop/warehouse test" && ./scripts/setup_px4_integration.sh
```

## Running a full tour

Five terminals. Start them in order — each step depends on the previous one.

### 0. Check for a stale Gazebo server

PX4 can exit while the `gz sim` server stays up; the next PX4 then attaches to
that stale server instead of starting its own world. Symptom: "I launched it
but nothing opened."

```bash
pgrep -af "px4|gz sim"
```

If that prints anything and you are not running a simulation:

```bash
pkill -9 -f "gz sim"
```

### 1. Simulation

```bash
cd "/home/coban/Desktop/warehouse test" && ./scripts/run_sim.sh
```

Note: `Number of good matches: 2` while the vehicle is on the ground is
normal — the optical-flow count only becomes meaningful in flight.

### 2. Localization (EV feed)

```bash
cd "/home/coban/Desktop/warehouse test" && source /opt/ros/jazzy/setup.bash && .venv/bin/python scripts/apriltag_localize.py
```

This writes `out/logs/ev_feed.csv`. Do not skip it: box position estimation
depends on it, and without it `build_inventory` drops every reading as
"no pose". While the drone is still on the ground it prints
`EV BOŞLUK ... tag YOK` — that is expected, poses start flowing after takeoff.

### 3. Scanning (QR + barcode)

```bash
cd "/home/coban/Desktop/warehouse test" && source /opt/ros/jazzy/setup.bash && .venv/bin/python scripts/scan_boxes.py --no-bridge --with-barcode
```

`--no-bridge` is required: `apriltag_localize` already opens the camera
bridge. Watch the summary lines for `etkin hız` (effective rate) and `barkod`
(barcode) counts.

To keep this run separate from previous ones, add `--out out/scans_run4` and
pass the same path to the analysis steps below.

### 4. Live camera view (optional, for screen recording)

```bash
cd "/home/coban/Desktop/warehouse test" && source /opt/ros/jazzy/setup.bash && .venv/bin/python scripts/view_front.py --no-bridge
```

Overlay colors: green = rack AprilTag, yellow = box QR, magenta = box barcode.
Close with `q` or `ESC` while the window has focus.

### 5. Flight

```bash
cd "/home/coban/Desktop/warehouse test" && bash scripts/flight_logged.sh tam_tur_geo --routes config/route_gen.yaml
```

Takes about 13 minutes. This script starts `log_state` in the background
itself, so no extra terminal is needed. Outputs: `out/logs/latest.log` and
`out/state_log.csv`.

When the flight finishes, press `Ctrl-C` in terminal 3 — the scan summary and
the final writes are printed there.

## After the flight

Run these four in order; the last three read what `build_inventory` produces.

```bash
cd "/home/coban/Desktop/warehouse test" && .venv/bin/python tools/build_inventory.py --truth
```

```bash
cd "/home/coban/Desktop/warehouse test" && .venv/bin/python tools/validate_inventory.py --list-worst 5
```

```bash
cd "/home/coban/Desktop/warehouse test" && .venv/bin/python tools/coverage_report.py --html out/coverage.html
```

```bash
cd "/home/coban/Desktop/warehouse test" && .venv/bin/python tools/view_inventory.py --truth
```

Outputs: `out/inventory.json` / `.csv`, `out/validation_report.json`,
`out/coverage.html`, `out/inventory_3d.html`. The last two are single-file
HTML reports with no external dependencies — open them directly in a browser.

## Regenerating the world

The world, labels and textures are generated from `config/warehouse.yaml`:

```bash
cd "/home/coban/Desktop/warehouse test" && bash scripts/regen.sh
```

Check code readability before flying — every code needs at least 3 pixels per
module in the rendered image, 4+ is comfortable:

```bash
cd "/home/coban/Desktop/warehouse test" && .venv/bin/python tools/gen_labels.py --budget
```

## Results (three full runs)

- Box QR: 216/216 (100%) in all three runs, 12/12 passes complete
- Inventory position: 3.8–4.2 cm median error, inventory accuracy 99.5–100%
- Barcode linking accuracy: 100%; link rate 67.5% after the 2026-08-18 fix
  (was 36% — degenerate zbar polygons were always rejected)
- Scan rate: 7.3 Hz, below the 10 Hz target; the remaining bottleneck is
  `pyzbar.decode` itself

## Notes

- The floor texture is critical infrastructure, not decoration: a flat grey
  floor gives the optical flow sensor nothing to track, which silently breaks
  velocity estimation and produced months of apparent "L1 instability".
- Do not tune EKF parameters. With no magnetometer and no GPS, optical flow is
  the only horizontal aid while the vehicle is on the ground; weakening it
  breaks arming.
- Label geometry always comes from `tools/gen_labels.py`. Deriving it in a
  second place has caused three separate silent offset errors.
