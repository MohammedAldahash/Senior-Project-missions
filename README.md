# Building Crack Inspection — Dual-Mission Drone System

## Overview

This system uses two autonomous drone missions and a backend pipeline to scan building facades for cracks, then return to each detected crack for close-range depth measurement.

```
Mission 1 (Scan)  →  Backend (Plan Generator)  →  Mission 2 (Depth Inspect)
```

---

## Execution Order

```bash
# Step 1 — Scan the building facade (produces mission1_detections.csv)
python mission1_scan.py

# Step 2 — Generate Mission 2 flight plan (produces mission2_flight_plan.csv)
python backend_plan_generator.py

# Step 3 — Fly depth inspection of every detected crack
python mission2_inspect.py
```

---

## File Descriptions

| File | Role |
|---|---|
| `mission1_scan.py` | Flies a boustrophedon grid over the facade, captures images, runs crack detection at each waypoint, logs GPS with 6-decimal precision, monitors video latency. Outputs `mission1_detections.csv`. |
| `backend_plan_generator.py` | Reads detections, validates GPS/data integrity, generates a multi-waypoint inspection plan per crack (approach → hover → depth-scan grid). Outputs `mission2_flight_plan.csv` and `backend_report.json`. |
| `mission2_inspect.py` | Loads the flight plan, converts GPS waypoints to NED offsets, flies to each crack, performs simulated depth measurements. Outputs `mission2_depth_report.json`. |

---

## Constraints & How They Are Met

### 1. Video Latency ≤ 1 second

Both missions measure video pipeline latency at every waypoint. Any frame exceeding 1000 ms is flagged as a violation and counted in the final report. In production, replace `measure_video_latency()` with real timestamp differencing between frame capture and ground-station receipt.

### 2. GPS Coordinate Precision — 6 Decimal Places

All GPS coordinates are rounded to 6 decimal places (`COORD_PRECISION = 6`) at capture time in Mission 1. The backend validates that every detection row has sufficient decimal precision before accepting it. Mission 2 preserves this precision through its NED coordinate transform.

### 3. Backend Generates Valid Mission 2 Plans for ≥ 95% of Detections

The backend validates each crack detection row against: required fields present, numeric GPS values, latitude/longitude within bounds, altitude plausibility, and decimal precision. Only rows that fail these checks are rejected. The generation success rate is computed and reported — the system prints a PASS/FAIL verdict against the 95% threshold.

---

## Data Flow

```
mission1_scan.py
  └─► mission1_detections.csv        (all waypoints + crack flags + GPS + latency)
  └─► mission1_all_waypoints.csv     (simplified waypoint log)

backend_plan_generator.py
  └─► mission2_flight_plan.csv       (ordered inspection waypoints per crack)
  └─► backend_report.json            (generation rate, failure log)

mission2_inspect.py
  └─► mission2_depth_report.json     (per-crack depth measurements + latency stats)
```

---

## Simulation Notes

- **Crack detection** is simulated with a seeded random confidence score (threshold 0.60). Replace `simulate_crack_detection()` with your CNN model.
- **Depth measurement** is simulated. Replace `simulate_depth_measurement()` with stereo vision, structured light, or LiDAR calls.
- **Video latency** is simulated with a Gaussian distribution. Replace with actual frame timestamp differencing in production.
- All three scripts connect to PX4 SITL on `udp://:14540`. Adjust the address for hardware or different simulators.
