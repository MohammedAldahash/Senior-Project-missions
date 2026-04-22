# Autonomous Drone Building Crack Inspection System

A two-mission autonomous drone system for detecting and depth-measuring structural cracks on building facades. Built on **PX4 Pixhawk**, **Raspberry Pi 5**, and **Intel RealSense D435**.

---

## System Architecture

```
Mission 1 (Facade Scan)
  ↓  img/*.jpg  (NED-encoded filenames)
Backend — YOLOv8 Crack Detection
  ↓  mission2_cracks.xlsx
TSP Solver — Optimal Route Planning
  ↓  mission2_route.xlsx
Mission 2 (Close-up Depth Inspection)
  ↓  final_report.xlsx
```

---

## Hardware

| Component | Details |
|---|---|
| Flight Controller | PX4 Pixhawk |
| Onboard Computer | Raspberry Pi 5 |
| Camera | Intel RealSense D435 (RGB-D) |
| Connection | Serial `/dev/ttyACM0` at 921600 baud |
| Simulation | PX4 SITL + Gazebo (`udp://:14540`) |

---

## Full Execution Pipeline

### Step 1 — Scan the building facade

```bash
# Real hardware (Pixhawk + RealSense)
python mission1_scan.py

# Simulation (Gazebo SITL + webcam/dummy feed)
python mission1_scanSim.py

# Camera test only (RealSense, no drone)
python mission1_scan_cameratest.py
```

The drone flies a **boustrophedon (serpentine) grid** over the facade. At each waypoint it captures a RealSense color image named after its NED position:

```
img_0.00_-1.00_-3.00.jpg   →   North=0.00  East=-1.00  Down=-3.00
```

- Max height: **15 rows (≈ 30 m)**
- Step size: **1 m per waypoint**
- Battery abort threshold: **20%**
- Output folder: `img/`

---

### Step 2 — Detect cracks with YOLOv8

```bash
python backend_plan_generator.py --images img/ --model best.pt --output mission2_cracks.xlsx
```

Runs a **YOLOv8** model on every image from Mission 1. For each detection it:
1. Parses drone NED from the filename
2. Computes the crack's absolute NED position using the bounding-box center offset
3. Appends a row to the output Excel file

**Detected crack types:** Corrosion, Settlement, Thermal Expansion, Diagonal Shear, Flexural, Compression, Tension, Torsion

| Argument | Default | Description |
|---|---|---|
| `--images` | `img` | Mission 1 image folder |
| `--model` | `best.pt` | YOLOv8 weights file |
| `--output` | `mission2_cracks.xlsx` | Output Excel (sheet: `Cracks`) |
| `--conf` | `0.25` | Confidence threshold |

---

### Step 3 — Solve optimal inspection route (TSP)

```bash
python mission2_solverTSPv2.py
```

Reads `detections.xlsx` (sheet: `Detections`) and solves the **Travelling Salesman Problem** to minimize total flight distance for Mission 2.

- **Exact solver** (brute-force permutations) for ≤ 10 cracks
- **Heuristic** (Nearest Neighbor + 2-opt) for > 10 cracks
- Output: `mission2_route.xlsx` with 3 sheets: `Route`, `VisitOrder`, `Summary`

---

### Step 4 — Close-up depth inspection

```bash
# Real hardware (Pixhawk + RealSense color + depth)
python mission2_inspect.py

# Simulation (Gazebo SITL + webcam, no depth)
python mission2_inspectSim.py

# Camera test only (RealSense color + depth, no drone)
python mission2_inspect_cameratest.py
```

Reads `mission2_route.xlsx` (sheet: `VisitOrder`) and flies to each crack in TSP order. At each crack:

1. Positions drone **0.5 m closer** to the wall than the detected position
2. Captures aligned **color + depth** frames from RealSense D435
3. Reads the **center-pixel depth** in millimeters
4. Saves the close-up image to `img_inspection/`
5. Appends a row to the report

Output: `final_report.xlsx` (sheets: `Report`, `Summary`, `ByType`)

---

## File Reference

| File | Mode | Description |
|---|---|---|
| `mission1_scan.py` | Real HW | Facade grid scan — Pixhawk + RealSense color |
| `mission1_scanSim.py` | Simulation | Same scan — Gazebo SITL + webcam/dummy feed |
| `mission1_scan_cameratest.py` | Camera only | RealSense test, drone movement simulated |
| `backend_plan_generator.py` | Offline | YOLOv8 inference on Mission 1 images → crack Excel |
| `mission2_solverTSPv2.py` | Offline | TSP solver → optimized visit order Excel |
| `mission2_inspect.py` | Real HW | Close-up inspection — Pixhawk + RealSense color + depth |
| `mission2_inspectSim.py` | Simulation | Same inspection — Gazebo SITL + webcam |
| `mission2_inspect_cameratest.py` | Camera only | RealSense color + depth test, drone movement simulated |

---

## Data Files

| File | Producer | Consumer |
|---|---|---|
| `img/*.jpg` | `mission1_scan.py` | `backend_plan_generator.py` |
| `detections.xlsx` | external / test flow | `mission2_solverTSPv2.py` |
| `mission2_cracks.xlsx` | `backend_plan_generator.py` | `mission2_solverTSPv2.py` |
| `mission2_route.xlsx` | `mission2_solverTSPv2.py` | `mission2_inspect.py` |
| `img_inspection/*.jpg` | `mission2_inspect.py` | — |
| `final_report.xlsx` | `mission2_inspect.py` | — |

---

## Dependencies

```bash
pip install mavsdk pyrealsense2 opencv-python numpy pandas openpyxl ultralytics
```

| Package | Purpose |
|---|---|
| `mavsdk` | Drone control via PX4 (offboard mode, telemetry) |
| `pyrealsense2` | Intel RealSense SDK (color + depth streams) |
| `opencv-python` | Image capture, display, and saving |
| `numpy` | Frame data conversion |
| `pandas` | Excel I/O |
| `openpyxl` | Styled Excel report generation |
| `ultralytics` | YOLOv8 crack detection model |

---

## Safety Features

- **Battery monitor** — background async task; mission aborts if battery drops below 20%
- **Height limit** — user-entered height capped at 15 rows (≈ 30 m) at runtime
- **Offboard failsafe** — if offboard mode fails to start, drone disarms and camera stops cleanly
- **Frame drop handling** — warnings logged if RealSense drops a frame; mission continues

---

## NED Coordinate System

All drone positions use the **NED (North-East-Down)** frame with the launch point as origin.

- **Down is negative** — e.g., `Z = -3.0` means 3 m above ground
- Column traversal moves in the **East** direction (negative = leftward from drone facing wall)
- Row traversal moves in the **Down** direction (scanning up or down each column)

Image filenames encode the exact NED position at capture time, which lets the backend and TSP solver recover drone coordinates without any external log.
