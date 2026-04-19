"""
backend_plan_generator.py
─────────────────────────
Sits between Mission 1 and the TSP solver.

Inputs:
    img/                     — folder of images from Mission 1
                               filenames:  img_{north}_{east}_{down}.jpg
    best.pt                  — YOLOv8 trained weights

Outputs:
    mission2_cracks.xlsx     — Excel file (sheet "Cracks") ready for mission2_solverTSP.py
                               Contains 1 DEPOT row + 1 row per detected crack

Usage:
    python backend_plan_generator.py --images img/ --model best.pt --output mission2_cracks.xlsx

Then run:
    python mission2_solverTSP.py --input mission2_cracks.xlsx --output mission2_route.xlsx
"""

import argparse
import glob
import os
import re
import pandas as pd
from ultralytics import YOLO

# ─── Class names — MUST match training order ───────────────────────────
CLASS_NAMES = [
    "Corrosion", "Settlement", "Thermal Expansion", "Diagonal Shear",
    "Flexural", "Compression", "Tension", "Torsion",
]

CONF_THRESHOLD = 0.25


# ─── Crack offset calculation (from handoff doc) ──────────────────────
def calc_crack_offset(cx_norm: float, cy_norm: float):
    """
    Given the normalized bounding-box center (cx, cy in [0,1]),
    return the crack's (X, Y, Z) offset in meters relative to
    the drone/camera position.

    X = 0.0          (depth — drone faces wall flat)
    Y = horizontal   (left = -0.5 m, right = +0.5 m)
    Z = vertical     (top  = -0.5 m, bottom = -1.5 m)
    """
    x = 0.0
    y = round((cx_norm - 0.5) * 1.0, 4)
    z = round(-(cy_norm * 1.0 + 0.5), 4)
    return x, y, z


# ─── Parse NED from filename ──────────────────────────────────────────
def parse_ned_from_filename(filename: str):
    """
    Extracts (north, east, down) from a filename like:
        img_0.00_-1.00_-3.00.jpg
    Returns tuple of floats or None on failure.
    """
    base = os.path.splitext(os.path.basename(filename))[0]  # img_0.00_-1.00_-3.00
    # Match: img_<float>_<float>_<float>
    pattern = r"^img_([-\d.]+)_([-\d.]+)_([-\d.]+)$"
    m = re.match(pattern, base)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


# ─── Main ─────────────────────────────────────────────────────────────
def run(image_dir: str, model_path: str, output_path: str):
    # 1. Load model
    print(f"Loading YOLOv8 model from: {model_path}")
    model = YOLO(model_path)

    # 2. Collect images
    patterns = [os.path.join(image_dir, f"*.{ext}") for ext in ("jpg", "jpeg", "png")]
    image_files = sorted(sum([glob.glob(p) for p in patterns], []))
    print(f"Found {len(image_files)} images in '{image_dir}'")

    if not image_files:
        print("ERROR: No images found. Did Mission 1 run correctly?")
        return

    # 3. Run inference on every image
    crack_rows = []
    crack_counter = 0
    total_detections = 0

    for img_path in image_files:
        ned = parse_ned_from_filename(img_path)
        if ned is None:
            print(f"  SKIP (bad filename): {img_path}")
            continue

        north, east, down = ned

        results = model.predict(img_path, conf=CONF_THRESHOLD, verbose=False)
        boxes = results[0].boxes

        if len(boxes) == 0:
            print(f"  [CLEAR] {os.path.basename(img_path)}")
            continue

        # Process every detection in this frame
        for box in boxes:
            cls_id = int(box.cls[0])
            confidence = float(box.conf[0])
            crack_type = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f"Unknown_{cls_id}"

            # Bounding box center (normalized)
            xywhn = box.xywhn[0]
            cx_norm = float(xywhn[0])
            cy_norm = float(xywhn[1])

            # Crack offset relative to camera
            off_x, off_y, off_z = calc_crack_offset(cx_norm, cy_norm)

            # Absolute crack position = drone NED + offset
            # north stays the same (drone faces wall, offset X=0)
            # east  += Y offset (horizontal)
            # down  += Z offset (vertical, Z is negative so it goes more negative = higher)
            abs_north = north + off_x   # always +0.0
            abs_east  = east  + off_y
            abs_down  = down  + off_z

            crack_counter += 1
            total_detections += 1
            crack_id = f"C{crack_counter:03d}"

            crack_rows.append({
                "node_id":    crack_id,
                "node_type":  "CRACK",
                "crack_id":   crack_id,
                "x":          round(abs_north, 4),
                "y":          round(abs_east, 4),
                "z":          round(abs_down, 4),
                "crack_type": crack_type,
                "confidence": round(confidence, 4),
                "offset_X":   off_x,
                "offset_Y":   off_y,
                "offset_Z":   off_z,
                "source_img": os.path.basename(img_path),
                "drone_N":    north,
                "drone_E":    east,
                "drone_D":    down,
            })

            print(f"  [CRACK] {os.path.basename(img_path)} → {crack_type} "
                  f"(conf={confidence:.2f}) at NED({abs_north:.2f}, {abs_east:.2f}, {abs_down:.2f})")

    print(f"\n{'='*50}")
    print(f"Total images processed : {len(image_files)}")
    print(f"Total cracks detected  : {total_detections}")
    print(f"{'='*50}")

    if total_detections == 0:
        print("No cracks detected — nothing to write.")
        return

    # 4. Add DEPOT row (launch point: 0, 0, -1.0 — matches Mission 1 & 2 home)
    depot_row = {
        "node_id":    "DEPOT",
        "node_type":  "DEPOT",
        "crack_id":   "DEPOT",
        "x":          0.0,
        "y":          0.0,
        "z":          -1.0,
        "crack_type": "",
        "confidence": 0.0,
        "offset_X":   0.0,
        "offset_Y":   0.0,
        "offset_Z":   0.0,
        "source_img": "",
        "drone_N":    0.0,
        "drone_E":    0.0,
        "drone_D":    0.0,
    }

    all_rows = [depot_row] + crack_rows
    df = pd.DataFrame(all_rows)

    # 5. Write Excel — sheet name "Cracks" to match mission2_solverTSP.py default
    with pd.ExcelWriter(output_path, engine="openpyxl") as writer:
        df.to_excel(writer, sheet_name="Cracks", index=False)

    print(f"\n[OK] Wrote {output_path}  ({len(crack_rows)} cracks + 1 depot)")
    print(f"[NEXT] Run:  python mission2_solverTSP.py --input {output_path} --output mission2_route.xlsx")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Backend: Run YOLOv8 on Mission 1 images → generate crack Excel for TSP")
    ap.add_argument("--images", default="img",        help="Folder of Mission 1 images (default: img)")
    ap.add_argument("--model",  default="best.pt",     help="Path to YOLOv8 weights (default: best.pt)")
    ap.add_argument("--output", default="mission2_cracks.xlsx", help="Output Excel path (default: mission2_cracks.xlsx)")
    ap.add_argument("--conf",   type=float, default=0.25, help="Confidence threshold (default: 0.25)")
    args = ap.parse_args()

    CONF_THRESHOLD = args.conf
    run(args.images, args.model, args.output)
