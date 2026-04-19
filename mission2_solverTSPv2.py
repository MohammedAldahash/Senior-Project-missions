"""
mission2_solverTSP.py
─────────────────────
Reads detections.xlsx from test_flow.py, solves TSP, outputs mission2_route.xlsx.

Flow:
  test_flow.py → detections.xlsx → THIS SCRIPT → mission2_route.xlsx → mission2_inspect.py

How it works:
  1. Reads "Detections" sheet from detections.xlsx
  2. Filters rows where crack_detected == True
  3. Parses drone NED coordinates from image_name (format: img_N_E_D.jpg)
  4. Combines drone NED + crack offsets (X, Y, Z) → absolute crack position
  5. Solves TSP (exact if ≤10 cracks, nearest-neighbor + 2-opt otherwise)
  6. Writes mission2_route.xlsx with 3 sheets: Route, VisitOrder, Summary

Usage:
  python mission2_solverTSP.py --input detections.xlsx --output mission2_route.xlsx
"""

import math
import itertools
import re
import os
from typing import List, Tuple
import pandas as pd


# ─── TSP core functions ──────────────────────────────────────────────

def euclidean(a: Tuple[float,float,float], b: Tuple[float,float,float]) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def build_distance_matrix(coords: List[Tuple[float,float,float]]) -> List[List[float]]:
    n = len(coords)
    dist = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            d = euclidean(coords[i], coords[j])
            dist[i][j] = d
            dist[j][i] = d
    return dist

def route_length(route: List[int], dist: List[List[float]]) -> float:
    return sum(dist[route[k]][route[k+1]] for k in range(len(route)-1))

def solve_tsp_exact(depot_idx: int, crack_indices: List[int], dist: List[List[float]]) -> List[int]:
    best_route = None
    best_len = float("inf")
    for perm in itertools.permutations(crack_indices):
        route = [depot_idx] + list(perm) + [depot_idx]
        L = route_length(route, dist)
        if L < best_len:
            best_len = L
            best_route = route
    return best_route

def nearest_neighbor_tour(n: int, depot_idx: int, dist: List[List[float]]) -> List[int]:
    unvisited = set(range(n))
    unvisited.remove(depot_idx)
    route = [depot_idx]
    cur = depot_idx
    while unvisited:
        nxt = min(unvisited, key=lambda j: dist[cur][j])
        route.append(nxt)
        unvisited.remove(nxt)
        cur = nxt
    route.append(depot_idx)
    return route

def two_opt(route: List[int], dist: List[List[float]], max_iters: int = 2000) -> List[int]:
    best = route[:]
    best_len = route_length(best, dist)
    n = len(best)
    improved = True
    iters = 0
    while improved and iters < max_iters:
        improved = False
        iters += 1
        for i in range(1, n-3):
            for k in range(i+1, n-2):
                new = best[:i] + best[i:k+1][::-1] + best[k+1:]
                new_len = route_length(new, dist)
                if new_len + 1e-9 < best_len:
                    best, best_len = new, new_len
                    improved = True
                    break
            if improved:
                break
    return best


# ─── Parse drone NED from image filename ─────────────────────────────

def parse_ned_from_filename(filename: str):
    """
    Extracts (north, east, down) from filename like:
        img_0.00_-1.00_-3.00.jpg
        img_0.00_-1.00_-3.00.png
    Returns (north, east, down) or None if format doesn't match.
    """
    base = os.path.splitext(os.path.basename(str(filename)))[0]
    pattern = r"^img_([-\d.]+)_([-\d.]+)_([-\d.]+)$"
    m = re.match(pattern, base)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


# ─── Main solver ─────────────────────────────────────────────────────

def solve_from_excel(input_path: str, output_path: str, cruise_speed_mps: float = 3.0) -> None:
    print(f"[INFO] Reading: {input_path}")
    df = pd.read_excel(input_path, sheet_name="Detections")

    # Filter only detected cracks
    cracks = df[df["crack_detected"] == True].reset_index(drop=True)
    print(f"  Found {len(cracks)} cracks out of {len(df)} images")

    if len(cracks) == 0:
        print("ERROR: No cracks detected — nothing to route.")
        return

    # Build internal table: DEPOT + cracks with absolute positions
    rows = []

    # DEPOT row — drone home (0, 0, -1.0) matches Mission 1 & 2
    rows.append({
        "node_id": "DEPOT", "node_type": "DEPOT", "crack_id": "DEPOT",
        "x": 0.0, "y": 0.0, "z": -1.0,
        "crack_type": "", "confidence": 0.0, "image_name": "",
    })

    skipped = 0
    for i, row in cracks.iterrows():
        ned = parse_ned_from_filename(row["image_name"])
        if ned is None:
            print(f"  WARNING: Cannot parse NED from '{row['image_name']}' — skipping this crack")
            skipped += 1
            continue

        drone_n, drone_e, drone_d = ned
        off_x, off_y, off_z = float(row["X"]), float(row["Y"]), float(row["Z"])

        # Absolute crack position = drone NED + crack offset
        abs_x = round(drone_n + off_x, 4)    # North + X (always 0)
        abs_y = round(drone_e + off_y, 4)    # East  + Y (horizontal)
        abs_z = round(drone_d + off_z, 4)    # Down  + Z (vertical)

        cid = f"C{len(rows):03d}"
        rows.append({
            "node_id": cid, "node_type": "CRACK", "crack_id": cid,
            "x": abs_x, "y": abs_y, "z": abs_z,
            "crack_type": str(row["crack_type"]),
            "confidence": float(row["confidence"]),
            "image_name": str(row["image_name"]),
        })

    num_cracks = len(rows) - 1  # minus DEPOT
    if num_cracks == 0:
        print("ERROR: No valid cracks after parsing filenames — check image_name format (expected: img_N_E_D.jpg)")
        return

    if skipped > 0:
        print(f"  Skipped {skipped} cracks (bad filename format)")

    print(f"  Processing {num_cracks} cracks for TSP...")

    df_internal = pd.DataFrame(rows).reset_index(drop=True)
    depot_idx = 0

    coords = list(zip(df_internal["x"].astype(float), df_internal["y"].astype(float), df_internal["z"].astype(float)))
    dist = build_distance_matrix(coords)

    crack_indices = [i for i in range(len(df_internal)) if i != depot_idx]

    if num_cracks <= 10:
        route_idx = solve_tsp_exact(depot_idx, crack_indices, dist)
        method = f"Exact (permutations), {num_cracks}!"
    else:
        route_idx = two_opt(nearest_neighbor_tour(len(df_internal), depot_idx, dist), dist)
        method = "Heuristic (Nearest Neighbor + 2-opt)"

    # VisitOrder sheet
    visit_rows = []
    for order, idx in enumerate(route_idx):
        visit_rows.append({
            "visit_order": order,
            "node_id":     df_internal.loc[idx, "node_id"],
            "node_type":   df_internal.loc[idx, "node_type"],
            "x":           df_internal.loc[idx, "x"],
            "y":           df_internal.loc[idx, "y"],
            "z":           df_internal.loc[idx, "z"],
            "crack_id":    df_internal.loc[idx, "crack_id"],
            "crack_type":  df_internal.loc[idx, "crack_type"],
            "confidence":  df_internal.loc[idx, "confidence"],
            "image_name":  df_internal.loc[idx, "image_name"],
        })
    df_visit = pd.DataFrame(visit_rows)

    # Route legs
    legs = []
    cum_d = 0.0
    cum_t = 0.0
    for step in range(len(route_idx)-1):
        i = route_idx[step]
        j = route_idx[step+1]
        d_ij = dist[i][j]
        t_ij = d_ij / cruise_speed_mps
        cum_d += d_ij
        cum_t += t_ij
        legs.append({
            "step":              step+1,
            "from_node":         df_internal.loc[i, "node_id"],
            "to_node":           df_internal.loc[j, "node_id"],
            "from_type":         df_internal.loc[i, "node_type"],
            "to_type":           df_internal.loc[j, "node_type"],
            "distance_m":        round(d_ij, 3),
            "est_flight_time_s": round(t_ij, 3),
            "cum_distance_m":    round(cum_d, 3),
            "cum_time_s":        round(cum_t, 3),
        })
    df_route = pd.DataFrame(legs)

    # Summary
    total_d = route_length(route_idx, dist)
    total_t = total_d / cruise_speed_mps
    df_summary = pd.DataFrame([{
        "solver_method":            method,
        "num_cracks":               num_cracks,
        "cruise_speed_mps":         cruise_speed_mps,
        "total_distance_m":         round(total_d, 3),
        "est_total_flight_time_s":  round(total_t, 3),
    }])

    with pd.ExcelWriter(output_path, engine="openpyxl") as writer:
        df_route.to_excel(writer, sheet_name="Route", index=False)
        df_visit.to_excel(writer, sheet_name="VisitOrder", index=False)
        df_summary.to_excel(writer, sheet_name="Summary", index=False)

    print(f"[OK] {method}")
    print(f"[OK] Total distance: {total_d:.3f} m | Est. flight time: {total_t:.1f} s")
    print(f"[OK] Wrote: {output_path}")


if __name__ == "__main__":
    INPUT_FILE  = "detections.xlsx"
    OUTPUT_FILE = "mission2_route.xlsx"
    CRUISE_SPEED = 3.0

    solve_from_excel(INPUT_FILE, OUTPUT_FILE, CRUISE_SPEED)
