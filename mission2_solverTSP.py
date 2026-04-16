import math
import itertools
import argparse
from typing import List, Tuple
import pandas as pd

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

def solve_from_excel(input_path: str, output_path: str, cruise_speed_mps: float = 3.0, sheet_name: str = "Cracks") -> None:
    df = pd.read_excel(input_path, sheet_name=sheet_name)
    req = {"node_id","node_type","x","y","z"}
    miss = req - set(df.columns)
    if miss:
        raise ValueError(f"Missing required columns: {miss}")
    df["node_type"] = df["node_type"].astype(str).str.upper().str.strip()

    depots = df[df["node_type"]=="DEPOT"]
    if len(depots)!=1:
        raise ValueError(f"Need exactly 1 DEPOT row, found {len(depots)}")
    df = df.reset_index(drop=True)
    depot_idx = int(depots.index[0])

    coords = list(zip(df["x"].astype(float), df["y"].astype(float), df["z"].astype(float)))
    dist = build_distance_matrix(coords)

    crack_indices = [i for i in range(len(df)) if i != depot_idx]
    num_cracks = len(crack_indices)

    if num_cracks <= 10:
        route_idx = solve_tsp_exact(depot_idx, crack_indices, dist)
        method = f"Exact (permutations), {num_cracks}!"
    else:
        route_idx = two_opt(nearest_neighbor_tour(len(df), depot_idx, dist), dist)
        method = "Heuristic (Nearest Neighbor + 2-opt)"

    # VisitOrder
    visit_rows = []
    for order, idx in enumerate(route_idx):
        visit_rows.append({
            "visit_order": order,
            "node_id": df.loc[idx, "node_id"],
            "node_type": df.loc[idx, "node_type"],
            "x": df.loc[idx, "x"],
            "y": df.loc[idx, "y"],
            "z": df.loc[idx, "z"],
            "crack_id": df.loc[idx, "crack_id"] if "crack_id" in df.columns else None,
            "severity": df.loc[idx, "severity"] if "severity" in df.columns else None,
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
            "step": step+1,
            "from_node": df.loc[i, "node_id"],
            "to_node": df.loc[j, "node_id"],
            "from_type": df.loc[i, "node_type"],
            "to_type": df.loc[j, "node_type"],
            "distance_m": round(d_ij, 3),
            "est_flight_time_s": round(t_ij, 3),
            "cum_distance_m": round(cum_d, 3),
            "cum_time_s": round(cum_t, 3),
        })
    df_route = pd.DataFrame(legs)

    total_d = route_length(route_idx, dist)
    total_t = total_d / cruise_speed_mps
    df_summary = pd.DataFrame([{
        "solver_method": method,
        "num_cracks": num_cracks,
        "cruise_speed_mps": cruise_speed_mps,
        "total_distance_m": round(total_d, 3),
        "est_total_flight_time_s": round(total_t, 3),
    }])

    with pd.ExcelWriter(output_path, engine="openpyxl") as writer:
        df_route.to_excel(writer, sheet_name="Route", index=False)
        df_visit.to_excel(writer, sheet_name="VisitOrder", index=False)
        df_summary.to_excel(writer, sheet_name="Summary", index=False)

    print(f"[OK] {method}")
    print(f"[OK] Wrote: {output_path}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--sheet", default="Cracks")
    args = ap.parse_args()
    solve_from_excel(args.input, args.output, args.speed, args.sheet)
