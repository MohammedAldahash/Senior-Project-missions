import asyncio
import cv2
import numpy as np
import pandas as pd
import pyrealsense2 as rs
import argparse
import os
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

def load_cracks_from_excel(filename="mission2_route.xlsx"):
    cracks = []
    print(f"Reading crack coordinates from {filename} (Sheet: VisitOrder)...")
    try:
        df = pd.read_excel(filename, sheet_name="VisitOrder")
        for index, row in df.iterrows():
            # Apply the user-provided fix: Skip the depot!
            if row["node_type"] != "CRACK":
                continue  

            cracks.append({
                "id": str(row["crack_id"]).strip(),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"])
            })
        print(f"Successfully loaded {len(cracks)} cracks.")
        return cracks
    except Exception as e:
        print(f"ERROR reading excel file: {e}")
        return []

async def stream_and_sleep(duration, pipeline):
    """Waits for the drone to move while keeping the RealSense color feed live."""
    end_time = asyncio.get_event_loop().time() + duration
    while asyncio.get_event_loop().time() < end_time:
        frames = pipeline.poll_for_frames()
        if frames:
            color_frame = frames.get_color_frame()
            if color_frame:
                frame_data = np.asanyarray(color_frame.get_data())
                cv2.imshow("Ground Station - RealSense Live", frame_data)
                cv2.waitKey(1)
        await asyncio.sleep(0.03) 

async def monitor_battery(drone, battery_state):
    """Background task to continuously monitor battery life."""
    async for battery in drone.telemetry.battery():
        battery_state["remaining"] = battery.remaining_percent

async def run():
    parser = argparse.ArgumentParser(description="Real Hardware: Close-up Crack Inspection")
    parser.add_argument('--port', type=str, default="serial:///dev/ttyACM0:921600", help="Serial port connection")
    args = parser.parse_args()

    target_cracks = load_cracks_from_excel("mission2_route.xlsx")
    if not target_cracks: return

    print("REAL MODE: Starting Intel RealSense pipeline (Color + Depth)...")
    pipeline = rs.pipeline()
    config = rs.config()
    try:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        
        align_to = rs.stream.color
        align = rs.align(align_to)
        await stream_and_sleep(2, pipeline)
    except Exception as e:
        print(f"RealSense failed to initialize: {e}")
        return

    drone = System()
    print(f"Connecting to drone via {args.port}...")
    await drone.connect(system_address=args.port)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    battery_state = {"remaining": 1.0}
    asyncio.create_task(monitor_battery(drone, battery_state))

    output_folder = "img_inspection"
    os.makedirs(output_folder, exist_ok=True)

    print("-- Arming & Taking off")
    await drone.action.arm()
    await drone.action.takeoff()
    await stream_and_sleep(8, pipeline)

    print("-- Starting offboard mode")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard failed: {error}")
        await drone.action.disarm()
        pipeline.stop()
        return

    print("\n-- Starting Close-up Inspection")
    CLOSE_UP_OFFSET = 0.5 
    abort_mission = False

    for crack in target_cracks:
        if abort_mission: break

        if battery_state["remaining"] < 0.20:
            print(f"CRITICAL: Battery low ({battery_state['remaining']*100:.1f}%). Aborting inspection!")
            abort_mission = True
            break

        cid, n, e, d = crack['id'], crack['x'] + CLOSE_UP_OFFSET, crack['y'], crack['z']
        print(f"\n>> Flying to Crack {cid} at [N:{n:.2f}, E:{e:.2f}, D:{d:.2f}]...")
        
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
        await stream_and_sleep(6, pipeline) 
        
        print(f"Capturing color + depth of Crack {cid}...")
        
        # Ensure we get a fresh, aligned set of frames for the actual capture
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if color_frame and depth_frame:
            color_image = np.asanyarray(color_frame.get_data())
            filepath = os.path.join(output_folder, f"crack_{cid}_real_closeup.jpg")
            cv2.imwrite(filepath, color_image)
            
            center_distance = depth_frame.get_distance(320, 240)
            print(f"   [DATA] RealSense reports wall is {center_distance:.2f} meters away.")
        else:
            print("Warning: RealSense dropped a frame during capture!")

    print("\n-- Inspection completed. Returning to launch...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    await stream_and_sleep(8, pipeline)

    print("-- Landing now...")
    await drone.offboard.stop()
    await drone.action.land()
    
    pipeline.stop()
    cv2.destroyAllWindows()
    print("-- Drone landed safely.")

if __name__ == "__main__":
    asyncio.run(run())