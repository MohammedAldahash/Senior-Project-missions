import asyncio
import cv2
import numpy as np
import pandas as pd
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
        print(f"Successfully loaded {len(cracks)} cracks in optimized order.")
        return cracks
    except Exception as e:
        print(f"ERROR reading excel file: {e}")
        return []

async def stream_and_sleep(duration, cap):
    """Waits for the drone to move while keeping the video feed live."""
    end_time = asyncio.get_event_loop().time() + duration
    latest_frame = None
    
    while asyncio.get_event_loop().time() < end_time:
        if cap and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                latest_frame = frame.copy()
                cv2.imshow("Ground Station - Live Feed", frame)
        else:
            latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            latest_frame[:] = (50, 50, 100) 
            cv2.putText(latest_frame, "WSL: WEBCAM OFFLINE", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            cv2.putText(latest_frame, "Using Virtual Feed", (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow("Ground Station - Live Feed", latest_frame)

        cv2.waitKey(1)
        await asyncio.sleep(0.03) 
        
    return latest_frame

async def monitor_battery(drone, battery_state):
    """Background task to continuously monitor battery life."""
    async for battery in drone.telemetry.battery():
        battery_state["remaining"] = battery.remaining_percent

async def run():
    target_cracks = load_cracks_from_excel("mission2_route.xlsx")
    if not target_cracks: return

    print("SIM MODE: Starting laptop webcam for live ground feed...")
    cap = cv2.VideoCapture(0)
    await stream_and_sleep(2, cap) 

    drone = System()
    print("Connecting to Gazebo Simulator (udp://:14540)...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    # Start battery monitor in the background
    battery_state = {"remaining": 1.0}
    asyncio.create_task(monitor_battery(drone, battery_state))

    output_folder = "img_inspection"
    os.makedirs(output_folder, exist_ok=True)

    print("-- Arming & Taking off")
    await drone.action.arm()
    await drone.action.takeoff()
    await stream_and_sleep(5, cap)

    # Note: Starting at -1.0 Z to match Mission 1
    print("-- Starting offboard mode")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard failed: {error}")
        await drone.action.disarm()
        return

    print("\n-- Starting Close-up Inspection")
    CLOSE_UP_OFFSET = 0.5 
    abort_mission = False

    for crack in target_cracks:
        if abort_mission: break

        # --- BATTERY SAFETY CHECK ---
        if battery_state["remaining"] < 0.20:
            print(f"CRITICAL: Battery low ({battery_state['remaining']*100:.1f}%). Aborting inspection!")
            abort_mission = True
            break

        cid, n, e, d = crack['id'], crack['x'] + CLOSE_UP_OFFSET, crack['y'], crack['z']
        print(f"\n>> Flying to Crack {cid} at [N:{n:.2f}, E:{e:.2f}, D:{d:.2f}]...")
        
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
        
        # Let drone move while streaming
        latest_frame = await stream_and_sleep(6, cap) 
        
        print(f"Capturing webcam image of Crack {cid}...")
        image_name = f"crack_{cid}_sim_closeup.jpg"
        filepath = os.path.join(output_folder, image_name)
        
        if latest_frame is not None:
            saved_img = latest_frame.copy()
            cv2.putText(saved_img, f"CLOSE UP: Crack {cid}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(filepath, saved_img)

    print("\n-- Inspection completed. Returning to launch...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    await stream_and_sleep(8, cap)

    print("-- Landing now...")
    await drone.offboard.stop()
    await drone.action.land()
    
    if cap: cap.release()
    cv2.destroyAllWindows()
    print("-- Drone landed safely.")

if __name__ == "__main__":
    asyncio.run(run())