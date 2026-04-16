import asyncio
import cv2
import numpy as np
import os
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def stream_and_sleep(duration, cap):
    """Waits for the drone to move while keeping the video feed live."""
    end_time = asyncio.get_event_loop().time() + duration
    latest_frame = None
    
    while asyncio.get_event_loop().time() < end_time:
        # If webcam works, use it. If not, generate a dummy screen!
        if cap and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                latest_frame = frame.copy()
                cv2.imshow("Ground Station - Live Feed", frame)
        else:
            latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            latest_frame[:] = (50, 50, 100) # Dark blue background
            cv2.putText(latest_frame, "WSL: WEBCAM OFFLINE", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            cv2.putText(latest_frame, "Using Virtual Feed", (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow("Ground Station - Live Feed", latest_frame)

        cv2.waitKey(1)
        await asyncio.sleep(0.03) # ~30 fps refresh rate
        
    return latest_frame

async def monitor_battery(drone, battery_state):
    """Background task to continuously monitor battery life."""
    async for battery in drone.telemetry.battery():
        battery_state["remaining"] = battery.remaining_percent

async def run():
    print("SIM MODE: Initializing Virtual Ground Station...")
    
    # Try to open webcam. It's okay if it fails in WSL!
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("WARNING: Webcam blocked by WSL. Using dummy video feed.")

    await stream_and_sleep(2, cap) 

    # Hardcoded simulator connection
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

    # --- HEIGHT SAFETY LIMIT ---
    while True:
        try:
            y_axis = int(input("Enter height (number of images, max 15): "))
            if y_axis > 15:
                print("Error: Building too tall! Maximum limit is 15 (<30m). Try again.")
            else:
                break
        except ValueError:
            print("Invalid input.")

    x_axis = int(input("Enter width (number of columns): "))
    step_size = 1.0 

    # --- FOLDER CREATION ---
    output_folder = "img"
    os.makedirs(output_folder, exist_ok=True)
    print(f"-- Images will be saved to the '{output_folder}' directory.")

    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    await stream_and_sleep(5, cap) 

    print("-- Setting initial offboard setpoint (Z: -1.0m)")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard mode failed: {error}")
        await drone.action.disarm()
        return

    # STARTING Z-AXIS NOW -1.0
    current_n, current_e, current_d = 0.0, 0.0, -1.0  
    isdown = True
    abort_mission = False

    print("-- Scanning started")
    for i in range(x_axis):
        if abort_mission: break
        
        y_range = range(y_axis) if isdown else reversed(range(y_axis))
        for j in y_range:
            
            # --- BATTERY SAFETY CHECK ---
            if battery_state["remaining"] < 0.20:
                print(f"CRITICAL: Battery low ({battery_state['remaining']*100:.1f}%). Aborting mission!")
                abort_mission = True
                break

            # Let drone move and stabilize, while streaming video
            latest_frame = await stream_and_sleep(2, cap)
            
            # --- NEW NAMING CONVENTION ---
            image_name = f"img_{current_n:.2f}_{current_e:.2f}_{current_d:.2f}.jpg"
            filepath = os.path.join(output_folder, image_name)
            
            print(f"Capturing {image_name}...")
            
            if latest_frame is not None:
                saved_img = latest_frame.copy()
                cv2.putText(saved_img, f"NED: {current_n:.1f}, {current_e:.1f}, {current_d:.1f}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imwrite(filepath, saved_img)
            
            # Move up or down
            current_d += step_size if not isdown else -step_size
            await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, 0.0))
            
        print(f"Moving to next column {i+1}")
        isdown = not isdown 
        
        if i < x_axis - 1 and not abort_mission: 
            current_e -= step_size 
            await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, 0.0))

    print("\n-- Mission ended. Returning to launch point...")
    # RETURN TO HOME AT Z: -1.0
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    await stream_and_sleep(8, cap) # Stream while flying home!

    print("-- Landing now...")
    await drone.offboard.stop()
    await drone.action.land()
    
    if cap: cap.release()
    cv2.destroyAllWindows()
    print("-- Drone landed safely.")

if __name__ == "__main__":
    asyncio.run(run())