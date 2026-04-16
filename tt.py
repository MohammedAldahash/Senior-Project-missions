import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def run():
    # ---------------------------------------------------------
    # 1. INITIALIZE PX4 DRONE
    # ---------------------------------------------------------
    drone = System()
    print("Connecting to simulator via UDP...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered in simulator!")
            break

    # Using smaller numbers for quick simulation testing
    try:
        y_axis = int(input("Enter height (number of images, e.g., 3): "))
        x_axis = int(input("Enter width (number of columns, e.g., 3): "))
    except ValueError:
        print("Invalid input, defaulting to 2x2 grid.")
        y_axis, x_axis = 2, 2

    step_size = 1.0 

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5) 

    print("-- Setting initial offboard setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        await drone.action.disarm()
        return

    current_n = 0.0   
    current_e = 0.0   
    current_d = -1.0  
    isdown = True

    # ---------------------------------------------------------
    # 2. MISSION & MOCK IMAGE CAPTURE LOOP
    # ---------------------------------------------------------
    print("-- Scanning started")
    for i in range(x_axis):
        if isdown:
            for j in range(y_axis):
                await asyncio.sleep(2) # Drone stabilizes
                
                print(f"Capturing img_{i}_{j}.jpg at [N:{current_n}, E:{current_e}, D:{current_d}]")
                
                # --- MOCK CAPTURE: Generate an image with coordinates ---
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, f"Col: {i}, Row: {j}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                cv2.putText(img, f"NED: {current_n}, {current_e}, {current_d}", (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imwrite(f"img_{current_n}_{current_e}_{current_d}.jpg", img)
                # --------------------------------------------------------
                
                current_d -= step_size 
                await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, 0.0))
                
            print(f"Moving to next column {i+1}")
            isdown = False   
        else:
            for j in range(y_axis):
                await asyncio.sleep(2)
                
                print(f"Capturing img_{i}_{y_axis-1-j}.jpg at [N:{current_n}, E:{current_e}, D:{current_d}]")
                
                # --- MOCK CAPTURE ---
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, f"Col: {i}, Row: {y_axis-1-j}", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                cv2.putText(img, f"NED: {current_n}, {current_e}, {current_d}", (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imwrite(f"img_{current_n}_{current_e}_{current_d}.jpg", img)
                # --------------------
                
                current_d += step_size 
                await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, 0.0))
                
            print(f"Moving to next column {i+1}")
            isdown = True
        
        if i < x_axis - 1: 
            current_e -= step_size 
            await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, 0.0))
    # ... (rest of your scanning loop above is unchanged)

    print("-- Scanning completed, returning to launch point...")
    
    # Send the drone back to the exact starting coordinates (0 N, 0 E) 
    # We maintain a safe height (-1.0 D) so it doesn't scrape the ground while flying back
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    
    # Give the drone enough time to physically fly back to the start.
    # If it is a very wide building, you might need to increase this sleep time!
    await asyncio.sleep(8) 

    print("-- Landing now...")
    
    # Safely exit offboard mode and trigger the auto-land sequence
    await drone.offboard.stop()
    await drone.action.land()
    
    print("-- Drone landed safely.")

if __name__ == "__main__":
    asyncio.run(run())