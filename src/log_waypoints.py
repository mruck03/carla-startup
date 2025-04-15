import carla

import asyncio

import time

import math
from datetime import datetime



def is_close(current, goal, tolerance):
    return abs(current - goal) <= tolerance



def is_within_goal(pose, yaw, goal_location, goal_yaw, pos_tol=0.5, yaw_tol=20.0):

    x_close = is_close(pose.x, goal_location.x, pos_tol)
    y_close = is_close(pose.y, goal_location.y, pos_tol)
    z_close = is_close(pose.z, goal_location.z, pos_tol)
    # yaw_diff = (yaw - goal_yaw + 180) % 360 - 180
    # yaw_close = abs(yaw_diff) <= yaw_tol

    return x_close and y_close and z_close # and yaw_close





async def log_vehicle_until_goal(vehicle, goal_location, goal_yaw, filename=None, timeout=60.0):

    if filename is None:
        filename = f"waypoints_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"


    start_time = time.time()
    total_log_line = 0 

    with open(filename, 'a') as f:

        while True:

            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation

            total_log_line += 1
            log_line = f"{location.x:.3f}, {location.y:.3f}, {location.z:.3f}, {rotation.pitch:.2f}, {rotation.yaw:.2f}, {rotation.roll:.2f}\n"
            f.write(log_line)



            if is_within_goal(transform, goal_location, goal_yaw):
                print("Reached goal.")
                break



            if (time.time() - start_time) > timeout:
                print("Timeout reached.")
                break

            await asyncio.sleep(0.05) 

    print(f"Logger finished. Total logged line: {total_log_line}")