# lab3_3.py
"""
EE 471 – Lab 3 Part 3
Validate inverse kinematics on OpenManipulator-X using task-space waypoints.

This version:
  • Computes IK for each waypoint
  • Commands the robot using the valid IK solution
  • Records elapsed time, joint angles, and end-effector pose
  • Saves the data to a pickle (.pkl) file for analysis
"""

from classes.Robot import Robot as rb
from prelab3 import get_ik
import numpy as np
import time
import pickle
from pathlib import Path
robot = rb()

# ----------------------------
# Joint limits (degrees)
# ----------------------------
joint_limits = np.array([
    [-90,   90],   # Joint 1
    [-120,  90],   # Joint 2
    [ -90,  75],   # Joint 3
    [-100, 100],   # Joint 4
])

def within_limits(joint_angles_deg):
    """Check that all angles are within defined limits."""
    for i, angle in enumerate(joint_angles_deg):
        if not (joint_limits[i,0] <= angle <= joint_limits[i,1]):
            return False
    return True

# ----------------------------
# Waypoints (x, y, z, α)
# ----------------------------
waypoints = [
    [25,  -100, 150, -60],   # 1
    [150,   80, 300,   0],   # 2
    [250, -115,  75, -45],   # 3
    [25,  -100, 150, -60],   # 4 (return)
]

# ----------------------------
# Main experiment
# ----------------------------
def main():
    traj_time = 5.0     # seconds between waypoints
    poll_dt   = 0.5     # log every 0.1s (10 Hz)
    filename  = "lab3_ik_run.pkl"

    # Initialize robot
    
    robot.write_motor_state(True)
    robot.write_time(traj_time)

    print("Homing to [0, 0, 0, 0] deg ...")
    robot.write_joints([0, 0, 0, 0])
    time.sleep(traj_time)

    # List for all logged data
    all_data = []

    # Go through each waypoint
    for i, pose in enumerate(waypoints, start=1):
        print(f"\n=== Waypoint {i}: {pose} ===")

        # Compute IK
        try:
            sol1, sol2 = get_ik(pose)
        except Exception as e:
            print(f"❌ IK failed for waypoint {i}: {e}")
            continue

        # Pick valid solution
        if within_limits(sol1):
            target = sol1
        elif within_limits(sol2):
            target = sol2
        else:
            print(f"⚠️ No valid IK solution within joint limits for waypoint {i}")
            continue

        print(f"Moving to joint angles: {np.round(target,3)} deg")

        # Command robot
        robot.write_joints(target.tolist())

        # Record motion data
        t_start = time.perf_counter()
        while time.perf_counter() - t_start < traj_time:
            t = time.perf_counter() - t_start
            joints = robot.get_joints_readings()[0, :4].tolist() # [q1, q2, q3, q4]
            ee = robot.get_ee_pos(joints)               # [x, y, z, α]
            all_data.append({
                "t_sec": t,
                "joints_deg": joints,
                "ee_pos": ee
            })
            time.sleep(poll_dt)

    # Save everything
    out_path = Path(filename).resolve()
    with open(out_path, "wb") as f:
        pickle.dump(all_data, f)

    print(f"\n✅ Data saved to {out_path}")
    print("Run `plot_lab3.py` to visualize the results.")

# ----------------------------
# Entry point
# ----------------------------

if __name__ == "__main__":
    main()