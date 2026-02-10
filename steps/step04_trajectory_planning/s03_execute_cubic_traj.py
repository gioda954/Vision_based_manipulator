import numpy as np
import time
import pickle
import matplotlib.pyplot as plt

from core.Robot import Robot
from core.TrajPlanner import TrajPlanner
from steps.step03_inverse_kinematics.s01_ik_geometric_solver import get_ik




def collect_data():
    """
    Collects data for the robot's movement and saves it to a pickle file.
    """
    traj_time = 5  # Trajectory time per segment
    points_num = 998  # Number of intermediate waypoints per segment
    robot = Robot()

    # Define task-space setpoints
    ee_poses = np.array([
        [25, -100, 150, -60],
        [150, 80, 300, 0],
        [250, -115, 75, -45],
        [25, -100, 150, -60]  # Return to start
    ])

    print("Computing IK for waypoints...")
    try:
        joint_angles = np.array([get_ik(pose)[0] for pose in ee_poses])
        print("IK solutions:")
        for i, angles in enumerate(joint_angles):
            print(f"  Waypoint {i+1}: {angles}")
    except ValueError as e:
        raise ValueError(f"End-Effector Pose Unreachable: {e}")

    # Create trajectory in joint space
    print("\nGenerating cubic trajectory...")
    tj = TrajPlanner(joint_angles)
    trajectories = tj.get_cubic_traj(traj_time, points_num)
    
    print(f"Trajectory shape: {trajectories.shape}")
    print(f"Total trajectory time: {trajectories[-1, 0]:.2f} seconds")

    # Calculate time step between trajectory points
    time_step = trajectories[1, 0] - trajectories[0, 0]
    print(f"Time step between points: {time_step*1000:.2f} ms")
    print(f"Command frequency: {1/time_step:.1f} Hz")

    # Pre-allocate data (over-allocate for safety during continuous sampling)
    total_points = len(trajectories)
    max_samples = total_points * 5  # Assume we might sample 5x during execution
    data_time = np.zeros(max_samples)
    data_ee_poses = np.zeros((max_samples, 4))
    data_q = np.zeros((max_samples, 4))
    count = 0

    # Initialize robot
    print("\nInitializing robot...")
    robot.write_motor_state(True)
    # robot.write_profile_acceleration(20)

    # Move to starting position
    print("Moving to start position...")
    robot.write_time(traj_time)
    robot.write_joints(trajectories[0, 1:])
    time.sleep(traj_time)  # Wait for completion

    print("\nExecuting trajectory...")
    robot.write_time(time_step)
    start_time = time.perf_counter()

    # Execute trajectory by streaming commands
    for i in range(1, len(trajectories)):
        # Calculate when this command should be sent
        target_time = start_time + trajectories[i, 0]
        
        # Wait until it's time to send this command
        while time.perf_counter() < target_time:
            # Collect data while waiting
            current_time = time.perf_counter() - start_time
            
            if count < max_samples:
                data_q[count, :] = robot.get_joints_readings()[0, :]
                data_time[count] = current_time
                data_ee_poses[count, :] = robot.get_ee_pos(data_q[count, :])[0:4]
                count += 1
            
            # Small sleep to prevent CPU overload
            time.sleep(0.001)  # 1ms sleep
        
        # Send the command at the scheduled time
        robot.write_joints(trajectories[i, 1:])
    
    total_time = time.perf_counter() - start_time
    print(f"\nTrajectory complete!")
    print(f"Planned time: {trajectories[-1, 0]:.2f}s")
    print(f"Actual time: {total_time:.2f}s")
    print(f"Total samples collected: {count}")
    print(f"Average sample rate: {count/total_time:.1f} Hz")

    # Trim unused space
    data_time = data_time[:count]
    data_ee_poses = data_ee_poses[:count, :]
    data_q = data_q[:count, :]

    # Save data to a picke file (TODO)

def plot_data():
    """
    Loads data from a pickle file and plots it.
    (TODO)
    """


if __name__ == "__main__":
    # Collect data
    collect_data()
    # Plot data
    plot_data()
