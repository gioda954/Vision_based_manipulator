import numpy as np
import time
import pickle

from core.Robot import Robot
from steps.step03_inverse_kinematics.s02_ik_solver_with_checks import get_ik


def collect_data():
    """
    Collect data for velocity-based trajectory tracking.
    Moves the robot through a triangular path using velocity control.
    """
    
    # =============================================================================
    # SETUP AND INITIALIZATION
    # =============================================================================
    
    # Create robot object
    robot = Robot()
    
    # Define task-space waypoints [x, y, z, alpha] in mm and degrees
    ee_poses = np.array([
        [25, -100, 150, 0],    # Waypoint 1
        [150, 80, 300, 0],     # Waypoint 2
        [250, -115, 75, 0],    # Waypoint 3
        [25, -100, 150, 0]     # Return to Waypoint 1
    ])
    
    # Store results in joint_angles array (4x4)

    print("Computing IK for waypoints...")
    joint_angles = np.zeros((len(ee_poses), 4))
    prev_solution = None
    for idx, pose in enumerate(ee_poses):
        sol1, sol2 = get_ik(pose)
        if prev_solution is None:
            chosen = sol1
        else:
            diff1 = np.linalg.norm(sol1 - prev_solution)
            diff2 = np.linalg.norm(sol2 - prev_solution)
            chosen = sol1 if diff1 <= diff2 else sol2
        joint_angles[idx] = chosen
        prev_solution = joint_angles[idx]

    print (prev_solution)


    # =============================================================================
    # CONTROL PARAMETERS
    # =============================================================================
    
    velocity_des = 50.0      # Desired task-space speed (mm/s)
    tolerance = 5.0          # Convergence tolerance (mm)
    max_joint_vel = 45.0     # Maximum joint velocity limit (deg/s) for safety
    
    print(f"\nControl parameters:")
    print(f"  Desired velocity: {velocity_des} mm/s")
    print(f"  Tolerance: {tolerance} mm")
    
    # =============================================================================
    # DATA STORAGE PRE-ALLOCATION
    # =============================================================================
    
    # Pre-allocate arrays for data collection (over-allocate for safety)
    max_samples = 10000
    data_time = np.zeros(max_samples)
    data_q = np.zeros((max_samples, 4))              # Joint angles (deg)
    data_q_dot = np.zeros((max_samples, 4))          # Joint velocities (deg/s)
    data_ee_pos = np.zeros((max_samples, 5))         # End-effector pose [x,y,z,pitch,yaw]
    data_ee_vel_cmd = np.zeros((max_samples, 3))    # Commanded EE velocity (mm/s)
    data_ee_vel_actual = np.zeros((max_samples, 6)) # Actual EE velocity (mm/s, rad/s)
    count = 0  # Sample counter
    
    
    # =============================================================================
    # ROBOT INITIALIZATION
    # =============================================================================
    
    print("\nInitializing robot...")
    # TODO: Enable motors
    # YOUR CODE HERE
    
    
    print("Moving to start position...")
    robot.write_mode("position")
    robot.write_joints(joint_angles[0])
    time.sleep(2.0)
    
    
    # TODO: Switch to velocity control mode
    # Hint: Use robot.write_mode("velocity")
    print("\nSwitching to velocity control mode...")
    # YOUR CODE HERE
    
    
    # =============================================================================
    # VELOCITY-BASED TRAJECTORY TRACKING
    # =============================================================================
    
    print("\nStarting velocity-based trajectory tracking...")
    start_time = time.perf_counter()
    
    # Loop through waypoints 2, 3, 4 (indices 1, 2, 3)
    for i in range(1, len(ee_poses)):
        
        # Extract target position (first 3 elements: x, y, z)
        target = ee_poses[i][:3]
        print(f"\n--- Moving to Waypoint {i+1}: {target} ---")
        
        # Initialize distance to target
        distance = np.inf
        iteration = 0
        
        # Continue until within tolerance of target
        while distance > tolerance:
            
            loop_start = time.perf_counter()
            
            # -----------------------------------------------------------------
            # STEP 1: READ CURRENT STATE
            # -----------------------------------------------------------------
            
            # TODO: Read current joint angles and velocities
            # YOUR CODE HERE
            q_deg = None  # Replace with actual reading
            q_dot_deg = None  # Replace with actual reading
            
            # TODO: Convert joint velocities from deg/s to rad/s
            # YOUR CODE HERE
            q_dot_rad = None  # Replace with conversion
            
            # TODO: Get current end-effector pose
            # YOUR CODE HERE
            ee_pose = None  # Replace with actual pose
            current_pos = None  # Extract first 3 elements (x, y, z)
            
            
            # -----------------------------------------------------------------
            # STEP 2: COMPUTE DISTANCE AND DIRECTION TO TARGET
            # -----------------------------------------------------------------
            
            # TODO: Compute error vector (target - current position)
            # YOUR CODE HERE
            error = None  # Replace with calculation
            
            # TODO: Compute distance to target (norm of error vector)
            # YOUR CODE HERE
            distance = None  # Replace with calculation

            # TODO: Compute unit direction vector
            # Hint: direction = error / distance (avoid division by zero)
            # YOUR CODE HERE
            direction = None  # Replace with calculation
            
            # -----------------------------------------------------------------
            # STEP 3: GENERATE DESIRED VELOCITY
            # -----------------------------------------------------------------
            
            # TODO: Scale direction by desired speed
            # YOUR CODE HERE
            speed = velocity_des  # Modify if implementing slowdown
            v_des = None  # Replace with calculation
            
            # TODO: Form 6D desired velocity vector [v_x, v_y, v_z, omega_x, omega_y, omega_z]
            # Hint: Stack v_des with zeros for angular velocity
            # YOUR CODE HERE
            p_dot_des = None  # Replace with 6x1 vector
            
            
            # -----------------------------------------------------------------
            # STEP 4: INVERSE VELOCITY KINEMATICS
            # -----------------------------------------------------------------
            
            # TODO: Get Jacobian at current configuration
            # YOUR CODE HERE
            J = None  # Replace with Jacobian
            
            # TODO: Compute pseudo-inverse of Jacobian
            # YOUR CODE HERE
            J_pinv = None  # Replace with pseudo-inverse
            
            # TODO: Compute required joint velocities (rad/s)
            # YOUR CODE HERE
            q_dot_cmd_rad = None  # Replace with calculation
            
            # TODO: Convert joint velocities from rad/s to deg/s
            # YOUR CODE HERE
            q_dot_cmd_deg = None  # Replace with conversion
            
            
            # -----------------------------------------------------------------
            # STEP 5: SEND VELOCITY COMMAND TO ROBOT
            # -----------------------------------------------------------------
            
            # TODO: Send velocity command to robot
            # Hint: Use robot.write_velocities(q_dot_cmd_deg)
            # YOUR CODE HERE
            
            
            # -----------------------------------------------------------------
            # STEP 6: VERIFY WITH FORWARD VELOCITY KINEMATICS
            # -----------------------------------------------------------------
            
            # TODO: Compute actual end-effector velocity
            # Hint: Use robot.get_fwd_vel_kin(...)
            # YOUR CODE HERE
            p_dot_actual = None  # Replace with calculation
            
            
            # -----------------------------------------------------------------
            # STEP 7: DATA COLLECTION
            # -----------------------------------------------------------------
            
            if count < max_samples:
                data_time[count] = time.perf_counter() - start_time
                data_q[count, :] = q_deg
                data_q_dot[count, :] = q_dot_deg
                data_ee_pos[count, :] = ee_pose
                data_ee_vel_cmd[count, :] = v_des
                data_ee_vel_actual[count, :] = p_dot_actual
                count += 1
            
            iteration += 1
            
        
        # End of while loop - target reached
        print(f"  Reached Waypoint {i+1}! Final distance: {distance:.2f} mm")
        
        # TODO: Stop robot briefly at waypoint
        # Hint: Send zero velocities and sleep briefly
        # YOUR CODE HERE
    
    
    # =============================================================================
    # CLEANUP AND DATA SAVING
    # =============================================================================
    
    # TODO: Stop robot completely
    print("\nTrajectory complete! Stopping robot...")
    # YOUR CODE HERE
    
    total_time = time.perf_counter() - start_time
    print(f"\nTotal execution time: {total_time:.2f} s")
    print(f"Total samples collected: {count}")
    print(f"Average sample rate: {count/total_time:.1f} Hz")
    
    # Trim unused portions of pre-allocated arrays
    data_time = data_time[:count]
    data_q = data_q[:count, :]
    data_q_dot = data_q_dot[:count, :]
    data_ee_pos = data_ee_pos[:count, :]
    data_ee_vel_cmd = data_ee_vel_cmd[:count, :]
    data_ee_vel_actual = data_ee_vel_actual[:count, :]
    
    # TODO: Save all data to pickle file
    filename='lab5_4_data.pkl'
    # Create dictionary with all collected data and control parameters
    print(f"\nSaving data to {filename}...")
    # YOUR CODE HERE
    data_dict = {
        'time': data_time,
        'joint_angles': data_q,
        # Add remaining data fields
    }
    
    # TODO: Write dictionary to pickle file
    # YOUR CODE HERE
    
    print("Data saved successfully!")

def plot_data():
    """
    Load data and create required plots.
    """

if __name__ == "__main__":
    # Run data collection
    collect_data()
    # Plot data
    plot_data()
