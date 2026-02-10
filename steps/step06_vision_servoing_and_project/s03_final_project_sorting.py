"""
Lab 8 (Final Project): Vision-Guided Robotic Pick-and-Place Sorting System
Team: [Your Team Name]
Members: [Team Member Names]

This script implements a complete robotic sorting system that:
1. Detects colored balls using computer vision
2. Localizes them in 3D space using camera-robot calibration
3. Plans smooth trajectories to pick up each ball
4. Sorts them into color-coded bins

System Architecture:
    Detection → Localization → Motion Planning → Execution → Repeat
"""

import numpy as np
import cv2
import time
from pathlib import Path

from core.Robot import Robot
from core.Realsense import Realsense
from core.TrajPlanner import TrajPlanner
from steps.step03_inverse_kinematics.s01_ik_geometric_solver import get_ik

# ============================================================================
# CONFIGURATION PARAMETERS
# ============================================================================

# Physical parameters
BALL_RADIUS = 15  # Physical radius of balls in millimeters

# Motion control parameters
TRAJECTORY_TIME = 2.5  # Time for each trajectory segment in seconds
NUM_POINTS = 100       # Number of waypoints in each trajectory

# Workspace safety bounds (millimeters, in robot frame)
# TODO: Adjust these based on your setup to prevent collisions
X_MIN, X_MAX = 50, 230   # Forward/backward limits
Y_MIN, Y_MAX = -150, 150 # Left/right limits

# Home position: [x, y, z, pitch] in mm and degrees
# This position should give the camera a clear view of the workspace
HOME_POSITION = [100, 0, 220, -15]

# Sorting bin locations: [x, y, z, pitch] in mm and degrees
# TODO: Adjust these positions based on your physical bin locations
BINS = {
    'red': [0, -220, 150, -40],
    'orange': [120, -220, 150, -40],
    'blue': [0, 220, 150, -45],
    'yellow': [120, 220, 150, -45]
}

# ============================================================================
# COMPUTER VISION: BALL DETECTION AND POSE ESTIMATION
# ============================================================================

def get_ball_pose(corners: np.ndarray, intrinsics: any, radius: float) -> tuple:
    """
    Estimate the 3D pose of a detected sphere using the Perspective-n-Point (PnP) algorithm.
    
    The PnP algorithm finds the position and orientation of an object by matching:
    - Known 3D points on the object (object_points)
    - Corresponding 2D points in the image (image_points)
    
    For a sphere, we create artificial "corner" points on the sphere's visible boundary
    to establish these correspondences.
    
    Args:
        corners: A 4x2 array of circle boundary points [top, bottom, left, right]
        intrinsics: Camera intrinsic parameters from RealSense
        radius: The sphere's physical radius in millimeters
    
    Returns:
        tuple: (rotation_matrix, translation_vector)
            - rotation_matrix: 3x3 matrix (not used for spheres, but returned by PnP)
            - translation_vector: 3x1 vector giving sphere center in camera frame (mm)
    
    Raises:
        RuntimeError: If PnP algorithm fails to find a solution
    """
    object_points = np.array([
        [0, radius, 0],
        [0, -radius, 0],
        [-radius, 0, 0],
        [radius, 0, 0],
    ], dtype=np.float32)
    
    image_points = corners.reshape(4, 2).astype(np.float32)

    camera_matrix = np.array([
        [intrinsics.fx, 0.0, intrinsics.ppx],
        [0.0, intrinsics.fy, intrinsics.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)

    dist_coeffs = np.asarray(getattr(intrinsics, "coeffs", [0, 0, 0, 0, 0]), dtype=np.float32)
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    if not success:
        raise RuntimeError("PnP pose estimation failed")

    rot_matrix, _ = cv2.Rodrigues(rvec)

    return rot_matrix, tvec


def detect_balls(image):
    """
    Detect colored balls in the input image using computer vision.
    
    Pipeline:
        1. Convert image to HSV color space for robust color detection
        2. Detect circular shapes using Hough Circle Transform
        3. For each detected circle:
           - Extract the color by analyzing hue values
           - Classify as red, orange, yellow, or blue
           - Record position and radius
    
    Args:
        image: BGR color image from camera
    
    Returns:
        list: List of tuples (color, (cx, cy), radius) for each detected ball
              Returns None if no balls detected
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=30,
        param1=120,
        param2=30,
        minRadius=8,
        maxRadius=120,
    )
    
    
    # If no circles detected, show image and return None
    if circles is None or len(circles[0]) == 0:
        cv2.imshow('Detection', image)
        cv2.waitKey(1)
        return None
    
    # Convert circle parameters to integers
    circles = np.uint16(np.around(circles))
    
    result = []  # List to store (color, center, radius) tuples
    
    # Process each detected circle
    for circle in circles[0, :]:
        center = (circle[0], circle[1])  # (cx, cy) in pixels
        radius = circle[2]               # radius in pixels

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        hue = cv2.mean(hsv[:, :, 0], mask=mask)[0]
        sat = cv2.mean(hsv[:, :, 1], mask=mask)[0]

        if sat < 50:
            continue
        if hue < 10 or hue >= 170:
            color = 'red'
        elif 10 <= hue < 22:
            color = 'orange'
        elif 22 <= hue < 35:
            color = 'yellow'
        elif 90 <= hue < 135:
            color = 'blue'
        else:
            continue
        
        # Skip if color not recognized
        if color is None:
            continue
        
        # Add to results
        result.append((color, center, radius))
        
        cv2.circle(image, center, radius, (0, 255, 0), 2)  # Green circle
        cv2.circle(image, center, 3, (0, 0, 255), -1)      # Red center dot
        cv2.putText(image, color, (center[0] - 20, center[1] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display detection results
    cv2.imshow('Detection', image)
    cv2.waitKey(1)
    
    return result if result else None


# ============================================================================
# MOTION CONTROL: TRAJECTORY PLANNING AND EXECUTION
# ============================================================================

def move_trajectory(robot, target_pos, traj_time=TRAJECTORY_TIME):
    """
    Move robot to target position using smooth quintic trajectory.
    
    This function:
    1. Gets current robot position
    2. Plans a quintic (5th-order polynomial) trajectory in task space
    3. Converts entire trajectory to joint space using inverse kinematics
    4. Executes trajectory with precise timing
    
    Args:
        robot: Robot instance
        target_pos: Target position [x, y, z, pitch] in mm and degrees
        traj_time: Duration of trajectory in seconds
    """
    current_joints = robot.get_joints_readings()[0]
    current_pos = robot.get_ee_pos(current_joints)
    setpoints = np.vstack((current_pos, target_pos))
    planner = TrajPlanner(setpoints)
    task_traj = planner.get_quintic_traj(traj_time, NUM_POINTS)
    
    
    # ==========================================================================
    joint_traj = np.asarray([get_ik(pose)[0] for pose in task_traj[:, 1:]], dtype=float)
    
    
    dt = float(task_traj[1, 0] - task_traj[0, 0])
    robot.write_time(dt)
    
    # ==========================================================================
    # Execute trajectory with precise timing (see Lab 4)
    # ==========================================================================
    start_time = time.time()
    for t, joints in zip(task_traj[:, 0], joint_traj):
        target = start_time + t
        remaining = target - time.time()
        if remaining > 0:
            time.sleep(remaining)
        robot.write_joints(joints)


# ============================================================================
# PICK AND PLACE OPERATIONS
# ============================================================================

def pick_ball(robot, ball_pos):
    """
    Execute a pick operation to grasp a ball.
    
    Sequence:
        1. Open gripper
        2. Move to approach position (above ball)
        3. Move down to grasp position
        4. Close gripper
        5. Lift ball to clear workspace
    
    Args:
        robot: Robot instance
        ball_pos: Ball position [x, y, z] in robot frame (mm)
    """
    print(f"Picking ball at {ball_pos}")
    
    # ==========================================================================
    # Open gripper
    # ==========================================================================
    # Use robot.write_gripper(1) for open, wait for motion
    robot.write_gripper(1)
    time.sleep(0.5)
    
    # ==========================================================================
    # Move to approach position (above the ball)
    # ==========================================================================
    # Create position [x, y, z_high, pitch] where z_high is ~100mm
    # Use steep pitch angle (e.g., -80°) for vertical approach
    approach = [ball_pos[0], ball_pos[1], 100, -80]   # may need tuning
    move_trajectory(robot, approach, TRAJECTORY_TIME)
    
    # ==========================================================================
    # Move down to grasp position
    # ==========================================================================
    # Lower z to just above table surface (e.g., 39mm for 30mm ball radius)
    # Adjust this height based on your table height and ball size!
    grasp = [ball_pos[0], ball_pos[1], 39, -80]       # may need tuning
    move_trajectory(robot, grasp, TRAJECTORY_TIME * 0.5)
    
    # ==========================================================================
    # Close gripper to grasp ball
    # ==========================================================================
    #  Use robot.write_gripper(0) for close, wait for secure grasp
    robot.write_gripper(0)
    time.sleep(1)
    
    # ==========================================================================
    # Lift ball to clear workspace
    # ==========================================================================
    # Move back up to approach height
    lift = [ball_pos[0], ball_pos[1], 100, -80] # may need tuning
    move_trajectory(robot, lift, TRAJECTORY_TIME * 0.5)


def place_ball(robot, color):
    """
    Place ball in the appropriate color-coded bin.
    
    Args:
        robot: Robot instance
        color: Ball color string ('red', 'orange', 'yellow', 'blue')
    """
    print(f"Placing {color} ball")
    
    # ==========================================================================
    # Get bin position for this color
    # ==========================================================================
    # Look up position in BINS dictionary
    bin_pos = BINS[color]
    
    # ==========================================================================
    # Move to bin location
    # ==========================================================================
    move_trajectory(robot, bin_pos, TRAJECTORY_TIME)
    
    # ==========================================================================
    # Release ball by opening gripper
    # ==========================================================================
    robot.write_gripper(1)
    time.sleep(1)


def go_home(robot):
    """
    Return robot to home position for next detection cycle.
    
    Args:
        robot: Robot instance
    """
    move_trajectory(robot, HOME_POSITION, TRAJECTORY_TIME)


# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def main():
    """
    Main control loop for the robotic sorting system.
    
    Workflow:
        1. Initialize robot, camera, and calibration
        2. Move to home position
        3. Loop:
           a. Capture image and detect balls
           b. Convert detected positions to robot frame
           c. Filter balls within workspace
           d. Pick and place first ball
           e. Repeat
    """
    print("="*60)
    print("Lab 8: Robotic Sorting System")
    print("="*60)
    
    # ==========================================================================
    # INITIALIZATION
    # ==========================================================================
    
    robot = Robot()
    camera = Realsense()
    intrinsics = camera.get_intrinsics()
    
    
    # ==========================================================================
    # Load camera-robot calibration matrix
    # ==========================================================================
    T_cam_to_robot = np.load(Path("assets/calibration/camera_robot_transform.npy"))
    
    
    # ==========================================================================
    # Setup robot in position control mode
    # ==========================================================================
    robot.write_mode("position")
    robot.write_motor_state(True)
    robot.write_time(TRAJECTORY_TIME)
    
    
    # ==========================================================================
    # Move to home position
    # ==========================================================================
    home_joints = get_ik(HOME_POSITION)[0]
    robot.write_joints(home_joints)
    time.sleep(TRAJECTORY_TIME)
    
    
    # ==========================================================================
    # Open gripper initially
    # ==========================================================================
    robot.write_gripper(1)
    
    
    print(f"\nReady! Using TRAJECTORY control")
    print("Press Ctrl+C to stop\n")
    
    # ==========================================================================
    # MAIN CONTROL LOOP
    # ==========================================================================
    
    try:
        iteration = 0
        
        while True:
            print(f"\n{'='*60}")
            print(f"Iteration {iteration}")
            
            # ==================================================================
            # STEP 1: CAPTURE IMAGE AND DETECT BALLS
            # ==================================================================
            
            color_frame, _ = camera.get_frames()
            
            
            spheres = detect_balls(color_frame) if color_frame is not None else None
            
            # Check if any balls detected
            if spheres is None:
                print("No balls detected")
                time.sleep(1)
                iteration += 1
                continue
            
            print(f"Detected {len(spheres)} ball(s)")
            
            # ==================================================================
            # STEP 2: CONVERT DETECTIONS TO ROBOT FRAME
            # ==================================================================
            
            robot_spheres = []  # List to store (color, robot_position) tuples
            
            for color, (cx, cy), radius in spheres:
                
                # ==============================================================
                corners = np.array([
                    [cx - radius, cy],
                    [cx + radius, cy],
                    [cx, cy + radius],
                    [cx, cy - radius],
                ], dtype=float)
                
                
                # ==============================================================
                cam_rot, cam_trans = get_ball_pose(corners, intrinsics, BALL_RADIUS)
                
                # ==============================================================
                pos_hom = np.append(cam_trans.reshape(3), 1.0)
                robot_pos = (T_cam_to_robot @ pos_hom)[:3]
                
                
                # ==============================================================
                # Check if position is within workspace bounds
                # ==============================================================
                # Check if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX
                # Skip balls outside workspace for safety
                if not (X_MIN <= robot_pos[0] <= X_MAX and 
                        Y_MIN <= robot_pos[1] <= Y_MAX):
                    print(f"  Skipping {color} ball outside workspace: {robot_pos}")
                    continue
                
                
                robot_spheres.append((color, robot_pos))
                print(f"  {color}: {robot_pos}")
            
            # Check if any valid balls found
            if not robot_spheres:
                print("No balls in workspace")
                time.sleep(1)
                iteration += 1
                continue
            
            # ==================================================================
            # STEP 3: PICK AND PLACE FIRST BALL
            # ==================================================================
            
            # Get first ball from list
            color, pos = robot_spheres[0]
            
            print(f"\nSorting {color} ball at {pos}")
            
            pick_ball(robot, pos)
            place_ball(robot, color)
            go_home(robot)
            
            
            iteration += 1
            time.sleep(1)  # Brief pause before next cycle
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        camera.stop()
        cv2.destroyAllWindows()
        print("Done!")


# ============================================================================
# PROGRAM ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()
