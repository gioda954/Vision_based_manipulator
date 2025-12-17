"""
Lab 6 Part 2: Camera-Robot Calibration - STARTER CODE
Implements camera-robot calibration using AprilTags and the Kabsch algorithm.
EE 471: Vision-Based Robotic Manipulation
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
"""
import numpy as np
import cv2
from time import time, sleep

from classes.Realsense import Realsense
from classes.AprilTags import AprilTags
from classes.Robot import Robot as rb


def point_registration(points_A, points_B):
    """
    Implement Kabsch algorithm to find optimal rigid transformation between point sets.
    
    This is the same algorithm from Pre-Lab 6, computing the transformation that
    maps points from coordinate system A to coordinate system B.
    
    Args:
        points_A: 3xN array of points in frame A (camera frame)
        points_B: 3xN array of corresponding points in frame B (robot frame)
        
    Returns:
        4x4 homogeneous transformation matrix from A to B
    """
    assert points_A.shape == points_B.shape, "Point sets must have same dimensions"
    assert points_A.shape[0] == 3, "Points must be 3D"
    
    # Compute centroids
    centroid_A = np.mean(points_A, axis=1, keepdims=True)
    centroid_B = np.mean(points_B, axis=1, keepdims=True)
    
    # Center the point sets
    A_centered = points_A - centroid_A
    B_centered = points_B - centroid_B
    
    # Compute cross-covariance matrix
    H = A_centered @ B_centered.T
    
    # SVD decomposition
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation matrix
    R = Vt.T @ U.T
    
    # Handle reflection case (ensure proper rotation, not reflection)
    if np.linalg.det(R) < 0:
        print("  Warning: Reflection detected, correcting...")
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute translation
    t = centroid_B - R @ centroid_A
    
    # Construct 4x4 homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3:4] = t
    
    return T

def main():
    """
    Main calibration routine.
    """
    # =====================================================================
    # INITIALIZATION
    # =====================================================================
    print("="*60)
    print("Camera-Robot Calibration")
    print("="*60)
    
    # Initialize camera and detector
    print("\nInitializing camera and AprilTag detector...")
    camera = Realsense()
    detector = AprilTags()

    # TODO: Get camera intrinsics
    # YOUR CODE HERE
    intrinsics = camera.get_intrinsics()  # Replace with actual intrinsics
    print(f"Camera intrinsics obtained")
    
    # TODO: Set AprilTag physical size in millimeters
    # IMPORTANT: Measure your actual tags!
    # YOUR CODE HERE
    TAG_SIZE = 40.0  # Update this value if needed
    print(f"Tag size: {TAG_SIZE} mm")
    
    # =====================================================================
    # DEFINE ROBOT FRAME REFERENCE POINTS
    # =====================================================================
    print("\nDefining robot frame reference points...")
    
    # TODO: Define measured tag positions in robot frame
    # IMPORTANT: Replace these with YOUR measured coordinates!
    # Measure each tag center position relative to robot base origin
    # Store in order: Tag ID 0, 1, 2, ..., 11 (left-to-right, top-to-bottom)
    # YOUR CODE HERE
    robot_points_array = np.array([
        # [X, Y, Z] coordinates in mm for each tag
        # Example structure (REPLACE WITH YOUR MEASUREMENTS):
        # Row 1 (tags 0-3)
        [80,  -90, 0],   # Tag 0
        [80,  -30, 0],   # Tag 1
        [80,   30, 0],   # Tag 2
        [80,   89, 0],   # Tag 3
        # Row 2 (tags 4-7)
        [140, -90, 0],   # Tag 4
        [140, -30, 0],   # Tag 5
        [140,  29, 0],   # Tag 6
        [140,  89, 0],   # Tag 7
        # Row 3 (tags 8-11)
        [200, -90, 0],   # Tag 8
        [200, -30, 0],   # Tag 9
        [200,  30, 0],   # Tag 10
        [200,  90, 0],   # Tag 11
    ])
    
    # Convert to 3xN format (transpose)
    points_robot = robot_points_array.T  # Shape: (3, 12)
    num_tags = points_robot.shape[1]
    print(f"Expecting {num_tags} tags (IDs 0-{num_tags-1})")
    
    # Initialize camera points array (3 x num_tags, filled with zeros)
    points_camera = np.zeros((3, num_tags))
    # Create a list of empty lists, one for each tag
    measurements = [[] for _ in range(num_tags)]
    
    # =====================================================================
    # COLLECT MEASUREMENTS
    # =====================================================================
    print("\n" + "="*60)
    print("Starting measurement collection...")
    print("Position camera to see all tags clearly.")
    print("Press 'c' to capture a measurement, 'q' to finish")
    print("="*60)
    
    num_measurements = 0
    target_measurements = 5  # Collect 5 measurements for averaging
    
    while num_measurements < target_measurements:
        
        # -----------------------------------------------------------------
        # STEP 1: CAPTURE FRAME
        # -----------------------------------------------------------------
        
        # Get camera frame
        # Hint: camera.get_frames() returns (color_frame, depth_frame)
        color_frame, _ = camera.get_frames()
        
        if color_frame is None:
            continue
        
        # -----------------------------------------------------------------
        # STEP 2: DETECT APRILTAGS
        # -----------------------------------------------------------------
        
        # TODO: Detect AprilTags in the frame
        # Hint: Use detector.detect_tags(color_frame)
        # YOUR CODE HERE
        tags = detector.detect_tags(color_frame)  # Replace with detected tags
        
        # -----------------------------------------------------------------
        # STEP 3: VISUALIZE DETECTIONS
        # -----------------------------------------------------------------
        
        # Create a copy for display
        display_frame = color_frame.copy()
        
        # Draw all detected tags on display_frame
        for tag in tags:
            display_frame = detector.draw_tags(display_frame, tag)
        
        # Status overlay
        status_text = f"Tags detected: {len(tags)}/{num_tags} | Measurements: {num_measurements}/{target_measurements}"
        cv2.putText(display_frame, status_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if len(tags) == num_tags:
            cv2.putText(display_frame, "Ready! Press 'c' to capture", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Align camera to see all tags", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow('Calibration', display_frame)
        key = cv2.waitKey(1) & 0xFF
        
        # -----------------------------------------------------------------
        # STEP 4: CAPTURE MEASUREMENT ON USER INPUT
        # -----------------------------------------------------------------
        
        if key == ord('c'):  # User pressed 'c' to capture
            
            if len(tags) == num_tags:
                print(f"\nCapturing measurement {num_measurements + 1}/{target_measurements}...")
                
                # Sort tags by ID to maintain correspondence with robot_points
                tags_sorted = sorted(tags, key=lambda t: t.tag_id)
                
                # Process each tag to get its pose
                temp_measurements = []
                
                for tag in tags_sorted:
                    
                    # TODO: Get pose estimation for this tag
                    # Hint: Use detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
                    # Returns: (rotation_matrix, translation_vector)
                    # Note: We only need translation_vector (tag position) for calibration
                    # YOUR CODE HERE
                    rot_matrix, trans_vector  = detector.get_tag_pose(tag.corners, intrinsics,TAG_SIZE)  # Replace with actual rotation
                    
                    
                    # TODO: Store the translation vector (position in camera frame)
                    temp_measurements.append(trans_vector.flatten())

                    # Hint: Flatten trans_vector and append to temp_measurements
                    # Note: We ignore rot_matrix - only position is needed for Kabsch algorithm
                    # YOUR CODE HERE
                

                # If all tags processed successfully, store measurements
                # Loop through temp_measurements and append each to measurements[idx]
                for idx, measurement in enumerate(temp_measurements):
                    measurements[idx].append(measurement)
                num_measurements += 1
                
            else:
                print(f"  Error: Only {len(tags)}/{num_tags} tags visible. Need all tags!")
        
        # Quit on 'q'
        elif key == ord('q'):
            if num_measurements > 0:
                print(f"\nFinishing with {num_measurements} measurements...")
                break
            else:
                print("\nNo measurements collected. Exiting...")
                return
    
    # =====================================================================
    # PROCESS MEASUREMENTS
    # =====================================================================
    print("\n" + "="*60)
    print("Processing measurements...")
    
    # Average all measurements for each tag
    for idx, tag_measurements in enumerate(measurements):
        if len(tag_measurements) > 0:
            avg_position = np.mean(tag_measurements, axis=0)
            points_camera[:, idx] = avg_position
            print(f"  Tag {idx}: {len(tag_measurements)} measurements averaged")
        else:
            print(f"  Warning: No measurements for tag {idx}!")
    
    # =====================================================================
    # COMPUTE TRANSFORMATION
    # =====================================================================
    print("\nComputing camera-to-robot transformation...")
    
    # TODO: Call point_registration() to compute transformation

    # YOUR CODE HERE
    T_cam_to_robot = point_registration(points_camera , points_robot) # Replace with actual transformation
    
    print("\nTransformation matrix (camera to robot):")
    print(T_cam_to_robot)
    
    # TODO: Extract rotation and translation for display
    # YOUR CODE HERE
    R = T_cam_to_robot[:3,:3] # Replace with actual rotation
    t = T_cam_to_robot[:3, 3] # Replace with actual translation

    print(f"\nTranslation: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] mm")
    
    # Verify rotation matrix properties
    det_R = np.linalg.det(R)
    orthogonality_check = np.linalg.norm(R @ R.T - np.eye(3))
    print(f"Rotation matrix determinant: {det_R:.6f} (should be 1.0)")
    print(f"Orthogonality error: {orthogonality_check:.6e} (should be ~0)")
    
    # =====================================================================
    # CALCULATE CALIBRATION ERROR
    # =====================================================================
    print("\n" + "="*60)
    print("Calculating calibration accuracy...")
    
    # TODO: Transform camera points to robot frame using T_cam_to_robot
    # Hint: Convert points_camera to homogeneous coordinates first (add row of ones)
    # Then multiply: T_cam_to_robot @ points_camera_homogeneous
    # YOUR CODE HERE
    points_camera_hom = np.vstack((points_camera, np.ones((1,points_camera.shape[1]))))  # Replace with homogeneous coordinates
    points_camera_transformed = (T_cam_to_robot @ points_camera_hom)[:3] # Replace with transformed points
    
    # TODO: Calculate errors between transformed camera points and robot points
    # Hint: Compute differences, then use np.linalg.norm() on each column
    # YOUR CODE HERE
    errors = points_camera_transformed - points_robot # Replace with error vectors
    error_magnitudes = np.linalg.norm(errors, axis=0)
    
    # TODO: Calculate error statistics
    # Hint: Use np.mean(), np.std(), np.max(), np.min()
    # YOUR CODE HERE
    mean_error = float(np.mean(error_magnitudes))  # Replace with mean
    std_error = float(np.std(error_magnitudes)) # Replace with std
    max_error = float(np.max(error_magnitudes))  # Replace with max
    min_error = float(np.min(error_magnitudes))  # Replace with min
    
    print(f"\nCalibration Error Statistics:")
    print(f"  Mean error:    {mean_error:.3f} mm")
    print(f"  Std deviation: {std_error:.3f} mm")
    print(f"  Min error:     {min_error:.3f} mm")
    print(f"  Max error:     {max_error:.3f} mm")
    
    # Print per-tag errors
    print(f"\nPer-tag errors:")
    for i in range(num_tags):
        print(f"  Tag {i:2d}: {error_magnitudes[i]:6.3f} mm")
    
    # Quality assessment
    if mean_error < 5.0:
        print("\n✓ Calibration quality: EXCELLENT (< 5 mm)")
    elif mean_error < 10.0:
        print("\n✓ Calibration quality: GOOD (< 10 mm)")
    else:
        print("\n⚠ Calibration quality: ACCEPTABLE but consider recalibrating")
    
    # =====================================================================
    # SAVE RESULTS
    # =====================================================================
    print("\n" + "="*60)
    
    # TODO: Save transformation matrix to file
    # Hint: Use np.save('camera_robot_transform.npy', T_cam_to_robot)
    filename = 'camera_robot_transform.npy'
    np.save(filename,T_cam_to_robot)
    # YOUR CODE HERE
    

    print(f"Transformation matrix saved to '{filename}'")
    print("="*60)
    
    print("\nCleaning up...")
    # Stop camera and close windows
    camera.stop()
    cv2.destroyAllWindows()

    print("Done!")


if __name__ == "__main__":
    main()