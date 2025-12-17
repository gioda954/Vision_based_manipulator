"""
Lab 6 Part 3: Validation of Camera-Robot Calibration - STARTER CODE
Tracks an AprilTag and transforms its position from camera to robot frame.
EE 471: Vision-Based Robotic Manipulation
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
"""
import numpy as np
import cv2
from time import time

from classes.Realsense import Realsense
from classes.AprilTags import AprilTags


def main():
    """
    Main validation routine - tracks a tag and displays its position in both frames.
    """
    # =====================================================================
    # INITIALIZATION
    # =====================================================================
    print("="*60)
    print("Camera-Robot Calibration Validation")
    print("="*60)
    
    # TODO: Initialize camera and detector
    # Hint: Create Realsense() and AprilTags() instances
    print("\nInitializing camera and detector...")
    # YOUR CODE HERE
    camera = Realsense()  # Replace with actual camera
    detector = AprilTags()  # Replace with actual detector
    
    # TODO: Get camera intrinsics
    # Hint: Use camera.get_intrinsics()
    # YOUR CODE HERE
    intrinsics = camera.get_intrinsics() # Replace with actual intrinsics
    
    # TODO: Load the calibration transformation matrix
    # Hint: Use np.load('camera_robot_transform.npy')
    # YOUR CODE HERE
    T_cam_to_robot = np.load('camera_robot_transform.npy') # Replace with loaded transformation
    print("\nLoaded camera-to-robot transformation matrix:")
    print(T_cam_to_robot)
    
    # TODO: Set the validation tag size in millimeters
    # IMPORTANT: Measure your validation tag!
    # YOUR CODE HERE
    TAG_SIZE = 40.0  # Update this value
    print(f"\nValidation tag size: {TAG_SIZE} mm")
    
    # Display settings
    PRINT_INTERVAL = 10  # Print every N frames to reduce clutter
    
    print("\n" + "="*60)
    print("Starting validation...")
    print("Move the tag around the workspace")
    print("Press 'q' to quit")
    print("="*60 + "\n")
    
    counter = 0
    
    # =====================================================================
    # MAIN TRACKING LOOP
    # =====================================================================
    while True:
        
        # -----------------------------------------------------------------
        # STEP 1: CAPTURE FRAME
        # -----------------------------------------------------------------
        
        # TODO: Get camera frame
        # Hint: Use camera.get_frames() which returns (color_frame, depth_frame)
        # YOUR CODE HERE
        color_frame, _ = camera.get_frames()  # Replace with actual frame
        
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
        # STEP 3: PROCESS DETECTED TAGS
        # -----------------------------------------------------------------
        
        # Check if any tags were detected
        if len(tags) > 0:
            
            # Use the first detected tag
            tag = tags[0]
            
            # TODO: Get pose estimation in camera frame
            # Hint: Use detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)
            # Returns: (rotation_matrix, translation_vector)
            # YOUR CODE HERE
            rot_matrix, trans_vector =detector.get_tag_pose(tag.corners, intrinsics, TAG_SIZE)  # Replace with actual rotation
             # Replace with actual translation
            
            # Check if pose estimation was successful
            if rot_matrix is not None and trans_vector is not None:
                
                # TODO: Extract position in camera frame (already in mm)
                # Hint: Flatten trans_vector to get a 1D array of shape (3,)
                # YOUR CODE HERE
                pos_camera = trans_vector.reshape(3)  # Replace with position array

                T_tag_to_cam = np.empty((4, 4), dtype=float)
                T_tag_to_cam[:3, :3] = rot_matrix # Replace with rotation matrix
                T_tag_to_cam[:3, 3] = pos_camera  # Replace with translation vector
                T_tag_to_cam[3] = (0.0, 0.0, 0.0, 1.0)
                
                # TODO: # Transform full pose to robot frame
                # Hint: Multiply T_cam_to_robot @ T_tag_to_cam
                # YOUR CODE HERE
                T_tag_to_robot = T_cam_to_robot @ T_tag_to_cam  # Replace with homogeneous coordinates (4x4)
                
                # Extract position and orientation in robot frame
                pos_robot = T_tag_to_robot[:3, 3]
                rot_robot = T_tag_to_robot[:3, :3]

                # Could convert rotation to Euler angles for display
                euler_robot = cv2.RQDecomp3x3(rot_robot)[0]
                
                # TODO: Calculate distance from camera
                # Hint: Use np.linalg.norm()
                # YOUR CODE HERE
                distance = np.linalg.norm(pos_camera)  # Replace with distance
                
                # TODO: Draw detection on image
                # Hint: Use detector.draw_tags(color_frame, tag)
                color_frame=detector.draw_tags(color_frame, tag)
                # YOUR CODE HERE
                

                # Add coordinate overlay on image
                y_offset = 60
                cv2.putText(color_frame, f"Tag ID: {tag.tag_id}", 
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.6, (0, 255, 0), 2)
                y_offset += 30
                cv2.putText(color_frame, f"Cam: ({pos_camera[0]:.1f}, {pos_camera[1]:.1f}, {pos_camera[2]:.1f}) mm", 
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 0), 2)
                y_offset += 25
                cv2.putText(color_frame, f"Robot: ({pos_robot[0]:.1f}, {pos_robot[1]:.1f}, {pos_robot[2]:.1f}) mm", 
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 255), 2)
                
                # Print coordinates periodically to terminal
                if counter % PRINT_INTERVAL == 0:
                    print("\n" + "-"*50)
                    print(f"Tag ID: {tag.tag_id}")
                    print(f"Distance from camera: {distance:.1f} mm")
                    print("\nTag Orientation:")
                    # TODO: Print Tag Orientation in Robot Frame
                    roll, pitch,yaw= (float(angle) for angle in euler_robot)
                    print(rot_robot)
                    print(f"euler angles in degrees : roll ={roll:.1f},pitch ={pitch:.1f},yaw ={yaw:.1f}")
                    # YOUR CODE HERE
                    
                    print("\nCamera Frame (mm):")
                    print(f"x={pos_camera[0]:.1f},y={pos_camera[1]:.1f},z={pos_camera[2]:.1f}")
                    # TODO: Print 3D positions in Camera Frame
                    # YOUR CODE HERE

                    print("\nRobot Frame (mm):")
                    # TODO: Print 3D positions in Robot Frame
                    print(f"x={pos_robot[0]:.1f},y={pos_robot[1]:.1f},z={pos_robot[2]:.1f}")
                    # YOUR CODE HERE

                    print("-"*50)
            
        else:
            # No tags detected
            if counter % PRINT_INTERVAL == 0:
                print("\nNo tag detected - move tag into view")
            
            cv2.putText(color_frame, "No tag detected", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 0, 255), 2)
            
        
        # -----------------------------------------------------------------
        # STEP 4: DISPLAY AND USER INTERACTION
        # -----------------------------------------------------------------
        
        # Show instruction
        cv2.putText(color_frame, "Press 'q' to quit", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (0, 255, 0), 2)
        
        # Display frame
        cv2.imshow('Calibration Validation', color_frame)
        
        # Increment counter
        counter += 1
        
        # Check for exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nExiting validation...")
            break
    
    print("\nValidation complete!")
    
    print("\nCleaning up...")
    camera.stop()
    cv2.destroyAllWindows()
    print("Done!")


if __name__ == "__main__":
    main()
