"""
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
Lab 6 Starter Code: AprilTag Detection and Pose Estimation Test Script
Tests the integration of RealSense camera with AprilTag detection and pose estimation.
"""
import numpy as np
import cv2
from time import time
from classes.Realsense import Realsense
from classes.AprilTags import AprilTags

def main():
    try:
        # Initialize RealSense camera
        rs = Realsense()
        # Get camera intrinsic parameters
        intrinsics = rs.get_intrinsics()
        
        # Initialize AprilTag detector
        at = AprilTags()

        # Tag size in millimeters (measure your actual tag size)
        TAG_SIZE = 40.0  # 40mm tag (update this to match your actual tag size)
        
        # Counter for controlling print frequency
        counter = 0
        last_time = time()
        
        while True:
            # Get frames from RealSense
            color_frame, _ = rs.get_frames()
            if color_frame is None:
                continue
                
            # Detect AprilTags
            tags = at.detect_tags(color_frame)
            
            # Process each detected tag
            for tag in tags:
                # Draw tag detection on image
                color_frame = at.draw_tags(color_frame, tag)
                
                # Get pose estimation
                rot_matrix, trans_vector = at.get_tag_pose(
                    tag.corners, 
                    intrinsics, 
                    TAG_SIZE
                )
                
                # Print every 10 frames (approximately)
                if counter % 10 == 0:
                    if rot_matrix is not None and trans_vector is not None:
                        # Convert rotation matrix to Euler angles
                        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                        
                        # Calculate distance (norm of translation vector)
                        distance = np.linalg.norm(trans_vector)
                        
                        # Print results
                        print(f"\nTag ID: {tag.tag_id}")
                        print(f"Distance: {distance:.1f} mm")
                        print(f"Orientation (deg): roll={euler_angles[0]:.1f}, "
                              f"pitch={euler_angles[1]:.1f}, "
                              f"yaw={euler_angles[2]:.1f}")
                        print(f"Position (mm): x={trans_vector[0][0]:.1f}, "
                              f"y={trans_vector[1][0]:.1f}, "
                              f"z={trans_vector[2][0]:.1f}")
                        
                        # Calculate and print frame rate
                        current_time = time()
                        # fps = 1.0 / (current_time - last_time)
                        # print(f"FPS: {fps:.1f}")
                        last_time = current_time
            
            # Display the image
            cv2.imshow('AprilTag Detection', color_frame)
            
            # Increment counter
            counter += 1
            
            # Break loop with 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"Error in main loop: {str(e)}")
        
    finally:
        # Clean up
        rs.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()