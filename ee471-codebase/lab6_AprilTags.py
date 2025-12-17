"""
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
AprilTag Detection and Pose Estimation Interface
Provides methods for detecting AprilTags in images and estimating their 3D poses.
Designed for vision-based robotic applications in EE 471.
"""

import cv2
import numpy as np
from pyapriltags import Detector

class AprilTags:
    """
    A class for AprilTag detection and pose estimation.
    
    This class provides methods to detect AprilTags in images, visualize the detections,
    and estimate the 3D pose of tags relative to the camera using PnP algorithm.
    
    Attributes:
        detector (Detector): PyAprilTags detector instance configured for tag36h11 family
    """
    
    def __init__(self):
        """
        Initialize the AprilTag detector.
        
        Creates a detector instance using the tag36h11 family, which is the standard
        tag family used in robotics applications.
        """
        try:
            self.detector = Detector()
        except Exception as e:
            raise RuntimeError(f"Failed to initialize AprilTag detector: {str(e)}")

    def detect_tags(self, image):
        """
        Detect AprilTags in an input image.
        
        Args:
            image (numpy.ndarray): Input BGR image from camera
            
        Returns:
            list: Detected tag objects, each containing:
                - tag_id: Identifier of the detected tag
                - center: Center point coordinates
                - corners: Corner point coordinates
                - homography: Tag plane to image plane homography
                
        Raises:
            ValueError: If input image is invalid or empty
        """
        if image is None or image.size == 0:
            raise ValueError("Invalid input image")
            
        try:
            # Convert to grayscale for tag detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Detect AprilTags in the image
            tags = self.detector.detect(gray)
            return tags
        except Exception as e:
            print(f"Error during tag detection: {str(e)}")
            return []

    def draw_tags(self, image, tag):
        """
        Visualize detected AprilTag on the input image.
        
        Args:
            image (numpy.ndarray): Input image to draw on
            tag: Single tag object from detector containing detection information
            
        Returns:
            numpy.ndarray: Image with visualized tag detection
            
        Note:
            Draws tag outline in green, center point in red, and tag ID number
        """
        try:
            # Extract tag information
            tag_id = tag.tag_id
            center = tuple(map(int, tag.center))
            corners = tag.corners.astype(int)

            # Draw tag outline (green)
            cv2.polylines(image, [corners], True, (0, 255, 0), 2)

            # Draw tag center (red)
            cv2.circle(image, center, 5, (0, 0, 255), -1)

            # Add tag ID label
            cv2.putText(image, str(tag_id), 
                       (center[0] - 10, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                       
            return image
            
        except Exception as e:
            print(f"Error drawing tag: {str(e)}")
            return image

    def get_tag_pose(self, corners, intrinsics, tag_size):
        """
        Estimate 3D pose of AprilTag using PnP algorithm.
        
        Args:
            corners (numpy.ndarray): 4x2 array of tag corner coordinates in image
            intrinsics: Camera intrinsic parameters (from RealSense)
            tag_size (float): Physical size of the tag in MILLIMETERS
            
        Returns:
            tuple: (rotation_matrix, translation_vector)
                - rotation_matrix: 3x3 rotation matrix from tag to camera frame
                - translation_vector: 3x1 translation vector from tag to camera 
                  frame (in MILLIMETERS)
                
        Note:
            Uses OpenCV's solvePnP with a planar tag model
        """
        try:
            # Convert tag size from mm to meters for OpenCV
            tag_size_m = tag_size / 1000.0
            
            # Define 3D model points (tag corners in tag frame, in meters)
            object_points = np.array([
                [-tag_size_m/2, -tag_size_m/2, 0],
                [ tag_size_m/2, -tag_size_m/2, 0],
                [ tag_size_m/2,  tag_size_m/2, 0],
                [-tag_size_m/2,  tag_size_m/2, 0]
            ])
            
            # Reshape image points for PnP
            image_points = corners.reshape((4, 2))
            
            # Construct camera matrix from intrinsics
            camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            
            # Solve PnP to get tag pose
            _, rvec, tvec = cv2.solvePnP(object_points, 
                                        image_points, 
                                        camera_matrix, 
                                        None)
            
            # Convert rotation vector to matrix
            rot_matrix, _ = cv2.Rodrigues(rvec)
            
            # Convert translation from meters to millimeters
            tvec_mm = tvec * 1000.0
            
            return rot_matrix, tvec_mm
            
        except Exception as e:
            print(f"Error estimating tag pose: {str(e)}")
            return None, None