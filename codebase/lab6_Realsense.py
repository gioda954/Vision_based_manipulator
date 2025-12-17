"""
(c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
Intel RealSense Camera Interface for EE 471
Provides a simplified interface to the RealSense D435 RGB-D camera for robotic vision tasks.
"""

import pyrealsense2 as rs
import numpy as np
import cv2

class Realsense:
    """
    A wrapper class for Intel RealSense D435 camera operations.
    
    This class provides an interface to configure and operate the RealSense D435 camera,
    including stream configuration, frame capture, and intrinsics acquisition.
    
    Attributes:
        pipeline (rs.pipeline): RealSense pipeline for streaming
        config (rs.config): Configuration object for the RealSense camera
        profile (rs.pipeline_profile): Streaming profile containing stream information
    """
    
    def __init__(self, width=640, height=480, fps=30):
        """
        Initialize and configure the RealSense camera.
        
        Args:
            width (int, optional): Stream width in pixels. Defaults to 640.
            height (int, optional): Stream height in pixels. Defaults to 480.
            fps (int, optional): Frames per second. Defaults to 30.
            
        Raises:
            RuntimeError: If camera initialization fails
        """
        try:
            # Configure RealSense pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable depth and color streams with specified parameters
            self.config.enable_stream(
                rs.stream.depth, 
                width, height, 
                rs.format.z16, 
                fps
            )
            self.config.enable_stream(
                rs.stream.color, 
                width, height, 
                rs.format.bgr8, 
                fps
            )
            
            # Start streaming
            self.profile = self.pipeline.start(self.config)
            
        except RuntimeError as e:
            raise RuntimeError(f"Failed to initialize RealSense camera: {str(e)}")
    
    def get_intrinsics(self):
        """
        Get the intrinsic parameters of the color camera.
        
        Returns:
            rs.intrinsics: Camera intrinsics including focal length, principal point,
                          and distortion coefficients
                          
        Raises:
            RuntimeError: If unable to get intrinsics
        """
        try:
            return self.profile.get_stream(
                rs.stream.color
            ).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            raise RuntimeError(f"Failed to get color camera intrinsics: {str(e)}")
    
    def get_depth_intrinsics(self):
        """
        Get the intrinsic parameters of the depth camera.
        
        Returns:
            rs.intrinsics: Depth camera intrinsics including focal length, principal point,
                          and distortion coefficients
                          
        Raises:
            RuntimeError: If unable to get intrinsics
        """
        try:
            return self.profile.get_stream(
                rs.stream.depth
            ).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            raise RuntimeError(f"Failed to get depth camera intrinsics: {str(e)}")
    
    def get_frames(self):
        """
        Capture and return the current color and depth frames.
        
        Returns:
            tuple: (color_image, depth_image)
                - color_image: numpy array of RGB image (height, width, 3)
                - depth_image: numpy array of depth values in millimeters (height, width)
            None, None: If frame capture fails
            
        Note:
            Depth values are in millimeters
        """
        try:
            # Wait for a coherent frame
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            # Check if frames are valid
            if not depth_frame or not color_frame:
                return None, None
                
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            print(f"Error capturing frames: {str(e)}")
            return None, None
    
    def stop(self):
        """
        Stop the RealSense pipeline and release resources.
        
        Should be called when finishing camera operations.
        """
        try:
            self.pipeline.stop()
        except Exception as e:
            print(f"Error stopping pipeline: {str(e)}")
