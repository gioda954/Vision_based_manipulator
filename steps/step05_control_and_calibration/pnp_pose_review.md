# Code Review – PnP Pose Estimation

## How `get_tag_pose()` Formulates the PnP Problem

- Converts the physical tag size from millimetres to metres so that the 3D model fed to OpenCV is metric.
- Builds the object-point array by placing the four tag corners on the z=0 plane in the tag frame, centred at the origin and scaled by the tag size (`[-s/2, -s/2, 0]` … `[s/2, s/2, 0]`).
- Reshapes the detected image-space corners into a 4×2 array that forms the corresponding image points.
- Packs the RealSense intrinsics (`fx`, `fy`, `ppx`, `ppy`) into the 3×3 camera intrinsic matrix and assumes zero distortion coefficients.
- Calls `cv2.solvePnP(...)` with the object and image points to obtain the pose, converts the returned Rodrigues rotation vector to a 3×3 rotation matrix, and rescales the translation vector back to millimetres for downstream use.

## `s07_apriltag_pose_demo.py` Workflow Overview

### Camera Initialisation
- Instantiates the `Realsense` wrapper, which starts the Intel RealSense pipeline and enables both colour (`bgr8`) and depth streams at 640×480/30 Hz.
- Immediately fetches the colour-camera intrinsic parameters for later PnP pose estimation.

### Per-frame Tag Detection
- Continuously grabs the latest colour frame from the RealSense stream.
- Uses `AprilTags.detect_tags()` to convert the frame to grayscale and run the PyAprilTags detector, returning any tag detections.
- For each tag, `draw_tags()` overlays the outline, ID, and centre point on the frame to aid visualisation.

### Pose Extraction and Display
- Calls `AprilTags.get_tag_pose()` with the tag’s corner coordinates, camera intrinsics, and physical tag size to obtain the rotation matrix and translation vector in the camera frame.
- Every ten frames it computes an orientation estimate (via `cv2.RQDecomp3x3`), derives the tag-to-camera distance from the translation vector, and prints the tag ID, distance, Euler-angle tuple, and Cartesian position in millimetres.
- Shows the annotated video stream in an OpenCV window and exits the loop when the user presses `q`, after which it stops the camera pipeline and closes the window.
