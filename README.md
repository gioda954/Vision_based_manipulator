# Vision-Based Manipulator Codebase

Repository for an EE471 vision-guided OpenManipulator-X project. The `codebase/` folder holds course pre-labs and labs for arm kinematics, trajectory planning, and camera-based perception (RealSense + AprilTags). Hardware targets a Dynamixel-based OpenManipulator-X with an Intel RealSense D435.

## Layout
- `codebase/classes/`: Hardware abstractions  
  - `OM_X_arm.py`, `DX_XM430_W350.py`: Dynamixel SDK wiring and register helpers  
  - `Robot.py`: Robot convenience methods (time-based profiles, gripper control, FK/Jacobian helpers, ball detection).  
  - `PID.py`: Simple PID utility.
- `codebase/TrajPlanner.py`, `codebase/MyTrajPlanner.py`: Cubic trajectory planners that generate joint-space waypoints between setpoints.
- `codebase/lab*.py`, `codebase/prelab*.py`: Sequential course exercises covering sampling, joint profiling, kinematics, trajectory generation, and vision (labs 1–8). Many expect a connected OpenManipulator-X and follow the lab handouts.
- `codebase/lab6_AprilTags.py`, `codebase/lab6_Realsense.py`: AprilTag detection and RealSense wrappers used by `lab6_*` and later vision labs.
- `codebase/data/`: Example logs (pickle) used by plotting scripts.
- `codebase/image_prelab8.jpg`, `codebase/processed_prelab8.jpg`, `camera_robor_trandf.npy`: Sample images/transforms for vision prelabs.
- `codebase/venv471/`: Local Python 3.11 virtual environment (you can create your own instead).

## Getting Started
1) Create and activate a fresh environment (recommended):
```
python3 -m venv .venv
source .venv/bin/activate
```
2) Install runtime dependencies (common set for the scripts):
```
pip install numpy matplotlib opencv-python pyrealsense2 pyapriltags dynamixel-sdk pyserial
```
   - RealSense support also requires the Intel librealsense drivers on your OS.
3) Connect the OpenManipulator-X (Dynamixel XM430 series) via USB before running any hardware scripts.
