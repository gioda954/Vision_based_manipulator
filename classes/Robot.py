# (c) 2025 S. Farzan, Electrical Engineering Department, Cal Poly
# Skeleton Robot class for OpenManipulator-X Robot for EE 471

import numpy as np
from .OM_X_arm import OM_X_arm
from .DX_XM430_W350 import DX_XM430_W350
import cv2 as cv
from pathlib import Path


"""
Robot class for controlling the OpenMawhich python
python -V

nipulator-X Robot.
Inherits from OM_X_arm and provides methods specific to the robot's operation.
"""
class Robot(OM_X_arm):
    """
    Initialize the Robot class.
    Creates constants and connects via serial. Sets default mode and state.
    """
    def __init__(self):
        super().__init__()

        self.GRIP_OPEN_DEG  = -45.0
        self.GRIP_CLOSE_DEG = +45.0
        self.GRIP_THRESH_DEG = 180.0

        # Robot Dimensions (in mm)
        self.mDim = [77, 130, 124, 126]
        self.mOtherDim = [128, 24]
        
        # Set default mode and state
        # Change robot to position mode with torque enabled by default
        # Feel free to change this as desired
        self.write_mode('position')
        self.write_motor_state(True)

        # Set the robot to move between positions with a 5 second trajectory profile
        # change here or call writeTime in scripts to change
        self.write_time(5)

    def _set_time_profile_bit_all(self, enable: bool):
        """Turn the Drive Mode 'time-based profile' bit (bit 2) on/off for all joints."""
        DX = DX_XM430_W350
        # Read current drive modes
        dm = self.bulk_read_write(DX.DRIVE_MODE_LEN, DX.DRIVE_MODE, None)  # list[int]
        if not isinstance(dm, list) or len(dm) != len(self.motorIDs):
            raise RuntimeError("Failed to read DRIVE_MODE for all joints.")
        new_dm = []
        for v in dm:
            if enable:
                new_dm.append(v | 0b100)   # set bit 2
            else:
                new_dm.append(v & ~0b100)  # clear bit 2
        # Write back (bulk)
        self.bulk_read_write(DX.DRIVE_MODE_LEN, DX.DRIVE_MODE, new_dm)

    """
    Sends the joints to the desired angles.
    Parameters:
    goals (list of 1x4 float): Angles (degrees) for each of the joints to go to.
    """
    def write_joints(self, q_deg):
        """Send joint target angles in degrees (list/array length N)."""
        DX = DX_XM430_W350
        q_deg = list(q_deg)
        if len(q_deg) != len(self.motorIDs):
            raise ValueError(f"Expected {len(self.motorIDs)} joint angles, got {len(q_deg)}")

        ticks = [int(round(angle * DX.TICKS_PER_DEG + DX.TICK_POS_OFFSET)) for angle in q_deg]

        # If you're in normal position mode (not extended), keep values in [0, 4095]
        ticks = [max(0, min(int(DX.TICKS_PER_ROT - 1), t)) for t in ticks]

        self.bulk_read_write(DX.POS_LEN, DX.GOAL_POSITION, ticks)

    """
    Creates a time-based profile (trapezoidal) based on the desired times.
    This will cause write_position to take the desired number of seconds to reach the setpoint.
    Parameters:
    time (float): Total profile time in seconds. If 0, the profile will be disabled (be extra careful).
    acc_time (float, optional): Total acceleration time for ramp up and ramp down (individually, not combined). Defaults to time/3.
    """
    def write_time(self, total_time_s, acc_time_s=None):
        """Configure trapezoidal TIME profile for all joints."""
        if acc_time_s is None:
            acc_time_s = float(total_time_s) / 3.0

        # Enable time-based profile (bit 2) for all joints
        self._set_time_profile_bit_all(True)

        acc_ms = int(round(acc_time_s * DX_XM430_W350.MS_PER_S))
        tot_ms = int(round(float(total_time_s) * DX_XM430_W350.MS_PER_S))

        # Bulk write to all joints
        self.bulk_read_write(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, [acc_ms] * len(self.motorIDs))
        self.bulk_read_write(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, [tot_ms] * len(self.motorIDs))

    """
    Sets the gripper to be open or closed.
    Parameters:
    open (bool): True to set the gripper to open, False to close.
    """
    def write_gripper(self, is_open: bool):
        """Open/close gripper using fixed angles in position mode."""
        target = self.GRIP_OPEN_DEG if is_open else self.GRIP_CLOSE_DEG
        self.gripper.write_position(target)

    def read_gripper(self) -> float:
        """Return gripper joint position in degrees."""
        return self.gripper.read_position()

    def read_gripper_open(self) -> bool:
        return (self.read_gripper() > self.GRIP_THRESH_DEG)

    """
    Sets position holding for the joints on or off.
    Parameters:
    enable (bool): True to enable torque to hold the last set position for all joints, False to disable.
    """
    def write_motor_state(self, enable):
        state = 1 if enable else 0
        states = [state] * self.motorsNum  # Repeat the state for each motor
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, states)

    """
    Supplies the joints with the desired currents.
    Parameters:
    currents (list of 1x4 float): Currents (mA) for each of the joints to be supplied.
    """
    def write_currents(self, currents):
        current_in_ticks = [round(current * DX_XM430_W350.TICKS_PER_mA) for current in currents]
        self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, current_in_ticks)

    """
    Change the operating mode for all joints.
    Parameters:
    mode (str): New operating mode for all joints. Options include:
        "current": Current Control Mode (writeCurrent)
        "velocity": Velocity Control Mode (writeVelocity)
        "position": Position Control Mode (writePosition)
        "ext position": Extended Position Control Mode
        "curr position": Current-based Position Control Mode
        "pwm voltage": PWM Control Mode
    """
    def write_mode(self, mode):
        if mode in ['current', 'c']:
            write_mode = DX_XM430_W350.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            write_mode = DX_XM430_W350.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            write_mode = DX_XM430_W350.POS_CNTR_MD
        elif mode in ['ext position', 'ep']:
            write_mode = DX_XM430_W350.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']:
            write_mode = DX_XM430_W350.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']:
            write_mode = DX_XM430_W350.PWM_CNTR_MD
        else:
            raise ValueError(f"writeMode input cannot be '{mode}'. See implementation in DX_XM430_W350 class.")

        self.write_motor_state(False)
        write_modes = [write_mode] * self.motorsNum  # Create a list with the mode value for each motor
        self.bulk_read_write(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, write_modes)
        self.write_motor_state(True)

    """
    Gets the current joint positions, velocities, and currents.
    Returns:
    numpy.ndarray: A 3x4 array containing the joints' positions (deg), velocities (deg/s), and currents (mA).
    """
    def get_joints_readings(self):
        """
        Returns a 3xN array: [deg; deg/s; mA] for the N arm joints (excludes gripper).
        """
        N = len(self.motorIDs)

        # Bulk read raw registers
        pos_u32 = self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION, None)  # list of ints
        vel_u32 = self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY, None)
        cur_u16 = self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT,  None)

        # Vectorize
        pos_u32 = np.array(pos_u32, dtype=np.uint32)
        vel_u32 = np.array(vel_u32, dtype=np.uint32)  # signed 32-bit
        cur_u16 = np.array(cur_u16, dtype=np.uint16)  # signed 16-bit

        # Convert signed types
        vel_i32 = (vel_u32.astype(np.int64) + (1 << 31)) % (1 << 32) - (1 << 31)
        vel_i32 = vel_i32.astype(np.int32)
        cur_i16 = (cur_u16.astype(np.int32) + (1 << 15)) % (1 << 16) - (1 << 15)
        cur_i16 = cur_i16.astype(np.int16)

        # Units
        q_deg  = (pos_u32.astype(np.int64) - int(DX_XM430_W350.TICK_POS_OFFSET)) / DX_XM430_W350.TICKS_PER_DEG
        qd_dps = vel_i32 / DX_XM430_W350.TICKS_PER_ANGVEL
        I_mA   = cur_i16 / DX_XM430_W350.TICKS_PER_mA

        readings = np.vstack([q_deg.astype(float), qd_dps.astype(float), I_mA.astype(float)])
        return readings

    """
    Sends the joints to the desired velocities.
    Parameters:
    vels (list of 1x4 float): Angular velocities (deg/s) for each of the joints to go at.
    """
    def write_velocities(self, vels):
        """Send joint target velocities in deg/s (list/array length N)."""
        vels = list(vels)
        if len(vels) != len(self.motorIDs):
            raise ValueError(f"Expected {len(self.motorIDs)} velocities, got {len(vels)}")

        ticks_per_s = [int(round(v * DX_XM430_W350.TICKS_PER_ANGVEL)) for v in vels]  # signed
        self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, ticks_per_s)
    
    def __init__(self):
        #stack
        """
        Initialize robot constants and the DH table.
        """
        self.link_lengths_mm = np.array([77.0, 130.0, 124.0, 126.0], dtype=float)

        # TODO: fill your DH table (4x4) based on your derivation.
        self.DHTable = np.array([[0,77,0,(-np.pi/2)],
                                 [-((np.pi/2)-np.arcsin(24/130)),0,130,0 ],
                                 [((np.pi/2)-np.arcsin(24/130)),0,124,0],
                                 [0,0,126,0]], dtype=float)  # replace with your table

    def get_dh_row_mat(self, row):

        theta = row[0]
        d = row[1] 
        a = row[2]
        alpha = row[3]
        

        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        A = np.array([[ct, -st*ca,  st*sa, a*ct],
                         [st,  ct*ca, -ct*sa, a*st],
                         [0,      sa,     ca,    d],
                         [0,       0,      0,    1]], dtype=float)
        return A

    def get_int_mat(self, joint_angles):

           # --- validate/convert input (degrees -> radians) ---
        q_deg = np.asarray(joint_angles, dtype=float).reshape(-1)
        if q_deg.size != 4:
            raise ValueError("joint_angles must have 4 elements (degrees).")
        q_rad = np.deg2rad(q_deg)

    # --- Internal DH table [θ0(rad), d, a, α(rad)] ---
    # Using your values; lengths here are in meters (convert mm -> m).
        

    # --- add the joint variables to θ column ---
        dh = self.DHTable.copy()

    # --- compute A_i for each row and stack ---
        A_stack = np.empty((4, 4, 4), dtype=float)
        for i in range(4):
            A_stack[:, :, i] = self.get_dh_row_mat(dh[i])

        return A_stack
        


    def get_fk(self, joint_angles):

        A_stack = self.get_int_mat(joint_angles)

    # --- Initialize overall transform as identity ---
        T = np.eye(4, dtype=float)

    # --- Multiply all transforms sequentially ---
        for i in range(4):
            T = T @ A_stack[:, :, i]

    # --- Return final base-to-end-effector transform ---
        return T
    def get_current_fk(self):
   
    # --- Retrieve current joint angles (degrees) ---
        joint_angles = self.get_joints_readings()   # this should return [q1, q2, q3, q4]

    # --- Compute forward kinematics using get_fk() ---
        T_current = self.get_fk(joint_angles)

        return T_current
    
    def get_ee_pos(self, joint_angles):
    
  
    # --- Get forward kinematics transform (in meters) ---
        T = self.get_fk(joint_angles)

    # --- Extract position (convert from m → mm)---
        x_mm = T[0, 3]  
        y_mm = T[1, 3] 
        z_mm = T[2, 3] 

    # --- Orientation from joint angles ---
        q_deg = np.asarray(joint_angles, dtype=float)
        yaw_deg = q_deg[0]                      # base rotation
        pitch_deg = -(q_deg[1] + q_deg[2] + q_deg[3])  # negative sum of last 3

    # --- Combine into 1×5 array ---
        ee_pos = np.array([x_mm, y_mm, z_mm, pitch_deg, yaw_deg])

        return ee_pos
    

    def get_jacobian(self, q):
        """
        Compute the manipulator Jacobian matrix.

        Parameters
        ----------
        q : numpy.ndarray
            Joint angles in DEGREES [q1, q2, q3, q4]

        Returns
        -------
        J : numpy.ndarray (6, 4)
        """
        # Se get_int_mat NON converte i gradi internamente, scommenta la riga sotto:
        # q = np.deg2rad(q)

        # 1) T accumulati
        a_matrices = self.get_int_mat(q)
        t_matrices = np.zeros((4, 4, 4))
        t_matrices[:, :, 0] = a_matrices[:, :, 0]
        for i in range(1, 4):
            t_matrices[:, :, i] = t_matrices[:, :, i-1] @ a_matrices[:, :, i]

        # 2) Jacobiano
        J = np.zeros((6, 4))

        # o_ee = origine dell’EEF
        o_ee = t_matrices[:3, 3, 3]

        # o0..o3
        origins = np.hstack((
            np.zeros((3, 1)),
            t_matrices[:3, 3, :3]
        ))

        # z0..z3
        z_axes = np.hstack((
            np.array([[0, 0, 1]]).T,
            t_matrices[:3, 2, :3]
        ))

        for i in range(4):
            J_v_i = np.cross(z_axes[:, i], o_ee - origins[:, i])
            J_omega_i = z_axes[:, i]
            J[:, i] = np.hstack((J_v_i, J_omega_i))

        return J
    
    def get_fwd_vel_kin(self, q_deg, qdot_rad):
       
        q_deg = np.asarray(q_deg, dtype=float).reshape(-1)
        qdot = np.asarray(qdot_rad, dtype=float).reshape(-1)

        if q_deg.size != 4 or qdot.size != 4:
            raise ValueError("q_deg and qdot_rad must have 4 elements each")

        # J expects q in degrees 
        J = self.get_jacobian(q_deg)          # shape (6,4)
        p_dot = J @ qdot                      # shape (6,)

        return p_dot
    
    
    def point_registration(A, B):
 
        assert A.shape == B.shape
        d, n = A.shape

        cA = A.mean(axis=1, keepdims=True)   # (d,1)
        cB = B.mean(axis=1, keepdims=True)   # (d,1)
        A0 = A - cA
        B0 = B - cB

    
        H = A0 @ B0.T

        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:             # reflection correction
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

    
        t = (cB - R @ cA).reshape(d)

        return R, t
    

    def detect_balls(self, img_bgr):
        """
        Detect and classify colored balls (red, orange, yellow, blue) in a workspace image.

        Returns:
            detections: list of dicts with keys:
                - color: str ('red'|'orange'|'yellow'|'blue')
                - center: (x, y) ints (centroid)
                - radius: float (px)
            vis: BGR image with circles, centroids, and color labels drawn
        """

        # ---- config 
        HSV_RANGES = {
            "red1":   ((0,  90, 70),  (10, 255, 255)),
            "red2":   ((170,90, 70),  (180,255,255)),
            "orange": ((10, 120, 130), (21, 255, 255)),   
            "yellow": ((22, 140, 140), (32, 255, 255)),   
            "blue":   ((95,  80,  70), (130,255, 255)),
        }
        KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
        MIN_AREA = 150
        MIN_CIRCULARITY = 0.72
        GAUSS_BLUR = (5,5)

        def _clean_mask(m):
            m = cv.morphologyEx(m, cv.MORPH_OPEN, KERNEL)
            m = cv.morphologyEx(m, cv.MORPH_CLOSE, KERNEL)
            return m

        def _circularity(cnt):
            A = cv.contourArea(cnt)
            P = cv.arcLength(cnt, True)
            if P <= 0:
                return 0.0
            return 4.0 * np.pi * A / (P * P)

        # ---- preprocessing ----
        img = cv.GaussianBlur(img_bgr, GAUSS_BLUR, 0)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # ---- color masks ----
        masks = {}
        for name, (lo, hi) in HSV_RANGES.items():
            mask = cv.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            masks[name] = _clean_mask(mask)

        # merge red wrap-around
        if "red1" in masks and "red2" in masks:
            masks["red"] = cv.bitwise_or(masks["red1"], masks["red2"])
            masks.pop("red1", None); masks.pop("red2", None)

        # ---- detect via contours ----
        detections = []
        for label, mask in masks.items():
            if label not in {"red", "orange", "yellow", "blue"}:
                continue
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv.contourArea(c)
                if area < MIN_AREA:
                    continue
                if _circularity(c) < MIN_CIRCULARITY:
                    continue
                (x, y), r = cv.minEnclosingCircle(c)
                M = cv.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = int(x), int(y)
                detections.append({"color": label, "center": (cx, cy), "radius": float(r)})

        # ---- visualization ----
        vis = img_bgr.copy()
        for d in detections:
            x, y = d["center"]
            r = max(3, int(round(d["radius"])))
            cv.circle(vis, (x, y), r, (0, 255, 0), 2)
            cv.circle(vis, (x, y), 3, (255, 255, 255), -1)
            cv.putText(vis, d["color"], (x - 10, y - r - 8),
                    cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv.LINE_AA)

        return detections, vis




       
    











        

