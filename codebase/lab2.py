import numpy as np


class Robot:
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
