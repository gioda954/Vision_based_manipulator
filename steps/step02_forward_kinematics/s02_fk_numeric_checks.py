# main.py
import numpy as np
from steps.step02_forward_kinematics.s01_dh_fk_model import Robot

np.set_printoptions(precision=3, suppress=True)

robot = Robot()

angles_sets = [
    [0, 0, 0, 0],
    [15, -45, -60, 90],
    [-90, 15, 30, -45]
]

for idx, ang in enumerate(angles_sets, start=1):
    T = robot.get_fk(ang)
    print(f"joint {idx}:\n{T}\n")
