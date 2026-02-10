import numpy as np

from core.TrajPlanner import TrajPlanner
from steps.step03_inverse_kinematics.s01_ik_geometric_solver import get_ik
from steps.step02_forward_kinematics.s01_dh_fk_model import Robot

robot = Robot()

waypoints = [
    [25,  -100, 150, -60],
    [150,   80, 300,   0],
    [250, -115,  75, -45],
    [25,  -100, 150, -60]
]

joint = []

for i in range(4):

    sol1, _ = get_ik(waypoints[i])
    joint.append(sol1)

traj = []

for i in range(4):

    cubic = TrajPlanner.get_cubic_traj(joint[i], 5, 998)
    traj.append(cubic)

if __name__ == "__main__":
    print("Generated joint-space trajectories for all waypoints.")
