import numpy as np
from TrajPlanner import TrajPlanner
from prelab3 import get_ik
from lab2 import Robot

robot= Robot()

waypoints = [
    [25,  -100, 150, -60],
    [150,   80, 300,   0],
    [250, -115,  75, -45],
    [25,  -100, 150, -60]
]

joint = []

for i in range(4):

    joint_angles = get_ik(waypoints[i])
    joint.append[joint_angles]

traj = []

for i in range(4):

    cubic = TrajPlanner.get_cubic_traj(joint[i],5,998)
    traj.append[cubic]

for i in range(len(cubic[1])):
    