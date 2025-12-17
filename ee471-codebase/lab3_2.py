import numpy as np
from prelab3 import get_ik
from lab2 import Robot

robot = Robot()

pose1 = [274, 0, 204, 0]
pose2 = [16, 4, 336, 15]
pose3 = [0, -270, 106, 0]


Ik1 = get_ik(pose1)
Ik2 = get_ik(pose2)
Ik3 = get_ik(pose3)


Fk1p = robot.get_ee_pos(Ik1[0])
Fk2p = robot.get_ee_pos(Ik2[0])
Fk3p = robot.get_ee_pos(Ik3[0])
Fk1r = robot.get_ee_pos(Ik1[1])
Fk2r = robot.get_ee_pos(Ik2[1])
Fk3r = robot.get_ee_pos(Ik3[1])

np.set_printoptions(precision=3, suppress=True)

print(Fk1p)
print(Fk2p)
print(Fk3p)
print(Fk1r)
print(Fk2r)
print(Fk3r)


