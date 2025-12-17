import numpy as np 
from lab2 import Robot

robot = Robot()

fk = Robot.get_current_fk()

print(f"the end effector to base matrix is : \n{fk}")

position = Robot.get_ee_pos(Robot.get_joint_readings)

print(f"the position is \n{position[0]}mm on x {position[1]}mm on y {position[2]}mm on z \n\n the yaw is {position[3]} and the pitch is {position[4]}")

# yaw = arctan(r21 / r11)
yaw = np.arctan2(fk[1, 0], fk[0, 0])

# roll = arctan(r32 / r33)
roll = np.arctan2(fk[2, 1], fk[2, 2])

# pitch = arcsin(-r31), check sign
pitch = np.arcsin(-fk[2, 0])
if fk[2, 0] > 0 and fk[2, 1] < 0:
    pitch = -pitch  # inversion

print(f"yaw = {yaw:.4f} rad, pitch = {pitch:.4f} rad, roll = {roll:.4f} rad")