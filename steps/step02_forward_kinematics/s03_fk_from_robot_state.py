import numpy as np

from core.Robot import Robot as HardwareRobot
from steps.step02_forward_kinematics.s01_dh_fk_model import Robot as DHRobot

hw = HardwareRobot()
model = DHRobot()

q_deg = hw.get_joints_readings()[0]
fk = model.get_fk(q_deg)

print(f"End-effector to base transform:\n{fk}")

position = model.get_ee_pos(q_deg)

print(
    "Position (mm): x={:.2f}, y={:.2f}, z={:.2f} | yaw={:.2f} deg, pitch={:.2f} deg".format(
        position[0], position[1], position[2], position[3], position[4]
    )
)

# yaw = arctan(r21 / r11)
yaw = np.arctan2(fk[1, 0], fk[0, 0])

# roll = arctan(r32 / r33)
roll = np.arctan2(fk[2, 1], fk[2, 2])

# pitch = arcsin(-r31), check sign
pitch = np.arcsin(-fk[2, 0])
if fk[2, 0] > 0 and fk[2, 1] < 0:
    pitch = -pitch  # inversion

print(f"yaw = {yaw:.4f} rad, pitch = {pitch:.4f} rad, roll = {roll:.4f} rad")
