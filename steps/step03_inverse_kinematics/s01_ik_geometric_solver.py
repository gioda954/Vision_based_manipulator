import numpy as np

# lengths 
L1 = 77
L2 = 130
L3 = 124
L4 = 126
l21 = 128
l22 = 24

def get_ik(pose):

    x = pose[0]
    y = pose[1]
    z = pose[2]
    a= np.deg2rad(pose[3])

    r = np.sqrt(x**2 + y**2)
    rw = r - L4 * np.cos(a)
    zw = z - L1 - L4 * np.sin(a)
    dw = np.sqrt(rw**2 + zw**2)

    mu = np.arctan2(zw, rw)
    beta = np.arccos((L2**2 + L3**2 - dw**2) / (2 * L2 * L3))
    gamma = np.arccos((dw**2 + L2**2 - L3**2) / (2 * dw * L2))
    delta = np.arctan2(l22, l21)
    theta1 = np.arctan2(y, x)

    # elbow-up
    theta2 = np.pi/2 - delta - gamma - mu
    theta3 = np.pi/2 + delta - beta
    theta4 = -a - theta2 - theta3

    # elbow-down
    theta2d = np.pi/2 - delta + gamma - mu
    theta3d = np.pi/2 + delta + beta
    theta4d = -a - theta2d - theta3d

    sol1 = np.rad2deg([theta1, theta2, theta3, theta4])
    sol2 = np.rad2deg([theta1, theta2d, theta3d, theta4d])

    # Check
    if np.isnan(sol1).any() or np.isnan(sol2).any():
        raise ValueError("Target pose is unreachable")

    return sol1, sol2

# Test
tests = [
    ("Case 1", [274, 0, 204, 0]),
    ("Case 2", [16, 4, 336, 15]),
    ("Case 3", [0, -270, 106, 0]),
]

for name, pose in tests:
    try:
        sol1, sol2 = get_ik(pose)

        x = pose[0]
        y = pose[1]
        z = pose[2]
        a= np.deg2rad(pose[3])

        print(f"\n{name}  (x={x}, y={y}, z={z}, α={a}°)")
        print("  → Solution Elbow-Up:   ", np.round(sol1, 2))
        print("  → Solution Elbow-Down: ", np.round(sol2, 2))
    except ValueError as e:
        print(f"\n{name}: {e}")
