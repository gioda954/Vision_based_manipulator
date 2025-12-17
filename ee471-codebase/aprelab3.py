import numpy as np

# ---------------------------
# Robot link lengths (mm)
# ---------------------------
L1 = 77
L2 = 130
L3 = 124
L4 = 126
l21 = 128
l22 = 24

def clamp(val, lo, hi):
    """Clamp value between lo and hi to avoid domain errors."""
    return max(lo, min(hi, val))

def get_ik(pose):
    """
    Computes both elbow-up and elbow-down inverse kinematics solutions
    for the given end-effector pose [x, y, z, alpha_deg].
    """
    x, y, z, alpha_deg = pose
    alpha = np.deg2rad(alpha_deg)

    # Step 1: Wrist position
    r = np.sqrt(x**2 + y**2)
    rw = r - L4 * np.cos(alpha)
    zw = z - L1 - L4 * np.sin(alpha)
    dw = np.sqrt(rw**2 + zw**2)

    # Step 2: Intermediate angles
    mu = np.arctan2(zw, rw)
    beta = np.arccos(clamp((L2**2 + L3**2 - dw**2) / (2 * L2 * L3), -1.0, 1.0))
    gamma = np.arccos(clamp((dw**2 + L2**2 - L3**2) / (2 * dw * L2), -1.0, 1.0))
    delta = np.arctan2(l22, l21)
    theta1 = np.arctan2(y, x)

    # Step 3: Two possible solutions
    # Elbow-up
    theta2_up = np.pi/2 - delta - gamma - mu
    theta3_up = np.pi/2 + delta - beta
    theta4_up = -alpha - theta2_up - theta3_up

    # Elbow-down
    theta2_down = np.pi/2 - delta + gamma - mu
    theta3_down = np.pi/2 + delta + beta
    theta4_down = -alpha - theta2_down - theta3_down

    sol1 = np.rad2deg([theta1, theta2_up, theta3_up, theta4_up])
    sol2 = np.rad2deg([theta1, theta2_down, theta3_down, theta4_down])

    # Check reachability
    if np.isnan(sol1).any() or np.isnan(sol2).any():
        raise ValueError("Target pose is unreachable.")

    return sol1, sol2


# ---------------------------
# Test cases
# ---------------------------
tests = [
    ("Case 1", [274, 0, 204, 0]),
    ("Case 2", [16, 4, 336, 15]),
    ("Case 3", [0, -270, 106, 0]),
]

# ---------------------------
# Run tests and print results
# ---------------------------
for name, pose in tests:
    try:
        sol1, sol2 = get_ik(pose)
        print(f"\n{name}  (x={pose[0]}, y={pose[1]}, z={pose[2]}, α={pose[3]}°)")
        print("  → Solution 1 (Elbow-Up):   ", np.round(sol1, 2))
        print("  → Solution 2 (Elbow-Down): ", np.round(sol2, 2))
    except ValueError as e:
        print(f"\n{name}: {e}")