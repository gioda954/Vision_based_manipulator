import numpy as np
from prelab7 import Robot

# Make matrix output concise with fixed precision.
np.set_printoptions(precision=4, suppress=True)

robot = Robot()

configs = [
    ("Overhead singularity (q = [0, -10.62, -79.38, 0] deg)", np.array([0.0, -10.62, -79.38, 0.0])),
    ("Home configuration (q = [0, 0, 0, 0] deg)", np.array([0.0, 0.0, 0.0, 0.0])),
]

for label, q_deg in configs:
    J = robot.get_jacobian(q_deg)
    Jv = J[:3, :3]  # Upper-left 3x3 block (linear part for first three joints)
    det_Jv = np.linalg.det(Jv)

    print("\n====================================================")
    print(label)
    print("Jacobian J(q):")
    print(J)
    print("Linear Jacobian Jv (top-left 3x3):")
    print(Jv)
    print(f"det(Jv): {det_Jv:.4f}")
