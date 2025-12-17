import numpy as np, pickle

with open("Part1_data.pkl", "rb") as f:
    D = pickle.load(f)

q_meas = np.asarray(D["q"])             # measured joint angles
q_plan = np.asarray(D["traj"])[:, 1:5]  # planned joint angles (if same length)
rms_joint = np.sqrt(np.mean((q_meas - q_plan)**2, axis=0))
print("RMS joint error (deg):", np.rad2deg(rms_joint))
