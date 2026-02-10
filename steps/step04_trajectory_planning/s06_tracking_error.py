import numpy as np
import pickle
from pathlib import Path

LOG_PATH = Path("assets/logs/Part1_data.pkl")

with open(LOG_PATH, "rb") as f:
    D = pickle.load(f)

q_meas = np.asarray(D["q"])             # measured joint angles
q_plan = np.asarray(D["traj"])[:, 1:5]  # planned joint angles (if same length)
rms_joint = np.sqrt(np.mean((q_meas - q_plan)**2, axis=0))
print("RMS joint error (deg):", np.rad2deg(rms_joint))
