
import numpy as np
from TrajPlanner import TrajPlanner

# # Define setpoints (example values)
setpoints = np.array([
     [15, -45, -60, 90],
     [-90, 15, 30, -45]
 ])

# # Create a TrajPlanner object
trajectories = TrajPlanner(setpoints)

# # Generate cubic trajectory
cubic_traj = trajectories.get_cubic_traj(traj_time=5, points_num=6)
print(cubic_traj)