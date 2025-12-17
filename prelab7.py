import numpy as np
from classes.PID import PIDController
import matplotlib.pyplot as plt



# Initialize controller with 50ms control period
controller = PIDController(dim=3, dt=0.05)
# Time vector (5 seconds of simulation)
t = np.linspace(0, 5, 801)

# Initialize arrays to store results
errors = np.zeros((len(t), 3))
outputs = np.zeros((len(t), 3))
# Set initial error (different for each axis to test independently)
error = np.array([10., 8., 6.]) # mm


# Simulate system response
for i in range(len(t)):
    dt = 0.05
# Store current error
    errors[i] = error

# Compute PID control output (desired velocity in mm/s)
    outputs[i] = controller.compute_pid(error)

# Simple model: error reduces due to control action
# Error reduction = velocity * timestep
    error = error - outputs[i] * 0.05

# Plot error vs time for each axis
plt.figure(figsize=(8, 4))
axis_labels = ["X-axis", "Y-axis", "Z-axis"]
for idx, label in enumerate(axis_labels):
    plt.plot(t, errors[:, idx], label=label, linewidth=1.5)
plt.title("PID Errors vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Error [mm]")
plt.grid(True)
plt.legend()

# Plot controller outputs vs time for each axis
plt.figure(figsize=(8, 4))
for idx, label in enumerate(axis_labels):
    plt.plot(t, outputs[:, idx], label=label, linewidth=1.5)
plt.title("PID Outputs vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Controller Output [mm/s]")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
