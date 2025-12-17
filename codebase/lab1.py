# lab1.py
# EE 471 — Lab 1: Multi-Joint Readings, Data Analysis, and Visualization
# Requirements: time, numpy, matplotlib.pyplot, provided Robot class

import time
import numpy as np
import matplotlib.pyplot as plt

# Assume the course provides this class in your environment.
# If the module name differs (e.g., ee471_robot), adjust the import accordingly.
from robot import Robot  # noqa: F401


def move_and_sample(robot: "Robot", target_deg, traj_time_s: float):
    """
    Issue a single multi-joint motion command and sample joint angles continuously.

    Args:
        robot: Robot instance with write_joints([...]) and read_joints() in degrees.
        target_deg: iterable of 4 joint targets in degrees.
        traj_time_s: desired trajectory time in seconds.

    Returns:
        t: np.ndarray, shape (N,)
        q: np.ndarray, shape (N,4), joint angles in degrees
    """
    # Issue one time-scaled multi-joint move. The servo firmware handles the profile internally.
    robot.write_joints(list(target_deg), traj_time_s)

    t_list = []
    q_list = []

    t0 = time.perf_counter()
    # Sample for the full requested window without enforcing a fixed rate
    while True:
        now = time.perf_counter()
        t_rel = now - t0
        t_list.append(t_rel)
        # Read present positions (degrees). Do not command the gripper here.
        q = robot.read_joints()  # expected: list/tuple length >= 4, in degrees
        q_list.append([q[0], q[1], q[2], q[3]])
        if t_rel >= traj_time_s:
            break

    # Convert to NumPy arrays with required shapes
    t = np.asarray(t_list, dtype=float)          # (N,)
    q = np.asarray(q_list, dtype=float)          # (N,4)
    return t, q


def plot_joint_traces(t, q, title_suffix):
    """
    Four subplots: one per joint. X: time (s). Y: angle (deg).
    """
    fig, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    joint_labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4"]
    for j in range(4):
        axes[j].plot(t, q[:, j], label=joint_labels[j])
        axes[j].set_ylabel("Angle (deg)")
        axes[j].set_title(f"{joint_labels[j]} vs Time {title_suffix}")
        axes[j].grid(True, which="both", alpha=0.3)
        axes[j].legend(loc="best")
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()


def analyze_timing(t, title_suffix):
    """
    Compute dt stats and plot histogram(s).
    """
    if t.size < 2:
        print("Insufficient samples for timing statistics")
        return

    dt = np.diff(t)
    mean_dt = float(np.mean(dt))
    med_dt = float(np.median(dt))
    min_dt = float(np.min(dt))
    max_dt = float(np.max(dt))
    std_dt = float(np.std(dt))

    print(f"Sampling interval statistics {title_suffix}")
    print(f"  N samples:           {t.size}")
    print(f"  N intervals:         {dt.size}")
    print(f"  mean(dt)     [s]:    {mean_dt:.6f}")
    print(f"  median(dt)   [s]:    {med_dt:.6f}")
    print(f"  min(dt)      [s]:    {min_dt:.6f}")
    print(f"  max(dt)      [s]:    {max_dt:.6f}")
    print(f"  std(dt)      [s]:    {std_dt:.6f}")

    # Full-range histogram
    plt.figure(figsize=(7, 4))
    plt.hist(dt, bins=100)
    plt.xlabel("Sampling interval Δt (s)")
    plt.ylabel("Count")
    plt.title(f"Δt Histogram {title_suffix} (full range)")
    plt.grid(True, alpha=0.3)

    # Optional focused histogram to reveal the main cluster (e.g., up to 50 ms)
    upper = 0.050  # 50 ms
    if np.any(dt > upper):
        plt.figure(figsize=(7, 4))
        plt.hist(dt[(dt >= 0) & (dt <= upper)], bins=100)
        plt.xlabel("Sampling interval Δt (s)")
        plt.ylabel("Count")
       
        