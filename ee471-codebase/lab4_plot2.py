import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import os
from datetime import datetime

PICKLE_PATH = "Part1_data.pkl"

# --- helper: get desktop path ---
def _desktop_path(filename):
    home = os.path.expanduser("~")
    return os.path.join(home, "Desktop", filename)

def _timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def _save_fig(fig, name):
    path = _desktop_path(f"{name}_{_timestamp()}.jpeg")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    print(f"Saved figure: {path}")

def _maybe_deg(arr):
    if arr.size == 0:
        return arr
    if np.nanmax(np.abs(arr)) < 6.283 * 1.2:
        return np.rad2deg(arr)
    return arr

def _equal_aspect_3d(ax, X, Y, Z):
    x_range = np.max(X) - np.min(X)
    y_range = np.max(Y) - np.min(Y)
    z_range = np.max(Z) - np.min(Z)
    max_range = max(x_range, y_range, z_range)
    x_mid = (np.max(X) + np.min(X)) / 2.0
    y_mid = (np.max(Y) + np.min(Y)) / 2.0
    z_mid = (np.max(Z) + np.min(Z)) / 2.0
    ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
    ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
    ax.set_zlim(z_mid - max_range/2, z_mid + max_range/2)

def plot_all(path=PICKLE_PATH):
    with open(path, "rb") as f:
        D = pickle.load(f)

    t  = np.asarray(D["time"])
    qm = np.asarray(D["q"])
    ee = np.asarray(D["ee_poses"])
    traj = D.get("traj", None)
    ee_wps = D.get("ee_waypoints", None)
    task_traj = D.get("task_traj", None)

    qm_deg = _maybe_deg(qm.copy())
    t_plan, qp_deg = None, None
    if traj is not None and traj.ndim == 2 and traj.shape[1] >= 5:
        t_plan = traj[:, 0]
        qp_deg = _maybe_deg(traj[:, 1:5])

    # (a) Joint angles vs time
    fig1 = plt.figure(figsize=(9, 5))
    labels = [f"q{i+1}" for i in range(qm_deg.shape[1])]
    for i, lab in enumerate(labels):
        plt.plot(t, qm_deg[:, i], label=f"{lab} measured", linewidth=1.5)
        if qp_deg is not None:
            plt.plot(t_plan, qp_deg[:, i], "--", label=f"{lab} planned", linewidth=1.0)
    plt.xlabel("Time [s]")
    plt.ylabel("Joint angle [deg]")
    plt.title("Joint angles vs time")
    plt.legend()
    plt.grid(True)
    _save_fig(fig1, "JointAngles")

    # (b) 3D End-effector path
    fig2 = plt.figure(figsize=(7, 6))
    ax = fig2.add_subplot(111, projection='3d')
    ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], linewidth=2, label="Actual path")
    if task_traj is not None:
        ax.plot(task_traj[:, 1], task_traj[:, 2], task_traj[:, 3],
                "--", linewidth=1.5, label="Planned path")
    if ee_wps is not None and len(ee_wps) >= 3:
        wp = np.unique(ee_wps[:, :3], axis=0)[:3]
        ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2], s=90, marker="^", label="Waypoints")
        for i, (x, y, z) in enumerate(wp, start=1):
            ax.text(x, y, z, str(i), fontsize=9)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title("3D End-Effector Trajectory")
    _equal_aspect_3d(ax, ee[:, 0], ee[:, 1], ee[:, 2])
    ax.legend()
    ax.grid(True)
    _save_fig(fig2, "3D_EndEffector")

    # (c) Task-space pose, velocity, acceleration
    vee = np.vstack([np.gradient(ee[:, i], t) for i in range(4)]).T
    aee = np.vstack([np.gradient(vee[:, i], t) for i in range(4)]).T
    fig3, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    names = ["x [mm]", "y [mm]", "z [mm]", "alpha [deg]"]
    for i in range(4):
        axs[0].plot(t, ee[:, i], label=names[i])
    axs[0].set_ylabel("Pose")
    axs[0].set_title("Task-space pose, velocity, acceleration")
    axs[0].legend(ncols=4, fontsize=9)
    axs[0].grid(True)

    for i in range(4):
        axs[1].plot(t, vee[:, i], label=f"{names[i].split()[0]} velocity")
    axs[1].set_ylabel("Velocity")
    axs[1].legend(ncols=4, fontsize=9)
    axs[1].grid(True)

    for i in range(4):
        axs[2].plot(t, aee[:, i], label=f"{names[i].split()[0]} acceleration")
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Acceleration")
    axs[2].legend(ncols=4, fontsize=9)
    axs[2].grid(True)

    plt.tight_layout()
    _save_fig(fig3, "TaskSpace_PVA")

    plt.show()

if __name__ == "__main__":
    plot_all()
