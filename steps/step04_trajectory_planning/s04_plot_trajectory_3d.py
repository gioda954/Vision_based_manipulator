import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from pathlib import Path

PICKLE_PATH = Path("assets/logs/Part1_data.pkl")

def _maybe_deg(arr):
    """Auto-convert radians to degrees if values look like radians."""
    if arr.size == 0:
        return arr
    if np.nanmax(np.abs(arr)) < 6.283 * 1.2:  # likely radians
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

    # Required fields from your collection script
    t  = np.asarray(D["time"])                 # [N]
    qm = np.asarray(D["q"])                    # measured joints [N,4] (rad or deg)
    ee = np.asarray(D["ee_poses"])                   # [N,4] = x,y,z [mm], alpha [deg]
    traj = D.get("traj", None)                 # planned [N, 1+4] = t, q1..q4
    ee_wps = D.get("ee_waypoints", None)       # waypoints in task space [M,4]

    # --- a) Joint angles vs time (deg), optional overlay planned ---
    qm_deg = _maybe_deg(qm.copy())
    t_plan, qp_deg = None, None
    if traj is not None and traj.ndim == 2 and traj.shape[1] >= 5:
        t_plan = traj[:, 0]
        qp_deg = _maybe_deg(traj[:, 1:5])

    plt.figure(figsize=(9, 5))
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

    # --- b) 3D end-effector trajectory with waypoints; equal aspect ---
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], linewidth=2, label="Actual path")
    ax.scatter(ee[0, 0], ee[0, 1], ee[0, 2], s=50, label="Start")
    ax.scatter(ee[-1, 0], ee[-1, 1], ee[-1, 2], s=50, label="End")

    # Mark three waypoints clearly (use first three unique from ee_waypoints if present)
    if ee_wps is not None and len(ee_wps) >= 3:
        wp_unique = np.unique(ee_wps[:, :3], axis=0)  # dedup on x,y,z
        wp_to_plot = wp_unique[:3] if wp_unique.shape[0] >= 3 else wp_unique
        ax.scatter(wp_to_plot[:, 0], wp_to_plot[:, 1], wp_to_plot[:, 2],
                   s=80, marker="^", label="Waypoints")
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title("3D end-effector trajectory")
    _equal_aspect_3d(ax, ee[:, 0], ee[:, 1], ee[:, 2])
    ax.legend()
    ax.grid(True)

    # --- c) Task-space pose, velocity, acceleration ---
    # Pose already in ee: x,y,z [mm], alpha [deg]
    # Derivatives via gradient over non-uniform t
    # Guard for strictly increasing time
    assert np.all(np.diff(t) > 0), "Time must be strictly increasing for derivatives."
    vee = np.vstack([np.gradient(ee[:, i], t) for i in range(4)]).T      # [N,4]
    aee = np.vstack([np.gradient(vee[:, i], t) for i in range(4)]).T     # [N,4]

    fig2, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    names = ["x [mm]", "y [mm]", "z [mm]", "alpha [deg]"]

    # Pose
    for i in range(4):
        axs[0].plot(t, ee[:, i], label=names[i])
        axs[0].set_ylabel("Pose")
        axs[0].set_title("Task-space pose, velocity, acceleration")
        axs[0].grid(True)
        axs[0].legend(ncols=4, fontsize=9)

    # Velocity
    v_units = ["mm/s", "mm/s", "mm/s", "deg/s"]
    for i in range(4):
        axs[1].plot(t, vee[:, i], label=f"{names[i].split()[0]} [{v_units[i]}]")
        axs[1].set_ylabel("Velocity")
        axs[1].grid(True)
        axs[1].legend(ncols=4, fontsize=9)

    # Acceleration
    a_units = ["mm/s²", "mm/s²", "mm/s²", "deg/s²"]
    for i in range(4):
        axs[2].plot(t, aee[:, i], label=f"{names[i].split()[0]} [{a_units[i]}]")
        axs[2].set_xlabel("Time [s]")
        axs[2].set_ylabel("Acceleration")
        axs[2].grid(True)
        axs[2].legend(ncols=4, fontsize=9)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_all()
