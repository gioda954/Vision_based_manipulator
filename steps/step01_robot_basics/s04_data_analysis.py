
import numpy as np
import matplotlib.pyplot as plt
import pickle
from pathlib import Path

LOG_PATH = Path("assets/logs/traj_log_20251006_115821.pkl")



def load_from_pickle(path: Path):
    with open(path, "rb") as f:
        return pickle.load(f)



def plot1(data1, data2):

    colors = plt.cm.tab10(np.arange(4))

    fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)
    for i in range(4):
        axs[i].plot(data1, data2[:, i], color= colors[i])
        axs[i].set_ylabel(f"Joint {i+1} [deg]")
        axs[i].grid(True)

    axs[-1].set_xlim(0, 10)
    axs[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def plot2(time):

    delta = np.diff(time)
    plt.hist(delta, bins=100, edgecolor="black")
    plt.xlabel("delta time")
    plt.ylabel("frequency")
    plt.show()
    

def plot3(time):

    
    plt.hist(time, bins=25, edgecolor="black")
    plt.xlabel("delta time")
    plt.ylabel("frequency")
    plt.show()
      
  
def calc_pro(time):

    delta = np.diff(time)

    mean = np.mean(delta)
    s = np.std(delta)
    length = len(delta)
    median = np.median(delta)
    Max = np.max(delta)
    Min = np.min(delta)

    print(
        "Run stats: %d samples | mean=%.4f | median=%.4f | min=%.4f | max=%.4f | std=%.4f"
        % (length, mean, median, Min, Max, s)
    )



def filteroutliers(t):

    delta = np.diff(t)
    q1, q3 = np.percentile(delta, [25, 75])
    iqr = q3 - q1
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr

    delta_clean = delta[(delta >= lower) & (delta <= upper)]
    return delta_clean


            


def main():
    data = load_from_pickle(LOG_PATH)

    t = np.array(data["t"])
    Q = np.array(data["q_deg"])

    tf = filteroutliers(t)

    plot1(t, Q)

    plot2(t)

    plot3(tf)

    calc_pro(t)


if __name__ == "__main__":
    main()
