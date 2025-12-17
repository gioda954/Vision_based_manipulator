
import time
import pickle
from datetime import datetime
from classes.Robot import Robot

def main():
    seconds = 2.0          # duration [s]
    poll_dt = 0.02         # interval [s]
    q_target = [45, -30, 30, 75]  

    robot = Robot()
    try:
        
        robot.write_motor_state(True)
        robot.write_time(seconds)

        # Home
        robot.write_joints([0, 0, 0, 0])
        time.sleep(seconds)

        # Prealloc log
        t_log = []
        q_log = []

        
        robot.write_joints(q_target)
        t0 = time.perf_counter()
        while True:
            t = time.perf_counter() - t0
            q_deg, qd_dps, I_mA = robot.get_joints_readings()
            t_log.append(t)
            
            q_log.append([float(q_deg[0]), float(q_deg[1]), float(q_deg[2]), float(q_deg[3])])
            if t >= seconds:
                break
            
            remaining = poll_dt - (time.perf_counter() - t0 - t)
            if remaining > 0:
                time.sleep(remaining)

        
        q_deg, _, _ = robot.get_joints_readings()
        t_log.append(time.perf_counter() - t0)
        q_log.append([float(q_deg[0]), float(q_deg[1]), float(q_deg[2]), float(q_deg[3])])

       
        data = {"t": t_log, "q_deg": q_log}
        fname = "traj_log_test.pkl"
        with open(fname, "wb") as f:
            pickle.dump(data, f)

    finally:
        robot.close()

if __name__ == "__main__":
    main()
