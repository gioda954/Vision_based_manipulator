import numpy as np

class TrajPlanner:
    

    def __init__(self, setpoints):
        
        self.setpoints = setpoints

    
    def calc_cubic_coeff(self, t0, tf, p0, pf, v0, vf):
             
 
        A = np.array([
        [1, t0, t0**2, t0**3],
        [0, 1, 2*t0, 3*t0**2],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*tf**2]
        ])

        q = np.array([p0, v0, pf, vf])

        a = np.linalg.inv(A) @ q

        print(f"Cubic coeff for segment {p0} → {pf}: {a}")

        return(a)    

    def calc_cubic_traj(self, traj_time, points_num, coeff):

            t = np.linspace(0, traj_time, points_num+2)
            a0, a1, a2, a3 = coeff
            traj = a0 + a1*t + a2*t**2 + a3*t**3

            return traj

    def get_cubic_traj(self, traj_time, points_num):
        p0 = self.setpoints[0]
        pf = self.setpoints[1]
        v0 = np.zeros(4)
        vf = np.zeros(4)
        traj_all = np.zeros((points_num + 2, 4))
        for i in range(4):
            coeff = self.calc_cubic_coeff(0, traj_time, p0[i], pf[i], v0[i], vf[i])
            traj_all[:, i] = self.calc_cubic_traj(traj_time, points_num, coeff)
        return traj_all