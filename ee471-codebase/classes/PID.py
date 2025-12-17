import numpy as np


KP=0.5
KI=0.05
KD=0.1



class PIDController:
    def __init__(self, dim=3, dt=0.05):
# Initialize gains (tuned for position control in mm)
        self.Kp = KP * np.eye(dim) # Proportional gain
        self.Ki = KI * np.eye(dim) # Integral gain
        self.Kd = KD * np.eye(dim) # Derivative gain

# Initialize error terms
        self.error_integral = np.zeros(dim)
        self.error_prev = np.zeros(dim)
        self.dt = dt # Control period in seconds
    def compute_pid(self, error):

        D = (error - self.error_prev)/self.dt
        I= (error+ self.error_integral)*self.dt

        PID = (self.Kp@error)+(self.Kd@D)+(self.Ki@I)

        self.error_integral += error
        self.error_prev = error 


        return PID
    


    


