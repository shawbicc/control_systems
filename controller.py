import numpy as np
from scipy.linalg import solve_continuous_are

class Controller():
    def __init__(self, physical_properties=[1.0, 0.1, 0.25, 9.81]):
        # get the physical properties
        self.m, self.J, self.L, self.g = physical_properties

        # pre-calculate gain for lqr
        self.K_lqr = self._compute_lqr_gain()


    def pd_output(self, state, target_x, target_y):
        x, y, theta, vx, vy, omega = state
        # calculate the errors
        error_x = target_x - x
        error_y = target_y - y

        #pd gains
        kp_x, kd_x = 0.1, 0.2 # horizontal
        kp_y, kd_y = 15.0, 10.0 # altitude
        kp_theta, kd_theta = 20.0, 10.0 # attitude

        # u1 calculation
        u1 = (self.m * self.g) + (kp_y * error_y) - (kd_y * vy)
        u1 = np.clip(u1, 0, 2*self.m*self.g) # limit thrust

        # u2 calculation
        target_theta = -(kp_x * error_x - kd_x * vx)
        target_theta = np.clip(target_theta, -0.5, 0.5) # tilt limit
        error_theta = target_theta - theta
        u2 = (error_theta * kp_theta) - (omega * kd_theta)

        return u1, u2
    
    def _compute_lqr_gain(self):
        # 1. Define Linearized Matrices A and B at hover (Slide 31) [cite: 440]
        # State vector: [x, y, theta, vx, vy, omega]
        A = np.zeros((6, 6))
        A[0, 3] = 1.0  # dx/dt = vx
        A[1, 4] = 1.0  # dy/dt = vy
        A[2, 5] = 1.0  # dtheta/dt = omega
        A[3, 2] = -self.g # dvx/dt = -g*theta (small angle approx) [cite: 440]
        
        B = np.zeros((6, 2))
        B[4, 0] = 1.0 / self.m # dvy/dt = u1/m
        B[5, 1] = 1.0 / self.J # domega/dt = u2/J [cite: 440]

        # 2. Define Weights Q and R (Slide 18-19) [cite: 630, 633, 663]
        # Q: Penalize state error [x, y, theta, vx, vy, omega]
        Q = np.diag([10, 10, 10, 1, 1, 1]) 
        # R: Penalize control effort [u1, u2]
        R = np.diag([0.1, 0.1])

        # 3. Solve Riccati Equation and get K (Slide 20) [cite: 685, 688, 699]
        S = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ S
        return K

    def lqr_output(self, state, target_x, target_y):
        # Current state
        x, y, theta, vx, vy, omega = state
        current_state = np.array([x, y, theta, vx, vy, omega])
        
        # Target state (hover at target_x, target_y) [cite: 512, 571]
        target_state = np.array([target_x, target_y, 0, 0, 0, 0])
        
        # State deviation [cite: 621, 680]
        state_error = current_state - target_state
        
        # Compute control deviation: u_tilde = -K * x_tilde
        # Note: Scipy's ARE solution usually implies u = -Kx
        u_tilde = -self.K_lqr @ state_error
        
        # Nominal control for hover (Slide 7) [cite: 514, 690]
        u0 = np.array([self.m * self.g, 0.0])
        
        # Total control: u = u0 + u_tilde
        u = u0 + u_tilde
        
        # Safety clip
        u1 = np.clip(u[0], 0, 2 * self.m * self.g)
        u2 = u[1]
        
        return u1, u2
