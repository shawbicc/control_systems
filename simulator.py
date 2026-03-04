import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

class Simulator:
    def __init__(self, controller, initial_state=[0, 2, 0, 0, 0, 0], m=1.0, J=0.1, L=0.25, g=9.81):
        self.m, self.J, self.L, self.g = m, J, L, g
        # State: [x, y, theta, vx, vy, omega]
        self.state = np.array(initial_state, dtype=float)
        
        # PID Gains (Tuned for stability)
        self.kp_y, self.kd_y = 15.0, 10.0   # Altitude
        self.kp_th, self.kd_th = 20.0, 10.0 # Attitude (Theta)
        self.kp_x, self.kd_x = 0.1, 0.2     # Horizontal (Outer loop)

        self.t_eval = np.linspace(0, 10, 300)
        self.history = []

        self.target_x = 3.0
        self.target_y = 7.0

        self.controller = controller

    def drone_dynamics(self, t, state, target_x, target_y):
        u1, u2 = self.controller(state, target_x, target_y)
        x, y, theta, vx, vy, omega = state
        
        # Nonlinear EoMs from Lecture [cite: 106, 122]
        d_state = [
            vx,
            vy,
            omega,
            -(u1 / self.m) * np.sin(theta),
            (u1 / self.m) * np.cos(theta) - self.g,
            u2 / self.J
        ]
        return d_state

    def run_simulation(self, target_x=2.0, target_y=7.0):
        self.target_x = target_x
        self.target_y = target_y
        sol = solve_ivp(
            self.drone_dynamics, 
            (0, 10), 
            self.state, 
            args=(target_x, target_y), 
            t_eval=self.t_eval,
            method='RK45'
        )
        self.sol = sol
        return sol

    def animate(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-5, 5); ax.set_ylim(0, 10)
        ax.set_aspect('equal'); ax.grid(True)
        
        line, = ax.plot([], [], 'o-', lw=4, color='blue')
        trail, = ax.plot([], [], '--', alpha=0.3, color='gray')
        target_dot, = ax.plot([self.target_x], [self.target_y], 'rx', markersize=10, label='Target')

        def update(frame):
            curr_x, curr_y, curr_theta = self.sol.y[0:3, frame]
            x_coords = [curr_x - self.L * np.cos(curr_theta), curr_x + self.L * np.cos(curr_theta)]
            y_coords = [curr_y - self.L * np.sin(curr_theta), curr_y + self.L * np.sin(curr_theta)]
            line.set_data(x_coords, y_coords)
            trail.set_data(self.sol.y[0, :frame], self.sol.y[1, :frame])
            return line, trail

        ani = FuncAnimation(fig, update, frames=len(self.t_eval), blit=True, interval=33)
        plt.legend()
        plt.show()
