import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

from simulator import Simulator
from controller import Controller

if __name__ == "__main__":

    controller = Controller()
    drone = Simulator(controller=controller.lqr_output)
    drone.run_simulation(target_x=4.0, target_y=7.0)
    drone.animate()