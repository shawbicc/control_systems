from simulator import Simulator
from controller import Controller

if __name__ == "__main__":

    controller = Controller()
    drone = Simulator(controller=controller.pd_output) # select the controller
    drone.run_simulation(target_x=4.0, target_y=7.0)
    drone.animate(save_as_gif=True, filename="pd.gif")