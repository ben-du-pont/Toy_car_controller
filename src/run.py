import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter

from config.controller_params import ControllerParams 
from config.vehicle_params import VehicleParams

from include.path import Path
from include.render import Renderer

from vehicle_dynamics import Vehicle
from controller import PurePursuitController, PIDController




matplotlib.use('TkAgg')


def main():
    path = Path()
    vehicle_params = VehicleParams().get_params()
    controller_params = ControllerParams().get_params()
    vehicle = Vehicle(vehicle_params)
    
    dt = 0.05  # Time step

    # Pure pursuit controller parameters
    L = controller_params['L']
    lookahead_distance = controller_params['Ld']

    # PID controller parameters
    kp = controller_params['kp']
    ki = controller_params['ki']
    kd = controller_params['kd']
    target_speed = controller_params['target_speed']
    
    pp_controller = PurePursuitController(L, lookahead_distance)
    pid_controller = PIDController(kp, kd, ki, dt, target_speed)

    renderer = Renderer(vehicle, path, pp_controller, pid_controller, dt)
    
    renderer.start_animation()


if __name__ == '__main__':
    main()




