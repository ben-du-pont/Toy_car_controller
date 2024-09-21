import matplotlib.pyplot as plt


from config.vehicle_params import VehicleParams

from include.path import Path
from include.render import Renderer
from include.physics_simulation import Simulator

from vehicle_dynamics import Vehicle
from controller import ControllerParams, PurePursuitController, PIDController


def main():
    path = Path()
    vehicle_params = VehicleParams().get_params()
    controller_params = ControllerParams().get_params()
    vehicle = Vehicle(vehicle_params)
    
    sim_dt = 0.1  # Simulation time step
    render_dt = 0.1  # Rendering time step

    # Pure pursuit controller parameters
    L = controller_params['L']
    lookahead_distance = controller_params['Ld']

    # PID controller parameters
    kp = controller_params['kp']
    ki = controller_params['ki']
    kd = controller_params['kd']
    target_speed = controller_params['target_speed']
    
    pp_controller = PurePursuitController(L, lookahead_distance)
    pid_controller = PIDController(kp, kd, ki, sim_dt, target_speed)


    
    # Create simulator and renderer
    simulator = Simulator(vehicle, path, pp_controller, pid_controller, sim_dt)
    renderer = Renderer(simulator)

    accumulated_sim_time = 0.0  # Virtual time accumulator

    plt.ion()  # Enable interactive mode for real-time plotting

    try:
        while True:
            # Simulate as many steps as possible until the render time interval is reached
            while accumulated_sim_time < render_dt:
                simulator.step()  # Run one simulation step
                accumulated_sim_time += sim_dt  # Accumulate simulation time

            # Reset accumulator and render the frame
            accumulated_sim_time -= render_dt  # Allow for fractional steps
            renderer.render()
            if simulator.path.lap_count > 1 and simulator.lap_time > 5:
                break

    except KeyboardInterrupt:
        pass

    plt.ioff()  # Disable interactive mode

if __name__ == '__main__':
    main()




