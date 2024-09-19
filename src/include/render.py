import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

class Renderer:
    def __init__(self, vehicle, path, pp_controller, pid_controller, dt):
        # Save the inputs as class attributes
        self.vehicle = vehicle
        self.path = path
        self.pp_controller = pp_controller
        self.pid_controller = pid_controller
        self.dt = dt
        self.lookahead_distance = pp_controller.Ld

        # Initialize the figure and axis
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')

        # Define car dimensions
        self.car_length = 2.5 # Visual length of the car
        self.car_width = 1.2 # Visual width of the car
        self.wheel_length = 0.6 # Visual length of the wheel
        self.wheel_width = 0.2 # Visual width of the wheel

        self.view_margin = 10  # Margin around the car for the plot view

        # Initialize car body
        self.car_body = patches.Rectangle((self.car_length/2, self.car_width/2), 
                                          self.car_length, self.car_width, 
                                          color='blue', 
                                          angle=np.degrees(self.vehicle.state['yaw']))
        self.ax.add_patch(self.car_body)

        # Plot the track and reference lines
        self.track_lines = []
        self.track_lines.append(self.ax.plot(self.path.px, self.path.py, color='black', linewidth=0.2)[0])
        self.track_lines.append(self.ax.plot(self.path.track_params[5].xy[0], self.path.track_params[5].xy[1], color='green', linewidth=0.2)[0])
        self.track_lines.append(self.ax.plot(self.path.track_params[6].xy[0], self.path.track_params[6].xy[1], color='green', linewidth=0.2)[0])
        self.track_lines.append(self.ax.plot(self.path.track_params[1].xy[0], self.path.track_params[1].xy[1], color='blue')[0])
        self.track_lines.append(self.ax.plot(self.path.track_params[2].xy[0], self.path.track_params[2].xy[1], color='yellow')[0])
        self.track_lines.append(self.ax.plot(self.path.track_params[4][:, 0], self.path.track_params[4][:, 1], color='orange')[0])

        # Initialize car wheels
        self.wheels = []
        for _ in range(4):
            wheel = patches.Rectangle((0, 0), self.wheel_length, self.wheel_width, color='black')
            self.ax.add_patch(wheel)
            self.wheels.append(wheel)

        # Initialize speed and steering angle text
        self.speed_text = self.ax.text(0.5, 0.95, '', transform=self.ax.transAxes, ha='center')
        self.steering_text = self.ax.text(0.5, 0.90, '', transform=self.ax.transAxes, ha='center')

        # Initialize waypoints
        self.waypoint, = self.ax.plot([], [], 'rx')
        self.waypoint_ahead, = self.ax.plot([], [], 'bo')

        # Activate the grid
        self.ax.grid(True)

        self.timer = 0
        self.collision = False
        # Initialize lap time display
        self.lap_time_text = self.ax.text(0.5, 0.85, '', transform=self.ax.transAxes, ha='center')


    def animate(self, i):
        current_time = i * self.dt
        state = self.vehicle.get_state()

        # Find current and ahead waypoints
        waypoint_idx = self.path.get_next_waypoint(state['x'], state['y'], 
                                                   self.path.track_params[3], 
                                                   self.path.track_params[0],
                                                   self.path.track_params[1],
                                                   self.path.track_params[2])
        waypoint_x, waypoint_y = self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_idx]['x'][0], \
                                 self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_idx]['y'][0]
        self.waypoint.set_xdata([waypoint_x])
        self.waypoint.set_ydata([waypoint_y])

        waypoint_ahead_idx = self.path.get_a_waypoint(state['x'], state['y'], 
                                                      self.path.track_params[3], 
                                                      self.path.track_params[0],
                                                      self.path.track_params[1],
                                                      self.path.track_params[2], 
                                                      self.lookahead_distance)
        
        waypoint_ahead_x, waypoint_ahead_y = self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_ahead_idx]['x'][0], \
                                             self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_ahead_idx]['y'][0]
        self.waypoint_ahead.set_xdata([waypoint_ahead_x])
        self.waypoint_ahead.set_ydata([waypoint_ahead_y])

        # Calculate inputs and update vehicle
        steering_input = self.pp_controller.calculate_steering_angle(
            self.vehicle.state['x'], self.vehicle.state['y'], self.vehicle.state['yaw'], 
            (waypoint_ahead_x, waypoint_ahead_y))
        throttle_input = self.pid_controller.control(self.pid_controller.target_speed, state['v_x'])

        self.vehicle.update(throttle_input, steering_input, self.dt)

        state = self.vehicle.get_state()

        # Update car body position and rotation
        car_center_x = state['x'] - self.car_length / 2
        car_center_y = state['y'] - self.car_width / 2
        corner_x = state['x'] - (self.car_length / 2 * np.cos(state['yaw']) - self.car_width / 2 * np.sin(state['yaw']))
        corner_y = state['y'] - (self.car_length / 2 * np.sin(state['yaw']) + self.car_width / 2 * np.cos(state['yaw']))
        self.car_body.set_xy((corner_x, corner_y))
        self.car_body.angle = np.degrees(state['yaw'])

        # Update speed and steering angle text
        self.speed_text.set_text(f'Speed: {state["v_x"]:.2f} m/s')
        self.steering_text.set_text(f'Steering: {state["steer"]:.2f} rad')

        # Set plot limits to follow the car
        
        self.ax.set_xlim(state['x'] - self.view_margin, state['x'] + self.view_margin)
        self.ax.set_ylim(state['y'] - self.view_margin, state['y'] + self.view_margin)

        self.path.increment_lap_count(state['x'], state['y'])

        self.collision = self.path.check_collision(state['x'], state['y'])
        if self.collision:
            self.timer = float('inf')
            self.lap_time_text.set_text('Out Of Bounds!')
            self.lap_time_text.set_fontproperties({'weight': 'bold'})
        # Check lap count to start/stop the timer
        else:
            if self.path.lap_count == 1:
                # Start the lap timer
                self.timer += self.dt
                self.lap_time_text.set_text(f'Lap Time: {self.timer:.2f}')

            elif self.path.lap_count > 1:
                if self.vehicle.state['v_x'] > 0.5:
                    self.pid_controller.target_speed = -5
                else:
                    self.pid_controller.target_speed = 0
                self.lap_time_text.set_text(f'Lap Time: {self.timer:.2f}')
                self.lap_time_text.set_fontproperties({'weight': 'bold'})


        # Update wheels' positions
        wheel_positions = [
            (self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(state['yaw'] + state['steer'])),
            (self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(state['yaw'] + state['steer'])),
            (-self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(state['yaw'])),
            (-self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(state['yaw']))
        ]

        for wheel, (x_offset, y_offset, angle) in zip(self.wheels, wheel_positions):
            wheel_x = state['x'] + x_offset * np.cos(state['yaw']) - y_offset * np.sin(state['yaw'])
            wheel_y = state['y'] + x_offset * np.sin(state['yaw']) + y_offset * np.cos(state['yaw'])
            wheel.set_xy((wheel_x, wheel_y))
            wheel.angle = angle

        # Return updated elements for blitting
        return self.car_body, *self.wheels, self.waypoint, self.waypoint_ahead, self.speed_text, self.steering_text, *self.track_lines

    def start_animation(self):
        # Create animation
        anim = FuncAnimation(self.fig, self.animate, frames=np.arange(0, 25, self.dt), 
                             interval=self.dt*1000, blit=False)
        

        # Save animation as gif
        # writer = PillowWriter(fps=1/self.dt)
        # anim.save('animation.gif', writer=writer)
        plt.show()



