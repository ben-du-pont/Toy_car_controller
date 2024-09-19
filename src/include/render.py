import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches
import numpy as np

class Renderer:

    def __init__(self, path, vehicle, dt=0.1):
        self.path = path
        self.vehicle = vehicle
        self.dt = dt

        # Initialize the figure and axes
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.ax.set_xlim(-10, 50)
        self.ax.set_ylim(-20, 20)

        # Define car dimensions
        self.car_length = 2.0
        self.car_width = 1.0
        self.wheel_length = 0.4
        self.wheel_width = 0.15

        # Initialize car body and wheels
        self.car_body = patches.Rectangle((self.car_length/2, self.car_width/2), self.car_length, self.car_width, color='blue', angle=np.degrees(vehicle.state['yaw']))
        self.ax.add_patch(self.car_body)

        self.ax.plot(path.px, path.py, color='black', linewidth=0.2)

        # add recessed line
        self.x_recessed, self.y_recessed = self.path.track_params[5].xy
        self.ax.plot(self.x_recessed, self.y_recessed, color='green', linewidth=0.2)

        #add recessed line2
        self.x_recessed2, self.y_recessed2 = self.path.track_params[6].xy
        self.ax.plot(self.x_recessed2, self.y_recessed2, color='green', linewidth=0.2)

        # add self.blue_line
        self.x_blue, self.y_blue = self.path.track_params[1].xy
        self.ax.plot(self.x_blue, self.y_blue, color='blue')
        self.x_yellow, self.y_yellow = path.track_params[2].xy
        self.ax.plot(self.x_yellow, self.y_yellow, color='yellow')
        # plot path.start_finish_line
        self.ax.plot(path.track_params[4][:, 0], path.track_params[4][:, 1], color='orange')

        self.wheels = []
        for _ in range(4):
            wheel = patches.Rectangle((0, 0), self.wheel_length, self.wheel_width, color='black')
            self.ax.add_patch(wheel)
            self.wheels.append(wheel)
        # Initialize text for speed and steering angle
        self.speed_text = self.ax.text(0.5, 0.95, '', transform=self.ax.transAxes, ha='center')
        self.steering_text = self.ax.text(0.5, 0.90, '', transform=self.ax.transAxes, ha='center')

        self.waypoint, = self.ax.plot([], [], 'rx')


    def animate(self, i):
        self.current_time = i * self.dt

        self.state = self.vehicle.get_state()

        waypoint_idx = self.path.get_next_waypoint(self.state['x'], self.state['y'], self.path.track_params[3], self.path.track_params[0],self.path.track_params[1],self.path.track_params[2])
        waypoint_x, waypoint_y = self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_idx]['x'][0], self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_idx]['y'][0]
        self.waypoint.set_data((waypoint_x, waypoint_y))

        self.state = self.vehicle.get_state()

        

        # Update car body
        # car_center_x = state['x'] - car_length / 2 - car_length / 2 * np.cos(state['yaw'])
        # car_center_y = state['y'] - car_width / 2 - car_width / 2 * np.sin(state['yaw'])
        corner_x = self.state['x'] - (self.car_length / 2 * np.cos(self.state['yaw']) - self.car_width / 2 * np.sin(self.state['yaw']))
        corner_y = self.state['y']  - (self.car_length / 2 * np.sin(self.state['yaw']) + self.car_width / 2 * np.cos(self.state['yaw']))

        self.car_body.set_xy((corner_x, corner_y))
        self.car_body.angle = np.degrees(self.state['yaw'])

        # Update speed and steering angle text
        self.speed_text.set_text(f'Speed: {self.state["v_x"]:.2f} m/s')
        self.steering_text.set_text(f'Steering: {self.state["steer"]:.2f}rad')

        # Set plot limits to follow the car
        view_margin = 10  # Margin around the car for the plot view
        self.ax.set_xlim(self.state['x'] - view_margin, self.state['x'] + view_margin)
        self.ax.set_ylim(self.state['y'] - view_margin, self.state['y'] + view_margin)
        # Activate the grid
        self.ax.grid(True)

        # Positions of the wheels relative to the car
        wheel_positions = [(self.car_length * 0.25 - self.wheel_length*0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width*0.5- self.car_width / 2, np.degrees(self.state['yaw'] + self.state['steer'])), 
                        (self.car_length * 0.25 - self.wheel_length*0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width*0.5- self.car_width / 2, np.degrees(self.state['yaw'] + self.state['steer'])),
                        (-self.car_length * 0.25 - self.wheel_length*0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width*0.5- self.car_width / 2, np.degrees(self.state['yaw'])),
                        (-self.car_length * 0.25 - self.wheel_length*0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width*0.5- self.car_width / 2, np.degrees(self.state['yaw']))]

        for wheel, (x_offset, y_offset, angle) in zip(self.wheels, wheel_positions):
            # Calculate the global position of each wheel
            wheel_x = self.state['x'] + x_offset * np.cos(self.state['yaw']) - y_offset * np.sin(self.state['yaw'])
            wheel_y = self.state['y'] + x_offset * np.sin(self.state['yaw']) + y_offset * np.cos(self.state['yaw'])
            wheel.set_xy((wheel_x, wheel_y))
            wheel.angle = angle
