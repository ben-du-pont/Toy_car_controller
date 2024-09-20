import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

class Renderer:
    def __init__(self, simulator):
        self.simulator = simulator 

        self.vehicle = simulator.vehicle
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.path = simulator.path
        # Initialize the car and track visualization
        self.initialize_track(self.path)

        self.lookahead_distance = simulator.lookahead_distance

        self.car_length = 2.5
        self.car_width = 1.2
        self.wheel_length = 0.6
        self.wheel_width = 0.2

        # Initialize car body and wheels visualization
        self.car_body = patches.Rectangle((self.vehicle.state['x'], self.vehicle.state['y']), 
                                          self.car_length, self.car_width, color='blue', angle=np.degrees(self.vehicle.state['yaw']))
        self.ax.add_patch(self.car_body)
        

        

        self.waypoint, = self.ax.plot([], [], 'rx')
        self.waypoint_ahead, = self.ax.plot([], [], 'bo')

        # Initialize car wheels
        self.wheels = []
        for _ in range(4):
            wheel = patches.Rectangle((0, 0), 0.6, 0.2, color='black')
            self.ax.add_patch(wheel)
            self.wheels.append(wheel)

        # Speed and steering text
        self.speed_text = self.ax.text(0.5, 0.95, '', transform=self.ax.transAxes, ha='center')
        self.steering_text = self.ax.text(0.5, 0.90, '', transform=self.ax.transAxes, ha='center')

        # Initialize lap time display
        self.lap_time_text = self.ax.text(0.5, 0.85, '', transform=self.ax.transAxes, ha='center')


    def initialize_track(self, path):
        # Plot the path and other track details
        self.track_lines = []
        self.track_lines.append(self.ax.plot(path.px, path.py, color='black', linewidth=0.2)[0])
        self.track_lines.append(self.ax.plot(path.track_params[1].xy[0], path.track_params[1].xy[1], color='blue')[0])
        self.track_lines.append(self.ax.plot(path.track_params[2].xy[0], path.track_params[2].xy[1], color='yellow')[0])
        self.track_lines.append(self.ax.plot(path.track_params[4][:, 0], path.track_params[4][:, 1], color='orange')[0])

    def update_visuals(self):
        state = self.vehicle.get_state()

        # Update car position
        car_center_x = state['x'] - 2.5 / 2
        car_center_y = state['y'] - 1.2 / 2
        corner_x = state['x'] - (self.car_length / 2 * np.cos(state['yaw']) - self.car_width / 2 * np.sin(state['yaw']))
        corner_y = state['y'] - (self.car_length / 2 * np.sin(state['yaw']) + self.car_width / 2 * np.cos(state['yaw']))
        self.car_body.set_xy((corner_x, corner_y))
        self.car_body.angle = np.degrees(state['yaw'])

        # Update wheels' positions
        self.wheel_positions = [
            (self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(self.vehicle.state['yaw'] + self.vehicle.state['steer'])),
            (self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(self.vehicle.state['yaw'] + self.vehicle.state['steer'])),
            (-self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 + self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(self.vehicle.state['yaw'])),
            (-self.car_length * 0.25 - self.wheel_length * 0.5, self.car_width * 0.5 - self.car_width * 0.5 - self.wheel_width * 0.5 - self.car_width / 2, np.degrees(self.vehicle.state['yaw']))
        ]

        for wheel, (x_offset, y_offset, angle) in zip(self.wheels, self.wheel_positions):
            wheel_x = state['x'] + x_offset * np.cos(state['yaw']) - y_offset * np.sin(state['yaw'])
            wheel_y = state['y'] + x_offset * np.sin(state['yaw']) + y_offset * np.cos(state['yaw'])
            wheel.set_xy((wheel_x, wheel_y))
            wheel.angle = angle
            
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

        # Update speed and steering text
        self.speed_text.set_text(f'Speed: {state["v_x"]:.2f} m/s')
        self.steering_text.set_text(f'Steering: {state["steer"]:.2f} rad')

        
        if self.simulator.on_track:
            self.lap_time_text.set_text(f'Lap time: {self.simulator.timer:.2f}')
        else:
            
            self.lap_time_text.set_text('Out Of Bounds!')
            self.lap_time_text.set_fontproperties({'weight': 'bold'})
        
        if self.simulator.path.lap_count > 1:
            self.lap_time_text.set_fontproperties({'weight': 'bold'})
        
        # Adjust plot limits
        self.ax.set_xlim(state['x'] - 10, state['x'] + 10)
        self.ax.set_ylim(state['y'] - 10, state['y'] + 10)

    def render(self):
        """Update the visuals and handle the rendering."""
        self.update_visuals()
        plt.pause(0.001)  # Short pause for rendering
