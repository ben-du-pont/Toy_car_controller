class Simulator:

    def __init__(self, vehicle, path, pp_controller, pid_controller, dt):
        self.vehicle = vehicle
        self.path = path
        self.pp_controller = pp_controller
        self.pid_controller = pid_controller
        self.dt = dt
        self.lookahead_distance = pp_controller.Ld
        self.simulated_time = 0.0  # Initialize virtual simulation time

        self.on_track = True
        self.timer = 0.0
        self.lap_time = 0.0
        self.lap_number = 0

    def step(self):
        """Run a single simulation step."""
        # Get current state and waypoints (same as before)
        state = self.vehicle.get_state()

        waypoint_ahead_idx = self.path.get_a_waypoint(state['x'], state['y'], 
                                                      self.path.track_params[3], 
                                                      self.path.track_params[0],
                                                      self.path.track_params[1],
                                                      self.path.track_params[2], 
                                                      self.lookahead_distance)

        waypoint_ahead_x, waypoint_ahead_y = self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_ahead_idx]['x'][0], \
                                             self.path.track_params[0][self.path.track_params[0]['index'] == waypoint_ahead_idx]['y'][0]

        # Compute control inputs
        steering_input = self.pp_controller.calculate_steering_angle(
            self.vehicle.state['x'], self.vehicle.state['y'], self.vehicle.state['yaw'], 
            (waypoint_ahead_x, waypoint_ahead_y))
        throttle_input = self.pid_controller.control(self.pid_controller.target_speed, state['v_x'])

        # Update vehicle state
        self.vehicle.update(throttle_input, steering_input, self.dt)

        # Check collisions and update lap count (same as before)
        self.path.increment_lap_count(state['x'], state['y'])
        self.path.check_collision(state['x'], state['y'])

        self.on_track = not self.path.check_collision(state['x'], state['y'])

        self.lap_time += self.dt

        if self.lap_number != self.path.lap_count:
            self.lap_number = self.path.lap_count
            self.lap_time = 0.0
            if self.lap_number == 2:
                print(f'Lap time: {self.timer:.2f} s')


        if not self.on_track:
            self.timer = float('inf')
            
        # Check lap count to start/stop the timer
        else:
            if self.path.lap_count == 1:
                # Start the lap timer
                self.timer += self.dt
                
                

            elif self.path.lap_count > 1:

                if self.vehicle.state['v_x'] > 0.5:
                    self.pid_controller.target_speed = -5
                else:
                    self.pid_controller.target_speed = 0
                

        # Increment virtual simulation time
        self.simulated_time += self.dt