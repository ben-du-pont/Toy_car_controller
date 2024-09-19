import numpy as np

class PurePursuitController:
    def __init__(self, wheelbase_length, lookahead_distance, max_steering_angle=0.45):
        """
        Initialize the Pure Pursuit controller.
        
        :param wheelbase_length: Length between the front and rear axles of the vehicle (L).
        :param lookahead_distance: The fixed distance from the vehicle to the lookahead point (Ld).
        """
        self.L = wheelbase_length
        self.Ld = lookahead_distance
        self.max_steering_angle = max_steering_angle

    def transform_to_vehicle_frame(self, x, y, theta, waypoint):
        """
        Transform waypoint coordinates to the vehicle's coordinate frame.
        
        :param x: Vehicle's x position.
        :param y: Vehicle's y position.
        :param theta: Vehicle's heading (orientation) angle in radians.
        :param waypoint: The (x, y) coordinates of the waypoint.
        :return: Transformed (x, y) coordinates of the waypoint in the vehicle's coordinate frame.
        """
        dx = waypoint[0] - x
        dy = waypoint[1] - y
        
        x_transformed = dx * np.cos(-theta) - dy * np.sin(-theta)
        y_transformed = dx * np.sin(-theta) + dy * np.cos(-theta)
        
        return x_transformed, y_transformed

    def calculate_steering_angle(self, x, y, theta, waypoint):
        """
        Calculate the steering angle to follow the path using the Pure Pursuit algorithm.
        
        :param x: Vehicle's x position.
        :param y: Vehicle's y position.
        :param theta: Vehicle's heading (orientation) angle in radians.
        :param waypoint: The (x, y) coordinates of the lookahead waypoint.
        :return: Required steering angle in radians.
        """
        x_transformed, y_transformed = self.transform_to_vehicle_frame(x, y, theta, waypoint)
        
        # Calculate the curvature using the Pure Pursuit formula
        kappa = (2 * y_transformed) / (self.Ld ** 2)
        
        # Calculate the steering angle
        steering_angle = np.arctan(kappa * self.L)
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        return steering_angle
    

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        """
        Initializes the PID Controller with gains and timestep.
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        :param dt: Time step for discrete differentiation and integration.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0
        self.integral = 0

        self.min_pid = -20
        self.max_pid = 20

    def reset(self):
        """Resets the PID controller state."""
        self.previous_error = 0
        self.integral = 0

    def scale_and_limit_pid_output(self, pid_output):
        """
        Scales the PID output to the range [-1, 1] and limits the output to that range.
        
        Args:
            pid_output (float): The raw output from the PID controller.
            min_pid (float): The expected minimum PID output (for example, -100).
            max_pid (float): The expected maximum PID output (for example, 100).
        
        Returns:
            float: The scaled and limited PID output, clamped to the range [-1, 1].
        """
        min_pid, max_pid = self.min_pid, self.max_pid
        # Ensure we avoid division by zero in case min_pid and max_pid are the same
        if max_pid - min_pid == 0:
            raise ValueError("max_pid and min_pid cannot be the same.")

        # Scale the PID output to the range [-1, 1]
        scaled_output = 2 * (pid_output - min_pid) / (max_pid - min_pid) - 1
        
        # Limit the output to [-1, 1]
        return max(-1, min(scaled_output, 1))

    def control(self, setpoint, measured_value):
        """
        Calculates the control action from a PID controller for a given setpoint and measured value.
        :param setpoint: The desired target value.
        :param measured_value: The current measured value.
        :return: Control input.
        """
        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        P = self.kp * error

        # Integral term
        self.integral += error * self.dt
        I = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        D = self.kd * derivative

        # Update previous error
        self.previous_error = error

        # Calculate total control input
        u = P + I + D

        return self.scale_and_limit_pid_output(u)