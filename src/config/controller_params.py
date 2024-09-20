class ControllerParams:
    controller_params = {}

    def __init__(self):

        ## PID Controller parameters
        self.kp = 100.0 # Edit this
        self.ki = 0.01 # Edit this
        self.kd = 0.001 # Edit this

        self.target_speed = 15.0 # Edit this

        ## Pure Pursuit Controller parameters
        self.lookahead_distance = 9.0 # Edit this

        # Set the controller parameters
        self.set_controller_params()








    def set_controller_params(self):

        self.controller_params = {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'target_speed': self.target_speed,
            'pid_min': -20,
            'pid_max': 20,

            'L': 2.0,
            'Ld': self.lookahead_distance,
            'max_steering_angle': 0.5 # in radians
        }
    
    def get_params(self):
        return self.controller_params