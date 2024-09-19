class ControllerParams:
    controller_params = {}

    def __init__(self):
        
        ## PID Controller parameters

        kp = 1.0 # Edit this
        ki = 0.01 # Edit this
        kd = 0.001 # Edit this

        target_speed = 13.0 # Edit this

        pid_min = -20 # This should not be changed - default value is -20
        pid_max = 20 # This should not be changed - default value is 20

        ## Pure Pursuit Controller parameters

        L = 2.0 # This should not be changed - default value is 2.0 (vehicle parameter)
        lookahead_distance = 9.0 # Edit this

        max_steering_angle = 0.5 # This should not be changed - default value is 0.5

        self.controller_params = {
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'target_speed': target_speed,
            'pid_min': pid_min,
            'pid_max': pid_max,

            'L': L,
            'Ld': lookahead_distance,
            'max_steering_angle': max_steering_angle # in radians
        }
    
    def get_params(self):
        return self.controller_params