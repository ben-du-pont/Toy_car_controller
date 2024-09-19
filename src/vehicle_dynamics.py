import numpy as np
from scipy.integrate import odeint

class Vehicle:
    def __init__(self, params):
        
        # Vehicle parameters
        self.m = params['mass']
        self.Iz = params['inertia']
        self.lf = params['front_length']
        self.lr = params['rear_length']
        self.w = params['width']
        self.h = params['height']
        self.Cl = params['lift_coefficient']
        self.Cr0 = params['rolling_resistance_constant']
        self.Cr2 = params['rolling_resistance_v_squared']
        self.hc = params['cog_height']
        self.Ef = params['front_track_width']
        self.Er = params['rear_track_width']
        
        # Tire parameters (Pacejka)
        self.B = params['pacejka_B']
        self.C = params['pacejka_C']
        self.D = params['pacejka_D']
        self.E = params['pacejka_E']
        self.H = params['pacejka_H']
        self.V = params['pacejka_V']
        
        # Other parameters
        self.g = 9.81  # gravitational acceleration
        self.rho = 1.225  # air density
        
        # Initial state
        self.state = {
            'x': 0,
            'y': 0,
            'yaw': 0,
            'v_x': params['initial_velocity'],
            'v_y': 0,
            'r': 0,
            'a_x': 0,
            'a_y': 0,
            'steer': 0
        }
    
    def pacejka_model(self, alpha, Fz):
        return Fz * self.D * np.sin(self.C * np.arctan(self.B * (alpha + self.H) - 
                                    self.E * (self.B * (alpha + self.H) - 
                                    np.arctan(self.B * (alpha + self.H))))) + self.V
    
    def calculate_slip_angles(self, v_x, v_y, r, delta):
        k1 = 0.74
        k2 = 1.2    
        epsilon1 = 8.5
        epsilon2 = 5.1

        alpha_FL = np.arctan((v_y + r * self.lf) * (v_x - 0.5 * self.Ef * r) * np.tanh(k1 * (v_x - 0.5 * self.Ef * r)) / 
                             ((v_x - 0.5 * self.Ef * r)**2 + epsilon1)) - delta
        
        alpha_FR = np.arctan((v_y + r * self.lf) * (v_x + 0.5 * self.Ef * r) * np.tanh(k1 * (v_x + 0.5 * self.Ef * r)) / 
                             ((v_x + 0.5 * self.Ef * r)**2 + epsilon1)) - delta
        
        alpha_RL = np.arctan((v_y - r * self.lr) * (v_x - 0.5 * self.Er * r) * np.tanh(k2 * (v_x - 0.5 * self.Er * r)) / 
                             ((v_x - 0.5 * self.Er * r)**2 + epsilon2))
        
        alpha_RR = np.arctan((v_y - r * self.lr) * (v_x + 0.5 * self.Er * r) * np.tanh(k2 * (v_x + 0.5 * self.Er * r)) / 
                             ((v_x + 0.5 * self.Er * r)**2 + epsilon2))
        
        return alpha_FL, -alpha_FR, alpha_RL, -alpha_RR
    
    def calculate_normal_forces(self, v_x, a_x, a_y):
        F_FL = (0.5 * self.lr * (self.m * self.g + 0.5 * self.Cl * v_x**2) / (self.lr + self.lf) - 
                0.5 * self.hc * a_x / (self.lr + self.lf) - 
                0.5 * self.hc * self.lr * a_y / (self.Ef * (self.lr + self.lf)))
        
        F_FR = (0.5 * self.lr * (self.m * self.g + 0.5 * self.Cl * v_x**2) / (self.lr + self.lf) - 
                0.5 * self.hc * a_x / (self.lr + self.lf) + 
                0.5 * self.hc * self.lr * a_y / (self.Ef * (self.lr + self.lf)))
        
        F_RL = (0.5 * self.lf * (self.m * self.g + 0.5 * self.Cl * v_x**2) / (self.lr + self.lf) + 
                0.5 * self.hc * a_x / (self.lr + self.lf) - 
                0.5 * self.hc * self.lf * a_y / (self.Er * (self.lr + self.lf)))
        
        F_RR = (0.5 * self.lf * (self.m * self.g + 0.5 * self.Cl * v_x**2) / (self.lr + self.lf) + 
                0.5 * self.hc * a_x / (self.lr + self.lf) + 
                0.5 * self.hc * self.lf * a_y / (self.Er * (self.lr + self.lf)))
        
        return F_FL, F_FR, F_RL, F_RR
    
    def vehicle_dynamics(self, state_values, t, throttle, steering_target):
        x, y, yaw, v_x, v_y, r, a_x, a_y, steer = state_values
        
        # Calculate slip angles
        alpha_fl, alpha_fr, alpha_rl, alpha_rr = self.calculate_slip_angles(v_x, v_y, r, steer)
        
        # Calculate normal forces
        Fz_fl, Fz_fr, Fz_rl, Fz_rr = self.calculate_normal_forces(v_x, a_x, a_y)
        
        # Calculate lateral forces using Pacejka model
        Fy_fl = self.pacejka_model(-alpha_fl, Fz_fl)
        Fy_fr = self.pacejka_model(alpha_fr, Fz_fr)
        Fy_rl = self.pacejka_model(-alpha_rl, Fz_rl)
        Fy_rr = self.pacejka_model(alpha_rr, Fz_rr)
        
        # Simplified longitudinal force model (proportional to throttle)
        Fx_fl = Fx_fr = Fx_rl = Fx_rr = throttle / 4
        
        # Calculate net forces and moment
        Fx_net = (Fx_rl + Fx_rr + Fx_fl * np.cos(steer) + Fx_fr * np.cos(steer) - 
                  Fy_fl * np.sin(steer) - Fy_fr * np.sin(steer) + 
                  self.m * v_y * r - self.Cr0 - self.Cr2 * v_x**2)
        
        Fy_net = (Fy_rl + Fy_rr + Fy_fl * np.cos(steer) + Fy_fr * np.cos(steer) + 
                  Fx_fl * np.sin(steer) + Fx_fr * np.sin(steer) - self.m * v_x * r)
        
        M_net = (self.lf * (Fx_fl * np.sin(steer) + Fx_fr * np.sin(steer) + 
                            Fy_fl * np.cos(steer) + Fy_fr * np.cos(steer)) - 
                 self.lr * (Fy_rl + Fy_rr) + 
                 self.w/2 * (Fy_fl * np.sin(steer) - Fy_fr * np.sin(steer) + 
                             Fx_fr * np.cos(steer) - Fx_fl * np.cos(steer)) + 
                 self.w/2 * (Fx_rr - Fx_rl))
        
        # State derivatives
        dx = v_x * np.cos(yaw) - v_y * np.sin(yaw)
        dy = v_x * np.sin(yaw) + v_y * np.cos(yaw)
        dyaw = r
        dv_x = Fx_net / self.m
        dv_y = Fy_net / self.m
        dr = M_net / self.Iz
        da_x = 0 #(Fx_net / self.m - a_x) / 0.1  # Assuming a time constant of 0.1s for acceleration
        da_y = 0#(Fy_net / self.m - a_y) / 0.1
        dsteer = (steering_target - steer) / 0.1  # Assuming a time constant of 0.1s for steering
        
        return [dx, dy, dyaw, dv_x, dv_y, dr, da_x, da_y, dsteer]
    
    def update(self, throttle, steering_target, dt):
        state_values = list(self.state.values())
        new_state_values = odeint(self.vehicle_dynamics, state_values, [0, dt], 
                                  args=(throttle, steering_target))
        
        new_state = dict(zip(self.state.keys(), new_state_values[-1]))
        self.state = new_state
        return self.state
    
    def get_state(self):
        return self.state