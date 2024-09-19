class VehicleParams:

    vehicle_params = {}

    def __init__(self, mass=180, 
                inertia=165,
                front_length=0.86,
                rear_length=0.67,
                width=1.5,
                height=1.2,
                cog_height=0.23,
                front_track_width=1.5,
                rear_track_width=1.5,
                lift_coefficient=3.4,
                rolling_resistance_constant=0,
                rolling_resistance_v_squared=0.5,
                pacejka_B=30,
                pacejka_C=0.97,
                pacejka_D=2.0,
                pacejka_E=0.0,
                pacejka_H=0.0,
                pacejka_V=0.0,
                initial_velocity=0):
        
        self.vehicle_params = {
            'mass': mass,
            'inertia': inertia,
            'front_length': front_length,
            'rear_length': rear_length,
            'width': width,
            'height': height,
            'cog_height': cog_height,
            'front_track_width': front_track_width,
            'rear_track_width': rear_track_width,
            'lift_coefficient': lift_coefficient,
            'rolling_resistance_constant': rolling_resistance_constant,
            'rolling_resistance_v_squared': rolling_resistance_v_squared,
            'pacejka_B': pacejka_B,
            'pacejka_C': pacejka_C,
            'pacejka_D': pacejka_D,
            'pacejka_E': pacejka_E,
            'pacejka_H': pacejka_H,
            'pacejka_V': pacejka_V,
            'initial_velocity': initial_velocity
        }

    def get_params(self):
        return self.vehicle_params