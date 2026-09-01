class SelfDriveController:
    def __init__(self):
        self.base_linear_speed = 0.25
        self.kp_angular = -0.018
        self.kd_angular = -0.005
        self.k_slow = 0.007
        self.last_error = 0.0

    def calculate_speeds(self, error: float, line_found: bool) -> tuple[float, float]:
        if not line_found:
            self.last_error = 0.0
            return 0.0, 0.0
            
        error_derivative = error - self.last_error
        self.last_error = error

        angular = (self.kp_angular * error) + (self.kd_angular * error_derivative)
        linear = self.base_linear_speed * max(0.2, 1.0 - min(1.0, self.k_slow * abs(error)))
        
        return angular, linear
        
    def reset(self):
        self.last_error = 0.0

