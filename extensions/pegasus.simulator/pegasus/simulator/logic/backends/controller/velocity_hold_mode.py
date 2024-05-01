"""
This class will contain the velocity hold mode controller

Based on controller design from:   https://www.researchgate.net/publication/267409258_Flight_path_PID_controller_for_propeller-driven_fixed-_wing_unmanned_aerial_vehicles#fullTextFileContent 
https://www.researchgate.net/publication/257581364_Central_Processing_Unit_for_an_Autopilot_Description_and_Hardware-In-the-Loop_Simulation
page: 571 of last link is good example of how to test controllers.
"""

import numpy as np

from pegasus.simulator.logic.backends.controller.control_devices import Throttle
from pegasus.simulator.params import DEFAULT_WORLD_SETTINGS

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0 # Comment out after testing

    def compute_PID(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
        self.derivative = derivative # comment out after testing.
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output
    
class VelocityHoldMode:
    
    def __init__(self):
        
        self.current_velocity = 0.0
        self.ref_v = 20 #m/s

        self._physics_dt = DEFAULT_WORLD_SETTINGS["physics_dt"]

        self.v_e = 0.0

        self.kp = 1.0  # Proportional gain   
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain  
        self.controller = PIDController(self.kp,self.ki,self.kd)

        self.throttle_dot_max = 0.5 
    
    def update(self,dt,current_velocity,current_acc):
        ka = 0.1
        kc2 = 0.0  #0.5 kinda works but deleting the feed foward desired velocity makes it work instantly.
        v_e = self.ref_v - current_velocity #- current_acc*ka

        output = self.controller.compute_PID(v_e,dt)

        # scaling = 0.01
        # tht_delta = tht_delta *scaling

        ffwd_v = kc2 * self.ref_v
        ffwd_a = ka * current_acc
        
        output = output + ffwd_v + ffwd_a

        throttle_command = np.clip(output, 0.0,1.0)
        return throttle_command