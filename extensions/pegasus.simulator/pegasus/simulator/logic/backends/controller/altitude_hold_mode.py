

"""
Ideas: 

Convert the rate of change in altitule, try to alter the angle of attack that way.

Calculate the desired rate of change from the extra force applied 
"""

import numpy as np

from pegasus.simulator.logic.backends.controller.states import AngleOfAttack
from pegasus.simulator.params import DEFAULT_WORLD_SETTINGS


## Find out update rate of angle af attack per second, then find out dt of simulation step. 

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0

    def compute_PID(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output
    
class AltitudeHoldMode():


    def __init__(self):

        

        self.current_altitude = 0.0  # Initialize altitude variable 

        self.aoa = AngleOfAttack()

        self.ref_altitude = 30.0

        self.h_e = 0.0

        self._physics_dt = DEFAULT_WORLD_SETTINGS["physics_dt"]

        self.kp = 0.1  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.0  # Derivative gain

        self.controller = PIDController(self.kp,self.ki,self.kd)

        self.alphadot_max = 2 # deg/s

    # inner loop alpha
    
    def update_aoa(self, aoa, aoa_adjustment):
        return
        

    def update(self, current_altitude, dt):
        h_e = self.ref_altitude - current_altitude

        aoa_delta = self.controller.compute_PID(h_e, dt)

        if aoa_delta > 0:
            aoa_delta = min(aoa_delta, self.alphadot_max*self._physics_dt)

        if aoa_delta < 0: 
            aoa_delta = max(aoa_delta, -self.alphadot_max*self._physics_dt)

        print("Angle of attack delta", aoa_delta)

        aoa = self.aoa.get_aoa()
        new_aoa = aoa + aoa_delta
        new_aoa = max(min(new_aoa, 12.2), -5.0)

        # new_aoa = max(min(new_aoa, MAX_AOA), MIN_AOA)
        # if aoa <12.2:
        #     new_aoa = aoa + aoa_delta
        # elif aoa > 12.2:
        #     new_aoa = aoa
        # elif aoa < -5.0: # TODO set right minimum angle of attack
        #     new_aoa = -5.0
        # else:
        #     new_aoa = aoa
        print("angle of attack: ", new_aoa)

        self.aoa.set_angle_of_attack(new_aoa)


        
"""
Rate of change of angle of attack around 1 deg per second . 

in simulation step is dt=0.004

1 sec = 250 simulation steps 

"""

# class PIDController:
#     def __init__(self, kp, ki, kd):
#         self.kp = kp  # Proportional gain
#         self.ki = ki  # Integral gain
#         self.kd = kd  # Derivative gain
#         self.previous_error = 0.0
#         self.integral = 0.0

#     def compute(self, error, delta_time):
#         self.integral += error * delta_time
#         derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0
#         output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
#         self.previous_error = error
#         return output

# class AltitudeHoldMode:
#     def __init__(self, desired_altitude, kp, ki, kd):
#         self.desired_altitude = desired_altitude
#         self.pid_controller = PIDController(kp, ki, kd)
#         self.aoa_sensor = AngleOfAttackSensor()  # Assuming this class is defined elsewhere

#     def update(self, current_altitude, vertical_velocity, vertical_acceleration, delta_time):
#         altitude_error = self.desired_altitude - current_altitude
#         aoa_adjustment = self.pid_controller.compute(altitude_error, delta_time)

#         # Here, we interpret aoa_adjustment directly as the change needed in AoA.
#         # In a real application, this might need to be scaled or transformed.
#         new_aoa = self.aoa_sensor.get_angle_of_attack() + aoa_adjustment

#         # Ensure AoA stays within realistic bounds to avoid stall or overspeed
#         new_aoa = max(min(new_aoa, MAX_AOA), MIN_AOA)

#         self.aoa_sensor.set_angle_of_attack(new_aoa)

#         # Adjusting the AoA will influence CL and thus the lift, indirectly affecting altitude.
#         # The actual change in lift and altitude would be computed by the aircraft's dynamics model,
#         # which is not detailed here.

#         return new_aoa  # This could be used to adjust the aircraft's control surfaces.
