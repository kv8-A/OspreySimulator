"""
| File: fixedwing.py
| Author: Kjell Vleescchouwer
| Based on the 'multirotor.py' file of Marcel Jacinto. 
| Description: Definition of the Fixedwing class which is used as the base for all the fixed-wing plane uav's vehicles.
"""

import numpy as np
from pegasus.simulator.validation import SimulationDataLogger

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle_new import Vehicle
from scipy.spatial.transform import Rotation
# Mavlink interface
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Sensors and dynamics setup
from pegasus.simulator.logic.vehicles.state import FixedWingState
from pegasus.simulator.logic.dynamics import LinearDrag, Lift, Drag, Thrust
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS, Airspeed, HeadingIndicator, FlightPathAngle, PitchAngle
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Control
from pegasus.simulator.logic.backends.controller import HeadingHoldMode , AltitudeHoldMode, VelocityHoldMode
from pegasus.simulator.logic.backends.controller.states import AngleOfAttack
from pegasus.simulator.logic.backends.controller.control_devices import Throttle

class FixedwingConfig:
    """
    A data class that is used for configuring a Multirotor
    """

    def __init__(self):
        """
        Initialization of the MultirotorConfig class
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "fixedwing"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = ""

        # The default thrust curve for a quadrotor and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurve()
        self.drag = Drag()
        self.lift = Lift()
        self.thrust = Thrust()

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS(), HeadingIndicator(), FlightPathAngle(), PitchAngle()] #, Airspeed()]

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        self.backends = [MavlinkBackend()]
        self.controls = [HeadingHoldMode(), AltitudeHoldMode(), VelocityHoldMode()]
        # self.controls = [HeadingHoldMode()]
        self.states = [AngleOfAttack(), Throttle()]



class Fixedwing(Vehicle):
    """Fixedwing class - It defines a base interface for creating a multirotor
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "fixedwing",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=FixedwingConfig(),
        datalogger= False,
    ):
        """Initializes the fixedwing object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            config (_type_, optional): _description_. Defaults to MultirotorConfig().
        """

        # 1. Initiate the Vehicle object itself
        super().__init__(stage_prefix, usd_file, init_pos, init_orientation)

        self._state = FixedWingState(config.states[0], config.states[1])

        # self._world.add_physics_callback(self._stage_prefix + "/state", self.update_fixedwing_state)

        # 2. Initialize all the vehicle sensors
        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.initialize(PegasusInterface().latitude, PegasusInterface().longitude, PegasusInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # Initialize fixed wing states

        # 3. Setup the dynamics of the system
        # Get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._drag = config.drag
        self._lift = config.lift
        self._thrust = config.thrust
        
        #setup the controls of the systems
        self._hhm = config.controls[0]      # Heading Hold Mode
        self._ahm = config.controls[1]      # Altitude Hold Mode
        self._vhm = config.controls[2]      # Velocity Hold Mode

        # Ref altitude and velocoity
        self._ahm.ref_altitude = 35
        self._vhm.ref_v = 7
        # TODO setting position and being able to change it during flight

        #setup the states of the system 
        # self.aoa = config.states[0]
        # self.throttle = config.states[1]

        # 4. Save the backend interface (if given in the configuration of the multirotor)
        # and initialize them
        self._backends = config.backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callbacks for the 
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)

        # Add the simulation datalogger class
        self.logger = SimulationDataLogger() if datalogger else None
        self.step = 0.0 # --> delete later
        self.time = 0.0 # --> deleta later

    def update_sensors(self, dt: float):
        """Callback that is called at every physics steps and will call the sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_sensor method will also be called for
        every backend. For example, if new data is generated for an IMU and we have a MavlinkBackend, then the update_sensor
        method will be called for that backend so that this data can latter be sent thorugh mavlink.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        for backend in self._backends:
            backend.update_state(self._state)

    def update_fixedwing_state(self, dt:float):
        self._state.update_state(dt)
        # Now update the angle of attack and the flight path angle so that they are included into the state of the vehicle.

    def start(self):
        """
        Intializes the communication with all the backends. This method is invoked automatically when the simulation starts
        """
        for backend in self._backends:
            backend.start()

    def stop(self):
        """
        Signal all the backends that the simulation has stoped. This method is invoked automatically when the simulation stops
        """
        for backend in self._backends:
            backend.stop()

        # This resets the angle of attack when the simulation has been reset in the UI
        self._state.angle_of_attack.reset() # Has to be done as this is self made and not part of the isaac sim. 
        self._state.throttle.reset()
        self.time = 0.0
        self.step = 0.0
        if self.logger is not None:
            filepath = "extensions/pegasus.simulator/pegasus/simulator/validation/controller_val"
            self.logger.save_to_csv(filepath+"/simulation_data.csv")
    
    def update_control(self, dt:float):
        # aoa = self.aoa.get_aoa()
        aoa = self._state.angle_of_attack.get_aoa()
        throttle = self._state.throttle.get_throttle()
        # throttle = self.throttle.get_throttle()
        # import control before updating the forces... Otherwise they get update with wrong values.
        altitude = self.state.position[2]
        fwd_acc = self.state.linear_acceleration[0]
        velocity = self.state.linear_body_velocity[0]
        # print("altitude: ", altitude)
        
        # Altitude Hold Mode
        zdot = self.state.linear_body_velocity[2]
        new_aoa = self._ahm.update(altitude, dt, aoa,zdot)
        self._state.angle_of_attack.set_angle_of_attack(new_aoa)
        # self.aoa.set_angle_of_attack(new_aoa)
        # aoa = self.aoa.get_aoa()
        aoa = self._state.angle_of_attack.get_aoa()
        cl = self._lift.get_cl(aoa)

        # Velocity Hold Mode
        throttle_cmd = self._vhm.update(dt, velocity,fwd_acc)
        # throttle = self.throttle.set_throttle(throttle_cmd)
        self._state.throttle.set_throttle(throttle_cmd)
        # throttle = self.throttle.get_throttle()
        

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in simulation based on the motor speed. 
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the articulation root of the vehicle
        articulation = self._world.dc_interface.get_articulation(self._stage_prefix)
        
        
        for backend in self._backends:
            backend.update(dt)

        self.update_control(dt)
        # TODO implement adjustment from wind velocity into angle of attack
        aoa = self._state.angle_of_attack.get_aoa()
        throttle = self._state.throttle.get_throttle()  
        cl = self._lift.get_cl(aoa)

        # aoa = self.aoa.get_aoa()
        # throttle = self.throttle.get_throttle()

        # # import control before updating the forces... Otherwise they get update with wrong values.
        altitude = self.state.position[2]

        # # Velocity Hold Mode
        # throttle_cmd = self._vhm.update(dt, velocity,fwd_acc)
        # throttle = self.throttle.set_throttle(throttle_cmd)
        # throttle = self.throttle.get_throttle()
        # print("Throttle: ", throttle)
        # # Sensor stuff 
        self.update_sensors(dt)  # Important that this is done after the control update.
        # print(self._sensors)
        print("Angle of attack", aoa)
        print("cl: ", cl)
        
        # print("Pitch Angle", self._sensors[6].state["pitch_angle"])
        # print("FlightpathAngle", self._sensors[5].state["flight_path_angle"] )
        observations = self._sensors
        # print("HI: ", observations[4].state["Heading:"])

        HI_ref = 90
        HI_m = observations[4].state["Heading:"]

        yaw_moment = self._hhm.update(HI_ref, HI_m)
        # print("Yaw MOment: ", yaw_moment)

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._drag.update(self._state, dt, cl)
        lift = self._lift.update(self._state, dt, aoa)
        thrust = self._thrust.update(self._state, dt, throttle)

        print("drag: ", drag)
        # Following two lines as if you throw the drone to start it up
        # if self.step < 8:
        #     self.apply_force([500,0,0])
        
        # self.apply_force([10,0,0], body_part="/fuselage_link") # [x,y,z] # some kind of thrust 
        self.apply_force(drag, body_part="/body")   #drag is a 1x3 array
        # self.apply_force([20,0,0], body_part="/body") # [x,y,z] # some kind of thrust 
        self.apply_force(thrust, body_part="/body") # [x,y,z] #hrust 
        # self.apply_force([20,0,0], body_part="/body") # [x,y,z] #hrust 

        # Apply Lift. TODO: fix distribution as well as small altitude controller
        self.apply_force(lift,body_part="/body")


        position = self.state.position
        lin_vel = self.state.linear_velocity
        lin_body_vel = self.state.linear_body_velocity
        attitude = self.state.attitude
        
        euler_att = self.state.attitude_eul
        
        # print("Linear body velocity: ", lin_body_vel)   
        print("THrust:: ", thrust)
        print("Forward velocity: ", lin_body_vel[0])
        # print("Upward Velocity: ", lin_body_vel[2] )
        # print("Drag, " , drag)
        print("Lift: ", lift)
        
        print("altitude: ", position[2])

        # print("acceleration: ",self.state.linear_acceleration)
        # print(dt)
        # Call the update methods in all backends
        # print("Position", position)
        # # print("lin velL ", lin_vel)
        # print("lin_body_vel", lin_body_vel)
   
        # print("attitude ", attitude)
        print("Heading:", observations[4].state["Heading:"])
        # print("attitude euler: ", euler_att)
        time = self._world._timeline.get_current_time()
        # print("Time: ",time)

        self.time += dt
        # print("Time 2: ", self.time)
        
        self.step += 1


        # # Implement the datalogger class 
        if self.logger is not None: 
            current_time = self.time
            self.logger.log(self.time,altitude)


        # self.time += dt
        # print("Time 2: ", self.time)
        # Implement Heading Hold Mode. Heading Indicator needed. 
  