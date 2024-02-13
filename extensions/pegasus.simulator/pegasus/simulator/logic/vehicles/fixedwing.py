"""
| File: fixedwing.py
| Author: Kjell Vleescchouwer
| Based on the 'multirotor.py' file of Marcel Jacinto. 
| Description: Definition of the Fixedwing class which is used as the base for all the fixed-wing plane uav's vehicles.
"""

import numpy as np

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle_new import Vehicle
from scipy.spatial.transform import Rotation
# Mavlink interface
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag, Lift, Drag, Thrust
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS, Airspeed, HeadingIndicator
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Control
from pegasus.simulator.logic.backends.controller import HeadingHoldMode , AltitudeHoldMode
from pegasus.simulator.logic.backends.controller.states import AngleOfAttack

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
        self.drag = Drag([0.05])
        self.lift = Lift()
        self.thrust = Thrust()

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS(), HeadingIndicator()] #, Airspeed()]

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        self.backends = [MavlinkBackend()]
        self.controls = [HeadingHoldMode(), AltitudeHoldMode()]
        # self.controls = [HeadingHoldMode()]
        self.states = [AngleOfAttack()]



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

        # 2. Initialize all the vehicle sensors
        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.initialize(PegasusInterface().latitude, PegasusInterface().longitude, PegasusInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # 3. Setup the dynamics of the system
        # Get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._drag = config.drag
        self._lift = config.lift
        self._thrust = config.thrust
        
        #setup the controls of the systems
        self._hhm = config.controls[0]
        self._ahm = config.controls[1]

        #setup the states of the system 
        self.aoa = config.states[0]

        # 4. Save the backend interface (if given in the configuration of the multirotor)
        # and initialize them
        self._backends = config.backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callbacks for the 
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)

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
        self.aoa.reset() # Has to be done as this is self made and not part of the isaac sim. 

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

        aoa = self.aoa.get_aoa()
        # import control before updating the forces... Otherwise they get update with wrong values.
        altitude = self.state.position[2]
        zdot = self.state.linear_body_velocity[2]
        new_aoa = self._ahm.update(altitude, dt, aoa,zdot)
        self.aoa.set_angle_of_attack(new_aoa)
        aoa = self.aoa.get_aoa()

        # Sensor stuff 
        self.update_sensors(dt)
        # print(self._sensors)
        observations = self._sensors
        # print("HI: ", observations[4].state["Heading:"])

        HI_ref = 90
        HI_m = observations[4].state["Heading:"]

        yaw_moment = self._hhm.update(HI_ref, HI_m)
        # print("Yaw MOment: ", yaw_moment)

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._drag.update(self._state, dt)
        lift = self._lift.update(self._state, dt, aoa)
        thrust = self._thrust.update(self._state, dt)

        # self.apply_force(drag, body_part="/base_link")   #drag is a 1x3 array
        # self.apply_force([10,0,0], body_part="/base_link") # [x,y,z]

  
        # self.apply_force([10,0,0], body_part="/fuselage_link") # [x,y,z] # some kind of thrust 
        self.apply_force(drag, body_part="/body")   #drag is a 1x3 array
        # self.apply_force([20,0,0], body_part="/body") # [x,y,z] # some kind of thrust 
        self.apply_force(thrust, body_part="/body") # [x,y,z] #hrust 

        # Apply Lift. TODO: fix distribution as well as small altitude controller
        self.apply_force(lift,body_part="/body")

        # self.apply_force([0,0,20.0],body_part="/body")

        # self.apply_torque([0.0,0.0,0.005], body_part="/body")
        # self.apply_torque([0.0,0.0,yaw_moment], body_part="/body")

        position = self.state.position
        lin_vel = self.state.linear_velocity
        lin_body_vel = self.state.linear_body_velocity
        attitude = self.state.attitude
        
        euler_att = self.state.attitude_eul
        
        print("THrust:: ", thrust)
        print("Forward velocity: ", lin_body_vel[0])
        print("Upward Velocity: ", lin_body_vel[2] )
        print("Drag, " , drag)
        print("Lift: ", lift)
        
        print("altitude: ", position[2])
        # print(dt)
        # print("Position", position)
        # # print("lin velL ", lin_vel)
        # print("lin_body_vel", lin_body_vel)
   
        # print("attitude ", attitude)
        # print("attitude euler: ", euler_att)


        # print("Lift:", lift[2])
        # print("Thrust:", thrust[0])
        # self.apply_force(lift,body_part="/fuselage_link")
        # self.apply_force(0.35*np.array(lift), body_part="/horizontal_tail_link")
        # self.apply_force([0,0,50], body_part="/base_link")
        # Call the update methods in all backends



        # Implement Heading Hold Mode. Heading Indicator needed. 
    # def handle_propeller_visual(self, rotor_number, force: float, articulation):
    #     """
    #     Auxiliar method used to set the joint velocity of each rotor (for animation purposes) based on the 
    #     amount of force being applied on each joint

    #     Args:
    #         rotor_number (int): The number of the rotor to generate the rotation animation
    #         force (float): The force that is being applied on that rotor
    #         articulation (_type_): The articulation group the joints of the rotors belong to
    #     """

    #     # Rotate the joint to yield the visual of a rotor spinning (for animation purposes only)
    #     joint = self._world.dc_interface.find_articulation_dof(articulation, "joint" + str(rotor_number))

    #     # Spinning when armed but not applying force
    #     if 0.0 < force < 0.1:
    #         self._world.dc_interface.set_dof_velocity(joint, 5 * self._thrusters.rot_dir[rotor_number])
    #     # Spinning when armed and applying force
    #     elif 0.1 <= force:
    #         self._world.dc_interface.set_dof_velocity(joint, 100 * self._thrusters.rot_dir[rotor_number])
    #     # Not spinning
    #     else:
    #         self._world.dc_interface.set_dof_velocity(joint, 0)

    # def force_and_torques_to_velocities(self, force: float, torque: np.ndarray):
    #     """
    #     Auxiliar method used to get the target angular velocities for each rotor, given the total desired thrust [N] and
    #     torque [Nm] to be applied in the multirotor's body frame.

    #     Note: This method assumes a quadratic thrust curve. This method will be improved in a future update,
    #     and a general thrust allocation scheme will be adopted. For now, it is made to work with multirotors directly.

    #     Args:
    #         force (np.ndarray): A vector of the force to be applied in the body frame of the vehicle [N]
    #         torque (np.ndarray): A vector of the torque to be applied in the body frame of the vehicle [Nm]

    #     Returns:
    #         list: A list of angular velocities [rad/s] to apply in reach rotor to accomplish suchs forces and torques
    #     """

    #     # Get the body frame of the vehicle
    #     rb = self._world.dc_interface.get_rigid_body(self._stage_prefix + "/body")

    #     # Get the rotors of the vehicle
    #     rotors = [self._world.dc_interface.get_rigid_body(self._stage_prefix + "/rotor" + str(i)) for i in range(self._thrusters._num_rotors)]

    #     # Get the relative position of the rotors with respect to the body frame of the vehicle (ignoring the orientation for now)
    #     relative_poses = self._world.dc_interface.get_relative_body_poses(rb, rotors)

    #     # Define the alocation matrix
    #     aloc_matrix = np.zeros((4, self._thrusters._num_rotors))
        
    #     # Define the first line of the matrix (T [N])
    #     aloc_matrix[0, :] = np.array(self._thrusters._rotor_constant)                                           

    #     # Define the second and third lines of the matrix (\tau_x [Nm] and \tau_y [Nm])
    #     aloc_matrix[1, :] = np.array([relative_poses[i].p[1] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)])
    #     aloc_matrix[2, :] = np.array([-relative_poses[i].p[0] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)])

    #     # Define the forth line of the matrix (\tau_z [Nm])
    #     aloc_matrix[3, :] = np.array([self._thrusters._rolling_moment_coefficient[i] * self._thrusters._rot_dir[i] for i in range(self._thrusters._num_rotors)])

    #     # Compute the inverse allocation matrix, so that we can get the angular velocities (squared) from the total thrust and torques
    #     aloc_inv = np.linalg.pinv(aloc_matrix)

    #     # Compute the target angular velocities (squared)
    #     squared_ang_vel = aloc_inv @ np.array([force, torque[0], torque[1], torque[2]])

    #     # Making sure that there is no negative value on the target squared angular velocities
    #     squared_ang_vel[squared_ang_vel < 0] = 0.0

    #     # ------------------------------------------------------------------------------------------------
    #     # Saturate the inputs while preserving their relation to each other, by performing a normalization
    #     # ------------------------------------------------------------------------------------------------
    #     max_thrust_vel_squared = np.power(self._thrusters.max_rotor_velocity[0], 2)
    #     max_val = np.max(squared_ang_vel)

    #     if max_val >= max_thrust_vel_squared:
    #         normalize = np.maximum(max_val / max_thrust_vel_squared, 1.0)

    #         squared_ang_vel = squared_ang_vel / normalize

    #     # Compute the angular velocities for each rotor in [rad/s]
    #     ang_vel = np.sqrt(squared_ang_vel)

    #     return ang_vel


"""
Create a function that calls the states, sensors etc... 

Sensors to be created: 
wind velocities
Angle of attack sensor --> alpha

create function which makes sensor values callable.

maybe just state?


        stateoverview = self.state # Objet class pegasus.simulator.logic.state.State
        self.state.attitude
        self.state.position
        self.state.line

-->> Sensors are not inside the State class.
        self._sensors #
        self._sensors[0].state --> class 'dict'

"""