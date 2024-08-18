import pytest
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.state import State


# @pytest.fixture
# def aerodynamics():
#     return Aerodynamics() 


# @pytest.mark.parametrize(
#     "groundspeed, wind_vector, euler_angle, wind_angle, ,expected",
#     [
#         ([10, 0, 0],[0,0,0], 0, 0, 13.0),
#     ]
# )     

# #wind angle is the same as the attitude angle 0 E=90, W=-90 , 

# # Test the caculate_airspeed method of the Aerodynamics class
# def test__caculate_airspeed_euler_angles(groundspeed, euler_angle, state, expected, aerodynamics):
#     aerodynamics.wind_velocity = 3.0
#     aerodynamics._windangle = 
#     assert aerodynamics.caculate_airspeed(groundspeed, euler_angle, state) == expected

# # TODO Validation:
#     # Check for the wind coming out of south, for 90 deg delta etc.  --> Checked
#     # Check for the wind coming from the east for 60 deg heading, so NE kinda. --> Checked. (Headwind)
#     # Check for the wind coming from the east for 240 deg heading, thus -120 so kindo f SW. --> Checked
#     # Check for the wind coming form the eas for 300 deg heading, thus -60, so kinda NW --> Checked
#     # Check for Western wind --> Checked
#     # Check for Southern wind --> Checked

