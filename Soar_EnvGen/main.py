import os 

import argparse
import numpy as np

from cfd import CFDSimulation
from wind_field import WindField


"""
The main class should result in a windfield folder for the simulation enviroment, nothing more is needed. 
This npy file will contain all the points and its respective wind vectors. These points will than have to be maped to 
the simulation enviroment inside isaac sim. This repositority does nothing more. 

In abstract terms:
 - Going in the enviroment you will be used inside the simulation
 - Going out is the windfield folder which contains the flow field for every time step
"""


# TODO Create main that runs 10 different scenearios at 10 different orientations and wind speeds
class Main: 
    def __init__(self,file:str):
        self.cfd = CFDSimulation(file=file)
        self.wind_field = WindField()

    def run_scenarios(self, wind_speed_start:float, wind_speed_stop: float,wind_speed_step:float):
        if isinstance(wind_speed_start, (np.float32, np.float64)):
                wind_speed = wind_speed.item()  # Converts NumPy float to Python float

        wind_speeds = np.arange(wind_speed_start, wind_speed_stop, wind_speed_step)
        for wind_speed in wind_speeds:
            if isinstance(wind_speed, (np.float32, np.float64)):
                wind_speed = wind_speed.item()  # Converts NumPy float to Python float

            # TODO add for loop for env orientations
            self.cfd.set_wind_velocity(wind_speed)
            self.cfd.run()   
            cfd_current_run_config = os.path.join(self.cfd.get_cfd_current_run_directory(), "config.yaml")
            self.wind_field.set_current_run_config(cfd_current_run_config)
            self.wind_field.set_wind_velocity_label(wind_speed)
            self.wind_field.read_windfield_data()
            
# if __name__ == "__main__":
#     main = Main(file="random_field_flow_test.obj")
#     main.run_scenarios(1.0, 10.0, 1.0)


def run_from_cli():
    # Use argparse to handle command-line arguments
    parser = argparse.ArgumentParser(description='Create a random field of buildings and export in USD and OBJ formats.')
    parser.add_argument('--file', type=str, default = "random_field_flow_test.obj", help='Enivorment file to run the simulation') #TU_Delft_campus.obj
    parser.add_argument('wind_speed_start', type=float, help='Start wind speed')
    parser.add_argument('wind_speed_stop', type=float, help='Stop wind speed')
    parser.add_argument('--wind_speed_step', type=float, default=1.0,help='Step value for wind speed')

    args = parser.parse_args()


    main = Main(file=args.file)
    # Run the main functionality
    main.run_scenarios(float(args.wind_speed_start), float(args.wind_speed_stop), float(args.wind_speed_step))


if __name__ == "__main__":
    run_from_cli()
