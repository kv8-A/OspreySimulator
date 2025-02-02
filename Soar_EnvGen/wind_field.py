import os
import json
import shutil
import numpy as np

import glob
import time
from datetime import datetime
import yaml
import re 
import subprocess
from pathlib import Path 


from numpy.typing import NDArray
 
from dataclasses import dataclass 


@dataclass
class FlowField:   
    points: np.ndarray[float]
    wind_vectors: np.ndarray[float]

@dataclass
class FlowFieldData:
    time: float
    flow_field: FlowField



class WindField: 
        

    def __init__(self, config = None):
        # Get wind field dir, and make one if its not there
        
        self._home_dir = Path.home()
        self._directory_path = os.path.dirname(os.path.abspath(__file__))
        self._windfield_dir = os.path.join(self._directory_path, "wind_fields")
        self._directories_run = {}

        self.current_run_settings: dict
        self.current_run_path: str
        self.time_interval_end: list[int, int]
        self.wind_velocity_label: float

    def set_current_run_config(self, config_dir_current_run: str):
        self.current_run_path = config_dir_current_run.replace("/config.yaml", "")
        with open(config_dir_current_run, 'r') as file:
            self.current_run_settings = yaml.safe_load(file)

    def _get_current_run_config(self):
        return self.current_run_settings
    
    def set_wind_velocity_label(self, wind_velocity: float):
        self.wind_velocity_label = wind_velocity
    
    def get_wind_velocity_label(self):
        self.wind_velocity_label = self.wind_velocity_label
    

    def get_dir_windfield(self, cfd_run: str) -> str:
        """
        Gets the directory containing the the wind fields and creates a new one if dir does not 
        exist.
        """
        # get cfd for current run or other run to analyse
        cfd_dir = os.path.dirname(cfd_run)
        
        # Create wind field directory path
        wind_field_dir = os.path.join(cfd_dir, "wind_fields")
        
        # Create wind field directory if it doesn't exist
        if not os.path.exists(wind_field_dir):

            os.makedirs(wind_field_dir)
        
        self._directories_run["cfd_current_run"] = cfd_run
        self._windfield_dir = wind_field_dir


        
        # Get the name of the run folder
        run_x = cfd_run.split("/")[-1]


        
        # Match map in windfield map. If it does not exist make it

        wind_field_dir_run = os.path.join(self._windfield_dir, run_x)
        
        # if path exist get dir


        # if wind_field_dir_run 
        # if doesnt exist shutil copy file to wind field dir
        # store wind field dir in 
        self._directories_run["wind_field_dir_run"] = wind_field_dir_run
        

    def _get_end_time_and_interval(self):
        config = self.current_run_settings
    
        end_time = config['solving']['endTime']
        write_interval = config['solving']['writeInterval']
        self.time_interval_end = [write_interval, end_time]
        
        return end_time, write_interval

    def read_windfield_data(self): # TODO, include time step 
        
        
        if not os.path.exists(self.current_run_path):
            raise FileNotFoundError(f"Path {self.current_run_path} does not exist")

        end_time, write_interval = self._get_end_time_and_interval()

        # TODO create a function that reads every timestep, not just the END time like now. for now its fine
        path = "cfd/cfd_runs/run_20240609_224721_buildings/50/U"
        path_velocity = f"{self.current_run_path}/{end_time}/U"
        path_cellCentres = f"{self.current_run_path}/{end_time}/C"
        # path = "cfd/cfd_runs/run_20240316_231219_buildings/postProcessing/ensightWrite/data/00000380/U"
        # path = "cfd/cfd_runs/run_20240316_231219_buildings/postProcessing/sets/streamLines/270/track0.vtp"

        # with open(path) as file:
        #     data = file.readlines()

        # Read velocity vectors
        with open(path_velocity, 'r') as file:
            lines = file.readlines()
        
        # Flag to start reading vector data
        start_reading = False

        velocity_vectors = []

        for line in lines:
            if start_reading:
                line = line.strip()
                if line == ")" or line ==");":
                    break
                if line.startswith("(") and line.endswith(")"):
                    # remove parenthesis and split by space
                    stripped_line = line.strip("()").split()
                    vector_tuple = tuple(map(float, stripped_line))
                    velocity_vectors.append(vector_tuple)

            if "internalField   nonuniform List<vector>" in line:
                start_reading = True  # Start reading from the next line
        
        velocity_vectors_array = np.array(velocity_vectors)

        # Read cell positions
        with open(path_cellCentres, 'r') as file:
            lines = file.readlines()

        # Flag to start reading vector data
        start_reading = False

        cell_positions = []

        for line in lines:
            if start_reading:
                line = line.strip()
                if line == ")" or line ==");":
                    break
                if line.startswith("(") and line.endswith(")"):
                    # remove parenthesis and split by space
                    stripped_line = line.strip("()").split()
                    vector_tuple = tuple(map(float, stripped_line))
                    cell_positions.append(vector_tuple)

            if "internalField   nonuniform List<vector>" in line:
                start_reading = True  # Start reading from the next line
            
        cell_positions_array = np.array(cell_positions)
        
        velocity_name1 = str(self.wind_velocity_label).split('.')[0]
        velocity_name2 = str(self.wind_velocity_label).split('.')[1]

        # TODO change end_time to time step and put into for loop
        wind_field_path = os.path.join(self._windfield_dir, self.current_run_path.split("/")[-1])
        if not os.path.exists(wind_field_path):
            os.makedirs(wind_field_path)
        npz_file_path = os.path.join(wind_field_path, f'flow_field_data_{end_time}_v_{velocity_name1}_{velocity_name2}.npz')
        np.savez(npz_file_path, velocity_vectors=velocity_vectors_array, cell_positions=cell_positions_array)

        return FlowFieldData(time=float(end_time), 
                             flow_field=FlowField(points=cell_positions_array, wind_vectors=velocity_vectors_array))

        

        # convert every timestep to npy file of points and wind vectors
        # save the current run folder within the windfield folder/ TODO aftwards make per enviroment



# wf = WindField()
# # # wf._set_current_run_config("cfd/cfd_runs/run_20240609_224721_buildings/config.yaml")
# # # wf._set_current_run_config("cfd/cfd_runs/run_20240908_164926_TU_Delft_campus/config.yaml")
# # # wf._set_current_run_config("cfd/cfd_runs/run_20240908_164528_random_field/config.yaml")
# # # wf._set_current_run_config("cfd/cfd_runs/run_20241012_223409_random_field_flow_test/config.yaml")
# # wf.set_current_run_config("cfd/cfd_runs/run_20241021_011838_random_field_flow_test_10_0/config.yaml")
# wf.set_current_run_config("/home/kjell/Documents/Repositories/Soar_EnvGen/cfd/cfd_runs/run_20241021_011838_random_field_flow_test_10_0/config.yaml")
# # # print(wf._get_current_run_config())

# # # wf.read_u_file()
# wf.set_wind_velocity_label(10.0)
# wf.read_windfield_data()

