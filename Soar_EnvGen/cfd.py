"""
This file will contain the cfd pipeline. Here the environment is meshed. The CFD parameters are set and the CFD file will be run.
"""

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

from utils import Utils
from settings import ConfigManager

SNAPPY_MARGIN_OF_MARGIN = 0.75

class CfdConfig(ConfigManager):

    def __init__(self,new_config=None):
        super().__init__()

        if new_config is not None:
                # sett config 
                pass 

        self.cfd_config = self.config["cfd"]
        self.directories = self.config["directories"]
        self.software_settings = self.config["software"]


# cfdc = CfdConfig()
# print(cfdc.cfd_config)

class CFDSimulation:
    # TODO Change file from default to non-default parameter
    def __init__(self,file: str ="buildings.obj", wind_vel: float=15.0, param:dict = None, config: CfdConfig = CfdConfig()):
        self.config = config
        self.file = file
        self.home_dir = config.home_dir
        self.directory_path = config.directory_path
        self.current_run_settings = {}
        self._utils = Utils()

        self.wind_vel = wind_vel

    def get_env(self):
        pass

    def _get_mesh_settings(self, settings_yaml:str = None): # new_parameters.yaml
        # set settings. Also save every config yaml file in every run folder
        # doing this makes it clear with what run which settings were used
        if settings_yaml is None: 
            # settings_yaml = self.config.directories["config_dir"].format(directory_path=self.directory_path)
            pass
        else:
            settings_yaml = self.config.directories["config_dir"].format(directory_path=self.directory_path)+settings_yaml
            new_config = self.config.load_config(settings_yaml)
            # self.config.config = new_config  # Set new config total to later have the config.yaml file of the run. 
            new_cfd_config = new_config["cfd"]
        
            self.config.cfd_config = new_cfd_config

    def _set_mesh_settings(self, margin_from_object:float=1.5):
        # self.config.cfd_config["0dir"]["Velocity"] = self.wind_vel
        self.config.cfd_config["0directory"] = {"inlet_velR" : self.wind_vel}

        # mesh_settings = self.config.cfd_config["mesh"]
        # solver_settings = self.config.cfd_config["solving"]

        # blockMesh_settings = mesh_settings["blockMeshDict"]
        # snappyHexMesh_settigs = mesh_settings["snappyHexMeshDict"]

        #TODO set settings for BlockMeshDict size form obj size
        
        x_coor, y_coor, z_coor = self._utils.get_obj_extreme_coordinates(self.file, f'{self.config.directories["current_cfd_run"]}/constant')
        self.config.cfd_config["mesh"]["blockMeshDict"]["xMin"] = x_coor["min"] * margin_from_object
        self.config.cfd_config["mesh"]["blockMeshDict"]["xMax"] = x_coor["max"] * margin_from_object
        self.config.cfd_config["mesh"]["blockMeshDict"]["yMin"] = y_coor["min"] * margin_from_object
        self.config.cfd_config["mesh"]["blockMeshDict"]["yMax"] = y_coor["max"] * margin_from_object
        self.config.cfd_config["mesh"]["blockMeshDict"]["zMin"] = z_coor["min"] * margin_from_object
        self.config.cfd_config["mesh"]["blockMeshDict"]["zMax"] = z_coor["max"] * margin_from_object

        # TODO set over total length 
        self.config.cfd_config["mesh"]["blockMeshDict"]["xCells"] = int((np.abs(x_coor["max"])+np.abs(x_coor["min"]))/6.0)
        self.config.cfd_config["mesh"]["blockMeshDict"]["yCells"] = int((np.abs(y_coor["max"])+np.abs(y_coor["min"]))/6.0)
        self.config.cfd_config["mesh"]["blockMeshDict"]["zCells"] =    int((np.abs(z_coor["max"])+np.abs(z_coor["min"]))/6.0)
        # TODO set over total length 
        # self.config.cfd_config["mesh"]["blockMeshDict"]["xCells"] = 150
        # self.config.cfd_config["mesh"]["blockMeshDict"]["yCells"] = 150
        # self.config.cfd_config["mesh"]["blockMeshDict"]["zCells"] = 15

        # TODO set xCells and yCells and zCells (is number of cells and should scale with the size of the object)
        snappy_hex_margin = SNAPPY_MARGIN_OF_MARGIN * margin_from_object
        min_box = [x_coor["min"] * snappy_hex_margin, y_coor["min"] * snappy_hex_margin, z_coor["min"] * snappy_hex_margin]
        max_box = [x_coor["max"] * snappy_hex_margin, y_coor["max"] * snappy_hex_margin, z_coor["max"] * snappy_hex_margin]

        self.config.cfd_config["mesh"]["snappyHexMeshDict"]["refinementBox"]["min"] = min_box
        self.config.cfd_config["mesh"]["snappyHexMeshDict"]["refinementBox"]["max"] = max_box

        # # Save the current configuration in the run directory
        config_file_path = os.path.join(self.config.directories["current_cfd_run"], "config.yaml")
        with open(config_file_path, 'w') as file:
            yaml.dump(self.config.cfd_config, file, default_flow_style=False)
    
    def set_wind_velocity(self, wind_vel:float):
        self.wind_vel = wind_vel

    def get_cfd_current_run_directory(self):
        return self.config.directories["current_cfd_run"]

    def _new_cfd_case_dir(self):
        """
        This method runs a new cfd case. It creates a new folder in the cfd directory. 
        As well as copies the object file to the the right directory for OpenFOAM to read.

        This also uses the wind velocity self variable to name the file. Be sure to reuse function to set it.
        """
 
        # Get cfd directory. Create new one for new run
        cfd_dir = self.config.directories["cfd_dir"].format(directory_path=self.directory_path)

        # get time and date
        now = datetime.now()
        date_time_str = now.strftime("%Y%m%d_%H%M%S")
        env_name = str(self.file).split(".")[0]
        velocity_name1 = str(self.wind_vel).split('.')[0]
        velocity_name2 = str(self.wind_vel).split('.')[1]

        # velocity = self.cfd.settings... # TODO set velocity parameter. 
        # TODO make new map per environment that will be run .. thus buildings, maasvlakte, campus ... 

        new_dir_name = f"cfd_runs/run_{date_time_str}_{env_name}_{velocity_name1}_{velocity_name2}" # TODO add velocity
        new_cfd_folder = os.path.join(cfd_dir,new_dir_name)

        if not os.path.exists(new_cfd_folder):
            os.makedirs(new_cfd_folder)

        # copy cfd default , then call the copied folder run_.._buildins_date
        default_dir = cfd_dir+"cfd_default"
        # Copy files of cfd defualt in cfd_run_date_envName_velocity
        for item in os.listdir(default_dir):
            s = os.path.join(default_dir, item)
            d = os.path.join(new_cfd_folder, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, False, None)
            else:
                shutil.copy2(s, d)

        # store new path in self
        self.config.directories["current_cfd_run"] = new_cfd_folder

        # Get the geometry file into the /constant/triSurface map
        os.makedirs(new_cfd_folder+"/constant/triSurface")
        # files directory
        #TODO set path to geometry files in a better way / differnet name
        geometry_files_dir = self.directory_path + "/blender_envs/blend_files/"
        geometry_files = glob.glob(geometry_files_dir+"*.obj")
        if os.path.join(geometry_files_dir,self.file) in geometry_files:
            shutil.copy2(geometry_files_dir+"/"+self.file, new_cfd_folder+"/constant")
            shutil.copy2(geometry_files_dir+"/"+self.file, new_cfd_folder+"/constant/triSurface")
        else:
            print("Geometry file not found")
        


    def apply_settings(self):
        # after preparing the settings and creating the new folder. Apply them.  
        # this means setting the U file etc in the directory. 

        # Apply all settings to 

        #directory cfd run
        # dir 

        """
        Cfd files to change are:
            -0.orig/U --? Uinlet (10,0,0)
            - BlockMeshDict --> 
            - SnappyHexMeshDict
            - ControlDict ??? Time etc
               
        """
        
        folder_edits = ["0.orig", "system"]
        file_edits = ["U", "controlDict", "blockMeshDict", "snappyHexMeshDict", "surfaceFeatureExtractDict"]
        file_paths = []
        for file in file_edits:
            search = os.path.join(self.config.directories["current_cfd_run"],"**",file)
            found_file = glob.glob(search,recursive=True)
            file_paths.extend(found_file)
        
        # print(file_paths)


        # 0. Apply U orig settings
        with open(file_paths[0], 'r') as file: 
            data = file.readlines()
        
        edited_data= []
        for line in data:
            if line.strip().startswith('Uinlet'):
                edited_data.append(f"Uinlet     ({self.wind_vel} 0 0);\n")
            else:
                edited_data.append(line)
        
        with open(file_paths[0],'w') as file:
            file.writelines(edited_data)

        

        # 1. Edit and apply control dict settings
        with open(file_paths[1], 'r') as file:
            data = file.readlines()
        
        edited_data = []
        
        for line in data:
            for key,value in self.config.cfd_config['solving'].items():
                if line.strip().startswith(key):
                    # Dynamically create the patern with the key
                    pattern = rf'({key}\s+)(\S+)(;)?'
                    replacement = lambda match: f"{match.group(1)}{value}{match.group(3) if match.group(3) else ';'}"
                    line = re.sub(pattern, replacement, line)

                    # edited_data.append(f"{key}  {value};\n")
                    line_modified = True
                    break # break the loop, other keys do not have to be evaluated once key has been matched

            edited_data.append(line)


        with open(file_paths[1],'w') as file:
            file.writelines(edited_data)

        # 2. Edit and apply Block Mesh dict settings
        # TODO Get xmin xmax calculation from blender dimensions...
       
        with open(file_paths[2], 'r') as file:
            data = file.readlines()
        
        edited_data = []
        
        for line in data:
            for key,value in self.config.cfd_config['mesh']['blockMeshDict'].items():
                if line.strip().startswith(key):
                    # Dynamically create the patern with the key
                    pattern = rf'({key}\s+)(\S+)(;)?'
                    replacement = lambda match: f"{match.group(1)}{value}{match.group(3) if match.group(3) else ';'}"
                    line = re.sub(pattern, replacement, line)

                    # edited_data.append(f"{key}  {value};\n")
                    line_modified = True
                    break # break the loop, other keys do not have to be evaluated once key has been matched

            edited_data.append(line)
                
        with open(file_paths[2],'w') as file:
            file.writelines(edited_data)

        # edited_data = []
        # for line in data:
        #     if line.strip().startswith()

        # Edit and apply Snappy HexMesh Dict settings 
        # TODO get refinement box values from blender dimensions
        # TODO set geometry file here
        # TODO File NAME STRING 
        filename_geometry = str(self.file)
        filename_geometry = filename_geometry.split(".")[0]    
    
        # 3. Edit and apply Snappy HexMesh Dict settings
        with open(file_paths[3], 'r') as file:
            data = file.readlines()
        
        edited_data = []
        dict_var = {}
        for key,value in self.config.cfd_config['mesh']['snappyHexMeshDict'].items():
            if isinstance(value,dict):
                for key1,value1 in value.items():
                    dict_var[key1] = value1
            else:
                dict_var[key] = value
        for line in data:
            # for key,value in self.config.cfd_config['mesh']['snappyHexMeshDict'].items():
            for key,value in dict_var.items():
                if line.strip().startswith(key):
                    # if isinstance(value,dict): # doing this will not copy dict after refinementBox
                    #     break
                    # edited_data.append(f"{key}  {value};\n")
                    if isinstance(value,list):
                        list_value_as_str = ' '.join(str(v) for v in value)  # Convert each element to str and join with spaces
                        pattern = rf'({key}\s+)\(\s*\S+\s+\S+\s+\S+\s*\);'  # Adjusted pattern to match the entire vector
                        replacement = rf'\1( {list_value_as_str} );'  # Properly formatted replacement string
                        line = re.sub(pattern, replacement, line)

                    else:
                        pattern = rf'({key}\s+)(\S+)(;)?'
                        replacement = lambda match: f"{match.group(1)}{value}{match.group(3) if match.group(3) else ';'}"
                        line = re.sub(pattern, replacement, line)
            # This line replaces the default "buildings" name with the current 3D env file
            if "buildings" in line:
                line = line.replace("buildings", filename_geometry)
                
            edited_data.append(line)
                
        with open(file_paths[3],'w') as file:
            file.writelines(edited_data)

        #4. Edit and apply surfaceFeatureExtractDict settings
        with open(file_paths[4], 'r') as file:
            data = file.readlines()
        
        edited_data = []
        
        for line in data:
            # This line replaces the default "buildings" name with the current 3D env file
            if "buildings" in line:
                line = line.replace("buildings", filename_geometry)
                
            edited_data.append(line)
        with open(file_paths[4], 'w') as file:
            data = file.writelines(edited_data) 

    def mesh_preparation(self):
        self._get_mesh_settings("new_parameters.yaml")
        self._new_cfd_case_dir()
        self._set_mesh_settings()
        
        self.apply_settings()
        



    def mesh(self):
        
        # Get into the the cfd folder to run openfoam 
        cfd_directory = self.config.directories["current_cfd_run"]
        # cfd_directory = cfd_directory.replace(self.directory_path+"/","")
        # TODO set openfoam directory from settings
        
        # openfoam_init_script = os.path.expanduser('~/Applications/OpenFOAM/OpenFOAM-v2212/etc/bashrc')
        openfoam_dir = self.config.software_settings["openfoam"]["openfoam_dir"].format(home_dir=self.home_dir, openfoam_version=self.config.software_settings["openfoam"]["version"])
        openfoam_init_script = os.path.expanduser(f"{openfoam_dir}/etc/bashrc")
        # source_command = f'source {openfoam_init_script}'
        # # source_command = f"source {self.config.software_settings.openfoam_dir} of2212 && echo 'OpenFOAM sourced successfully'"


        # # subprocess.run(source_command, shell=True, executable='/bin/bash', check=True)
        # # os.system(f'/bin/bash -c "{source_command}"')
        # # os.system(f"{source_command}")
        # # # os.system('of2212')

        # # os.system('source home/kjell/Applications/OpenFOAM/OpenFOAM-v2212/etc/bashrc')

        # # Run the bash file inside the map, Allrun.pre
        # # It's assumed that 'Allrun.pre' is executable; otherwise, you might need to prepend with 'bash'
        # allrun_command = os.path.join(cfd_directory,"./Allrun.pre")
        # # subprocess.run(allrun_command, shell=True, check=True)
        # # os.system(allrun_command)
        # os.system(f'/bin/bash -c "{source_command}" && cfd/cfd_runs/run_20240305_082652_buildings/./Allrun.pre')
        # Then we run the bash file inside the map, Allrun.pre

        source_command = f'source {openfoam_init_script}'
        run_command = f"{cfd_directory}/./Allrun.pre"
        cmd = f"{source_command} && {run_command}"
        # cmd = f"source $HOME/Applications/OpenFOAM/OpenFOAM-v2212/etc/bashrc && {cfd_directory}/./Allrun.pre"
        subprocess.run(cmd,shell=True,executable='/bin/bash')
        
        # TODO try running  
    # todo get environment limits to set mesh distances

    


    # TODO when time left. Run in parallel to speed up
    def run(self):
        self.mesh_preparation()
        print("Mesh Preparation done, starting to mesh")
        self.mesh()

        # Run the CFD
        cfd_directory = self.config.directories["current_cfd_run"]
        openfoam_dir = self.config.software_settings["openfoam"]["openfoam_dir"].format(home_dir=self.home_dir, openfoam_version=self.config.software_settings["openfoam"]["version"])
        openfoam_init_script = os.path.expanduser(f"{openfoam_dir}/etc/bashrc")
        source_command = f'source {openfoam_init_script}'
        run_command = f"{cfd_directory}/./Allrun"
        cmd = f"{source_command} && {run_command}"
        subprocess.run(cmd,shell=True,executable='/bin/bash')
       
       
       
        # postprocess_command = "postProcess -func 'components(U)' && postProcess -func 'writeCellCentres'"
        # postprocess_command = f"{cfd_directory}/{postprocess_command}"
        # postprocess_command = f"{source_command} && {postprocess_command}"
        # subprocess.run(postprocess_command,shell=True, executable='/bin/bash')
        # Set cwd to cfd folder thus self.directories["current_cfd_run"]
        # Make trisurface direcotry 
    
        # copy file to trisurface directory 
    
        # 
        # mesh files 
    
        #

    def wind_field_generation(self):
        """
        Here the wind field will be saved and created
        """
        


    def save_wind_field(self):
        """
        Windfield data look at main.py hajo line 361 to 266
        """


        pass

# # cfd_sim = CFDSimulation(file="buildings.obj")  #Buildings New is the file that is set for the CFD
# # cfd_sim = CFDSimulation(file="buildings_american_grid.obj")  #Buildings New is the file that is set for the CFD
# # cfd_sim = CFDSimulation(file="random_field.obj")  #Buildings New is the file that is set for the CFD
# # cfd_sim = CFDSimulation(file="TU_Delft_campus.obj")  #Buildings New is the file that is set for the CFD
# cfd_sim = CFDSimulation(file="random_field_flow_test.obj", wind_vel=10.0)  #Buildings New is the file that is set for the CFD
# # cfd_sim.mesh_preparation()
# # cfd_sim.mesh()
# cfd_sim.run()
# # print(cfd_sim.config.cfd_config['mesh']['snappyHexMeshDict'])
# # for key,value in cfd_sim.config.cfd_config['mesh']['snappyHexMeshDict'].items():
# #     print(key,value)
    

# Blender file to stl to mesh


# Get dimensions



# Rotate from xyz to xzy openfoam --> OpenFOAM its axis is as followed: wind coming from x and height of buildings is in y direction. Please look at video of girl 


# Set OpenFOAM parameters 



# How to change parameters inside feature extract dict etc.. TODO look at code Hajo

# make sure to make trisurface file

# Run geometry : 

"""
1. Surface Feature Extract
2. BlockMesh
3. SnappyHexMesh
"""



# check 0 files




# Run CFD : SimpleFoam