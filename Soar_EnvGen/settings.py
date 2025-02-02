import os
from pathlib import Path
import math 
import subprocess
import yaml

home_dir = Path.home()
directory_path = os.path.dirname(os.path.abspath(__file__))




# source right openfoam version
openfoam_version = "of2212"
openfoam_dir = f"{home_dir}/Applications/OpenFOAM/OpenFOAM{openfoam_version}"


# source_bash = 
# os.system(openfoam_cmd)


class ConfigManager:
    def __init__(self):
        self.home_dir = Path.home()
        self.directory_path = os.path.dirname(os.path.abspath(__file__))
        self.default_config_dir = {
            'directories': {
                'cfd_dir': '/path/to/cfd',
                'blender_dir': '/path/to/blender',
                'config_dir': f"{self.directory_path}/config/"
                # Add other directories as needed
            }
        }


        self.config = {}

        self.default_config_yaml = self.load_config()




    def set_dir_config(self):
        pass

    def load_config(self, filename=None):
        if filename is None:
            filename = f"{self.default_config_dir['directories']['config_dir']}default.yaml"

        
        """Load a configuration from a YAML file."""
        if not os.path.exists(filename):
            print(f"File {filename} not found. Using default configuration.")
            filename = f"{self.default_config_dir['directories']['config_dir']}default.yaml"
            with open(filename) as file:
                self.config = yaml.safe_load(file)
            return self.config
        with open(filename) as file:
            self.config = yaml.safe_load(file)
            # No need to interpolate {home_dir} when loading since it's for output only

        return self.config

    def save_config(self, filename='current.yaml'):
        """Save the current configuration to a YAML file."""
        with open(filename, 'w') as file:
            yaml.dump(self.config, file, default_flow_style=False)

    

    def set_configs(self, yamldict:dict):

        # software versions
        

        # Directories


        # Cfd settings
        pass 


        


"""
This file will need to load all the settings for openfoam in the a dict. These settings are read in from a json or yml  file 
This settings dict then go in to the settings variable for the cfdSimulation class.
"""

# cfm = ConfigManager()

# print(cfm.config)

#     def update_config(self, updates):
#         """Recursively update the configuration with a given dictionary."""
#         def recursive_update(original, updates):
#             for key, value in updates.items():
#                 if isinstance(value, dict):
#                     original[key] = recursive_update(original.get(key, {}), value)
#                 else:
#                     original[key] = value
#             return original
        
#         self.config = recursive_update(self.config, updates)

# # Usage
# config_manager = CfdConfigManager()

# # Save the initial default configuration with {home_dir} interpolated
# config_manager.save_config()

# # To update and save current configuration with {home_dir} interpolation:
# updates = {
#     'directories': {
#         'cfd_dir': '{home_dir}/new/path/to/cfd',
#     },
#     'software_versions': {
#         'blender': '3.8.0',
#     },
#     'cfd_settings': {
#         'endTime': 2000,
#         'deltaT': 0.5,
#     },
#     'blender_settings': {
#         'resolution': [2560, 1440],
#     }
# }
# config_manager.update_config(updates)