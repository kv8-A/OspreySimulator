import bpy
from blender_utils import BlenderUtils
from blender_envs.random_building_enviroment_creator import create_random_field_of_buildings
import sys
import argparse

class RandomBuildingEnviromentCreator:
    def __init__(self, building_folder, building_texture_folder, plane_texture_folder, field_size, building_count, output_filepath):
        self.building_folder = building_folder
        self.building_texture_folder = building_texture_folder
        self.plane_texture_folder = plane_texture_folder
        self.field_size = field_size
        self.building_count = building_count
        self.output_filepath = output_filepath
        self._blend_utils = BlenderUtils()

    def create_random_field_of_buildings(self):
        create_random_field_of_buildings(self.building_folder, self.building_texture_folder, self.plane_texture_folder, self.field_size, self.building_count, self.output_filepath)

    def usd_export(self):
        self._blend_utils.usd_export(file_name=self.output_filepath)

    def obj_export(self):
        self._blend_utils.obj_export(file_name=self.output_filepath)    

    def main(self):
        self.create_random_field_of_buildings()
        self.usd_export()
        self.obj_export()   

    
# def run_from_cli():
#     # Use argparse to handle command-line arguments
#     parser = argparse.ArgumentParser(description='Create a random field of buildings and export in USD and OBJ formats.')
#     parser.add_argument('building_folder', type=str, help='Folder containing building models')
#     parser.add_argument('building_texture_folder', type=str, help='Folder containing building textures')
#     parser.add_argument('plane_texture_folder', type=str, help='Folder containing plane textures')
#     parser.add_argument('field_size', type=int, help='Size of the field')
#     parser.add_argument('building_count', type=int, help='Number of buildings to generate')
#     parser.add_argument('output_filepath', type=str, help='Filepath to save the output')

#     args = parser.parse_args()

#     # Create an instance of RandomBuildingEnviromentCreator
#     creator = RandomBuildingEnviromentCreator(
#         building_folder=args.building_folder,
#         building_texture_folder=args.building_texture_folder,
#         plane_texture_folder=args.plane_texture_folder,
#         field_size=args.field_size,
#         building_count=args.building_count,
#         output_filepath=args.output_filepath
#     )

#     # Run the main functionality
#     creator.run()

# if __name__ == "__main__":
#     run_from_cli()

building_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files"  # Replace with the path to your building assets folder
building_texture_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files/textures/building_textures"
plane_texture_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files/textures/ground_planes"
field_size = 750  # Size of the field (e.g., 200x200 units)
building_count = 50  # Number of buildings to place in the field
# output_filepath = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/random_field_new.blend"  # Replace with the desired save path
output_filepath = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/random_field_flow_test.blend"  # Replace with the desired save path

# Clear existing meshes (optional)
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Run the main function
# create_random_field_of_buildings(
#     building_folder,
#     building_texture_folder,
#     plane_texture_folder,
#     field_size,
#     building_count,
#     output_filepath
# )

creator = RandomBuildingEnviromentCreator(
    building_folder=building_folder,
    building_texture_folder=building_texture_folder,
    plane_texture_folder=plane_texture_folder,
    field_size=field_size,
    building_count=building_count,
    output_filepath=output_filepath
)

creator.main()