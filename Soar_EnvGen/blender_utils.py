import bpy 
import numpy as np 
import os 

# import pandas as pd 
import mathutils


class BlenderUtils:

    def __init__(self):
        # self.file = file
        self.blend_files_dir = "blender_envs/blend_files/" #os.path.dirname(file)

    def load_blend_file(self, file_name):
        # Construct the full path to the .blend file
        file_path = os.path.join(self.blend_files_dir, file_name)
        # Check if the file exists
        if not os.path.isfile(file_path):
            print(f"File {file_path} not found.")
            return False
        # Load the .blend file
        bpy.ops.wm.open_mainfile(filepath=file_path)
        return True
    
    def find_ground_plane(self):
        def recursive_search(collection):
            for obj in collection.objects:
                if obj.name == "GroundPlane":
                    return obj
            for sub_collection in collection.children:
                result = recursive_search(sub_collection)
                if result:
                    return result
            return None

        for collection in bpy.data.collections:
            ground_plane = recursive_search(collection)
            if ground_plane:
                return ground_plane
        return None


    def obj_export(self, file_name, obj_file_name=None):
        if not self.load_blend_file(file_name):
            return None
        if obj_file_name is None:
            obj_file_name = file_name.replace(".blend", ".obj")
        # obj_file = file_name.replace(".blend", ".obj")
        obj_file = os.path.join(self.blend_files_dir,obj_file_name)
        # bpy.ops.export_scene.obj(filepath=obj_file, check_existing=False,use_selection=False)
      
        # Setting axis forward and axis up as followed makes sure that it is already correclty orientated for paraview.
        bpy.ops.export_scene.obj(filepath=obj_file, check_existing=False,use_selection=False,axis_forward="Y",axis_up ="Z") 
        return obj_file
    
    # def usd_export(self, file_name, usd_file_name=None):
    #     if not self.load_blend_file(file_name):
    #         return None
    #     if usd_file_name is None:
    #         usd_file_name = file_name.replace(".blend", ".usd")
    #     usd_file = os.path.join(self.blend_files_dir, usd_file_name)
    #     bpy.ops.wm.usd_export(filepath=usd_file, check_existing=False, selected_objects_only=False, visible_objects_only=False)
    #     return usd_file
    def usd_export(self, file_name, usd_file_name=None):
        if not self.load_blend_file(file_name):
            return None
        if usd_file_name is None:
            usd_file_name = file_name.replace(".blend", ".usd")
        usd_file = os.path.join(self.blend_files_dir, usd_file_name)
        
        bpy.ops.wm.usd_export(
            filepath=usd_file,
            check_existing=False,
            selected_objects_only=False,
            visible_objects_only=False,
            export_textures=True,       # Ensure textures are exported
            export_materials=True,  # Ensure materials are exported
            root_prim_path="/Collection",    # Set the root prim path to /World
        )


        return usd_file
    
    def get_all_collection_names(self, file_name):
        if not self.load_blend_file(file_name):
            return None 
        collection_names = [collection.name for collection in bpy.data.collections]
        print("Available collections:")
        for name in collection_names:
            print(f"  {name}")
        return collection_names 


    def get_environment_dimensions(self, file_path:str):
        """
        Get the dimensions of the environment in the Blender file.
        """
        # Load the Blender file
        if not self.load_blend_file(file_path):
            return None

        # Assuming your environment is a collection named "Environment"
        collection_name = self.get_all_collection_names(file_name=file_path)[0]
        environment_collection = bpy.data.collections.get(collection_name)
        # environment_collection = bpy.data.collections.get("Collection")
        
        if not environment_collection:
            print("Environment collection not found.")
            return None

        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')

        for obj in environment_collection.objects:
            if obj.type == 'MESH':
                # Update the bounding box
                for vert in obj.bound_box:
                    world_vert = obj.matrix_world @ mathutils.Vector(vert)
                    min_x = min(min_x, world_vert.x)
                    min_y = min(min_y, world_vert.y)
                    min_z = min(min_z, world_vert.z)
                    max_x = max(max_x, world_vert.x)
                    max_y = max(max_y, world_vert.y)
                    max_z = max(max_z, world_vert.z)

        dimensions = {
            'length_x': max_x - min_x,
            'length_y': max_y - min_y,
            'length_z': max_z - min_z,
        }

        return dimensions
        



# methods to get dimensions of blender environment 



# Transform blender file to right cooordinates

# other bledner utils



# testing
    
bu = BlenderUtils()

# file = "buildings_new.blend"
# file = "buildings.blend"
file = "random_field_flow_test.blend"

bu.usd_export(file_name=file)

# bu.obj_export(file_name =file)
# # print(bu.obj_export(file))

# bu.get_all_collection_names(file)   
# dimensions = bu.get_environment_dimensions(file)
# print(dimensions)

