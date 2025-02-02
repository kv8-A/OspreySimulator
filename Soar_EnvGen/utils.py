from pxr import Usd, UsdGeom, UsdUtils, Gf
import os
from typing import Tuple

class Utils:
    def __init__(self):
        pass   


    def get_obj_dimensions(self, obj_file_name:str, path_to_file:str) -> dict:
        """
        Get the dimensions of an object file.
        Args:
            obj_file_name (str): The name of the object file.
            path_to_file (str): The path to the object file.
        Returns:
            dict: A dictionary containing the width, height, and depth of the object.
        """
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        file_path = f"{path_to_file}/{obj_file_name}"

        with open(file_path, 'r') as obj_file:
            for line in obj_file:
                if line.startswith('v '):  # This line defines a vertex
                    _, x, y, z = line.strip().split()
                    x, y, z = float(x), float(y), float(z)

                    min_x = min(min_x, x)
                    min_y = min(min_y, y)
                    min_z = min(min_z, z)
                    max_x = max(max_x, x)
                    max_y = max(max_y, y)
                    max_z = max(max_z, z)

        width = max_x - min_x
        height = max_y - min_y
        depth = max_z - min_z
        
        dimensions = {
            'width': width,
            'height': height,
            'depth': depth
        }   

        return dimensions
    
    def get_obj_extreme_coordinates(self, obj_file_name:str, path_to_file:str) -> Tuple[dict,dict,dict]:
        """
        Get the dimensions of a Blender object.
        """
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        file_path = f"{path_to_file}/{obj_file_name}"

        with open(file_path, 'r') as obj_file:
            for line in obj_file:
                if line.startswith('v '):  # This line defines a vertex
                    _, x, y, z = line.strip().split()
                    x, y, z = float(x), float(y), float(z)

                    min_x = min(min_x, x)
                    min_y = min(min_y, y)
                    min_z = min(min_z, z)
                    max_x = max(max_x, x)
                    max_y = max(max_y, y)
                    max_z = max(max_z, z)
        
        x_coor = {
            'min': min_x,
            'max': max_x
        }
        y_coor = {  
            'min': min_y,
            'max': max_y
        }   
        z_coor = {
            'min': min_z,
            'max': max_z
        }   
        return x_coor, y_coor, z_coor   
    
    
    def usd_to_obj(self, usd_file_path, obj_file_path):
        # TODO does not work. TO difficult do manually in Blender or Isaac Sim
        """
        Converts a USD file to an OBJ file.
        
        Args:
            usd_file_path (str): The path to the input USD file.
            obj_file_path (str): The path to the output OBJ file.
        """
        try:
            # Ensure the input file exists
            if not os.path.exists(usd_file_path):
                raise FileNotFoundError(f"USD file not found: {usd_file_path}")

            # Load the USD stage
            stage = Usd.Stage.Open(usd_file_path)
            if not stage:
                raise RuntimeError("Failed to open the USD file.")

            # Find all geometry in the USD file
            geom_prims = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]

            # Export the geometry to OBJ
            with open(obj_file_path, 'w') as obj_file:
                for prim in geom_prims:
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
                    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get()

                    # Write OBJ data
                    for point in points:
                        obj_file.write(f"v {point[0]} {point[1]} {point[2]}\n")

                    for count, indices in zip(face_vertex_counts, face_vertex_indices):
                        obj_file.write("f")
                        for i in range(count):
                            obj_file.write(f" {indices + 1}")
                        obj_file.write("\n")
            
            print(f"USD to OBJ conversion successful: {obj_file_path}")

        except Exception as e:
            print(f"An error occurred during USD to OBJ conversion: {e}")


# Example usage
# usd_file = "usd_files/campus_Final.usd"
# obj_file = "usd_files/campus_Final.obj"
# u = Utils()


# # "blender_envs/blend_files/random_field.obj"
# dimensions = u.get_obj_dimensions("random_field.obj", "blender_envs/blend_files")
# x_coor, y_coor, z_coor = u.get_obj_extreme_coordinates("random_field.obj", "blender_envs/blend_files")
# dimensions = u.get_obj_dimensions("random_field.obj", "blender_envs/blend_files")
# x_coor, y_coor, z_coor = u.get_obj_extreme_coordinates("random_field.obj", "blender_envs/blend_files")
# # u.usd_to_obj(usd_file, obj_file)
# print(dimensions)
# print(x_coor, y_coor, z_coor)
# import numpy as np
# print(np.round(z_coor['min'],1))