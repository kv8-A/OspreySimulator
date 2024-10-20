import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from edge_detection import extract_upper_edges_main

def load_camera_parameters(json_path: str) -> dict:
    """
    Load camera parameters from a JSON file.

    Args:
        json_path (str): Path to the JSON file containing camera parameters.

    Returns:
        dict: A dictionary with camera parameters including intrinsics and pose.
    """
    with open(json_path, 'r') as file:
        camera_params = json.load(file)

    # Convert position and rotation to numpy arrays
    camera_params['position'] = np.array(camera_params['position'], dtype=np.float32)
    camera_params['rotation'] = np.array(camera_params['rotation'], dtype=np.float32)

    return camera_params

def pixel_to_world(u: int, v: int, distance: float, intrinsics: dict, camera_position: np.ndarray, camera_rotation: np.ndarray) -> np.ndarray:
    """
    Convert pixel coordinates and distance to world coordinates.

    Args:
        u (int): Pixel x-coordinate.
        v (int): Pixel y-coordinate.
        distance (float): Distance from camera to the point.
        intrinsics (dict): Dictionary with camera intrinsics values (fx, fy, cx, cy).
        camera_position (np.ndarray): Camera position as (3,).
        camera_rotation (np.ndarray): Camera rotation as Euler angles (3,) in degrees.

    Returns:
        np.ndarray: 3D point in world coordinates (3,).
    """
    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']

    # Convert pixel coordinates to normalized camera coordinates
    x = (u - cx) * distance / fx
    y = (v - cy) * distance / fy
    z = distance

    # Camera coordinates in homogeneous coordinates
    point_camera = np.array([x, y, z, 1], dtype=np.float32).reshape(4, 1)

    # Create a 4x4 transformation matrix from position and rotation
    rotation_matrix = R.from_euler('xyz', camera_rotation, degrees=True).as_matrix()
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = camera_position

    # camera_view_transform = np.array([
    #     [4.930380657631324e-32, -1.0, -1.0947644252537633e-47, 0.0],
    #     [ 1.0, 4.930380657631324e-32, -2.220446049250313e-16, 0.0],
    #     [ 2.220446049250313e-16, 1.0947644252537633e-47, 1.0, -0.0],
    #     [ -6.661338147750939e-15, -3.28429327576129e-46, -30.0, 1.0]
    # ])
    # Transform to world coordinates
    point_world_homogeneous = np.dot(transform_matrix, point_camera)
    # point_world_homogeneous = np.dot(camera_view_transform, point_camera)
    point_world = point_world_homogeneous[:3, 0] / point_world_homogeneous[3, 0]

    return point_world

def create_world_coordinate_array(distance_image: np.ndarray, edges: np.ndarray, intrinsics: dict, camera_params: dict) -> np.ndarray:
    """
    Generate a 512x512 array of world coordinates for edge pixels based on distance image.

    Args:
        distance_image (np.ndarray): 2D array of distances .
        edges (np.ndarray): Array of edge pixel coordinates (N, 2) where N is the number of edge pixels.
        intrinsics (dict): Camera intrinsics as a dictionary.
        camera_params (dict): Camera parameters dictionary with position and rotation.

    Returns:
        np.ndarray: A 3D array (distance_image.shape[0], distance_image.shape[1], 3) with world coordinates of edge pixels.
    """
    # Initialize an empty 512x512x3 array for world coordinates
    # world_coords_array = np.full((512, 512, 3), np.nan, dtype=np.float32)
    # world_coords_array = np.full((512, 720, 3), np.nan, dtype=np.float32)
    distance_image_shape = distance_image.shape
    world_coords_array = np.full((distance_image_shape[0], distance_image_shape[1], 3), np.nan, dtype=np.float32)

    # Get camera position and rotation
    camera_position = camera_params['position']
    camera_rotation = camera_params['rotation']

    # idx = 0
    # for (v, u) in edges:  # OpenCV uses (v, u) for (row, col) or (y, x)
    #     distance = distance_image[v, u]  # This is correct for OpenCV indexing
    #     if distance > 0:  # Only process valid distances
    #         # u corresponds to x and v corresponds to y in the pixel_to_world function
    #         world_coord = pixel_to_world(u, v, distance, intrinsics, camera_position, camera_rotation)
    #         world_coords_array[v, u] = world_coord  # Store in the 512x512 array
    #         print(f"Distance at idx {idx}: {distance}")
    #         print(f"World coordinate at idx {idx}: {world_coord}")
    #         idx += 1
    for u in range(distance_image_shape[1]):
        for v in range(distance_image_shape[0]):
            distance = distance_image[v, u]
            if distance > 0:
                world_coord = pixel_to_world(u, v, distance, intrinsics, camera_position, camera_rotation)
                world_coords_array[v, u] = world_coord  # Store in the 512x512 array
                print(world_coord)
    return world_coords_array




# Example usage
input_file = 'synthetic_data_generation/output/distance_to_camera_0001.npy'
camera_file = f"synthetic_data_generation/output/camera_pose_{input_file.split('/')[-1].split('_')[-1].split('.')[0]}.json"
camera_params = load_camera_parameters(camera_file)
intrinsics = camera_params['intrinsics']

distance_image = np.load(input_file)
# `edges` is assumed to be an array of shape (N, 2) with pixel coordinates of edges
print(distance_image[400])

# edges = extract_upper_edges_main(input_file)
edges = np.array([])

world_coords_array = create_world_coordinate_array(distance_image, edges, intrinsics, camera_params)
array_of_coordinates = world_coords_array[300]
print("World coordinates array shape:", world_coords_array.shape)
print(world_coords_array[300])