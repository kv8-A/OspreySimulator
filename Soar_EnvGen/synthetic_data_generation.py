import numpy as np
import imagio
import json

def generate_depth_map():
    pass



# Function to load depth map from an EXR file
def load_depth_map(file_path):
    depth_map = imagio.imread(file_path, format='EXR')
    return depth_map

# Perform euclidean distance transform on the depth map
# The goal is to get the distance of the updraft regions and map it to where there is an object in the depth map


# Updrafts regions are only above a certain height.
# The height is determined by knowing the camera position and the height of the object in the depth map
# heigt of the object in the depth map is euclidean distance calculations need. 
# Need to map a depth map to a 3D position in space


def calculate_pixel_position(camera_position, direction_vector, distance):
    """
    Calculate the 3D position of a pixel given the camera position,
    direction vector, and Euclidean distance to the pixel.
    
    :param camera_position: A tuple or list of (x, y, z) representing the camera's position.
    :param direction_vector: A tuple or list of (dx, dy, dz) representing the direction vector.
    :param distance: A float representing the Euclidean distance from the camera to the pixel.
    :return: A tuple of (x, y, z) representing the 3D position of the pixel.
    """
    camera_position = np.array(camera_position)
    direction_vector = np.array(direction_vector)
    
    # Normalize the direction vector
    direction_magnitude = np.linalg.norm(direction_vector)
    normalized_direction = direction_vector / direction_magnitude
    
    # Calculate the pixel position in 3D space
    pixel_position = camera_position + distance * normalized_direction
    
    return tuple(pixel_position)

# Example usage
camera_position = (300, 499, 20)
direction_vector = (1, 1, 0)  # Replace with actual direction
distance = 10

pixel_position = calculate_pixel_position(camera_position, direction_vector, distance)
print("Pixel Position in 3D Space:", pixel_position)

# Get positions per object, if camera is at position (300, 499, 20) and eucledian distance is 10, 
# what is the distance 


# Dummy function to calculate updraft regions
def calculate_updraft_regions(depth_map, wind_direction='head'):
    updraft_regions = np.zeros_like(depth_map, dtype=bool)
    if wind_direction == 'head':
        threshold = 10  # Example threshold
        updraft_regions = depth_map < threshold
    return updraft_regions

# Function to combine depth map and updraft regions into a JSON
def combine_data_to_json(depth_map, updraft_regions, output_file_path):
    data = {'pixels': []}
    height, width = depth_map.shape
    for y in range(height):
        for x in range(width):
            pixel_data = {
                'x': x,
                'y': y,
                'depth': float(depth_map[y, x]),
                'updraft': bool(updraft_regions[y, x])
            }
            data['pixels'].append(pixel_data)
    with open(output_file_path, 'w') as f:
        json.dump(data, f, indent=4)

# Load depth map
depth_map_file_path = 'path/to/depth_map.exr'
depth_map = load_depth_map(depth_map_file_path)

# Calculate updraft regions
wind_direction = 'head'  # or 'tail'
updraft_regions = calculate_updraft_regions(depth_map, wind_direction)

# Combine data and save to JSON
output_file_path = 'path/to/combined_data.json'
combine_data_to_json(depth_map, updraft_regions, output_file_path)
