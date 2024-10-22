"""

Objective: For each pixel corresponding to the top edges of buildings, you want to move the associated 3D coordinate a 
bit upward and a bit forward (say, by 1 meter each). Then, at these new positions, you want to query your wind field to obtain 
the vertical wind velocity components, average them, and create a new image of shape (H, W) representing the upward wind vector at each pixel.
The key steps involved are:

Obtain 3D Coordinates of Top Edge Pixels: You already have the 3D coordinates for each pixel from your distance image.

Move Coordinates Upward and Forward: For each top-edge pixel, adjust its 3D coordinate upward and forward by 1 meter.

Query the Wind Field: Use a KDTree or another method to find the wind vectors at these new positions.

Average Vertical Wind Components: For each pixel, average the vertical components (z-components) of the wind vectors obtained.

Create the Output Image: Store these averaged vertical wind components in a new image of the same size.
"""

import numpy as np
import os
from typing import List, Tuple
from scipy.spatial import KDTree
from numpy.typing import NDArray

from edge_detection import extract_upper_edges_noconnect, extract_edges_pixels


class WindFieldProcessor:
    def __init__(self, wind_field_path: str, world_coor_path: str, output_path: str):
        """
        Initializes the WindFieldProcessor with paths to the wind field data, world coordinate files, and output directory.

        :param wind_field_path: Path to the wind field .npz file.
        :param world_coor_path: Path to the directory containing world coordinate .npy files.
        :param output_path: Path to the directory where the output vertical wind images will be saved.
        """
        self.wind_field_path: str = wind_field_path
        self.world_coor_path: str = world_coor_path
        self.output_path: str = output_path
        self.velocity_vectors: NDArray[np.float64] | None = None
        self.cell_positions: NDArray[np.float64] | None = None
        self.world_coordinates_image: np.ndarray
        self.kd_tree: KDTree | None = None
        self.H: int = 0
        self.W: int = 0
        self.top_edge_pixels: List[Tuple[int, int]] = []

    def _load_wind_field_data(self) -> None:
        """
        Loads the wind field data (velocity vectors and cell positions) from the .npz file.
        """
        data = np.load(self.wind_field_path)
        self.velocity_vectors = data['velocity_vectors']
        self.cell_positions = data['cell_positions']
        self.kd_tree = KDTree(self.cell_positions)
        print(f"Wind field data loaded from {self.wind_field_path}")
    
    def _load_world_coordinates(self, idx: int) -> np.ndarray:
        """
        Loads the world coordinates from a .npy file corresponding to the given index.

        :param idx: String index for the world coordinate file (e.g., '0001').
        :return: A numpy array of shape (H, W, 3) containing the world coordinates.
        """
        idx_str = str(idx).zfill(4)

        world_coor_file = os.path.join(self.world_coor_path, f"reshaped_point_cloud_{idx_str}.npy")
        world_coordinates_image = np.load(world_coor_file)
        self.H, self.W, _ = world_coordinates_image.shape  # Initialize H and W if loading for the first time
        print(f"World coordinates loaded for {idx} from {world_coor_file}")
        self.world_coordinates_image = world_coordinates_image
        return world_coordinates_image
    
    def move_coordinate(self, original_coord: np.ndarray, movement_vector: np.ndarray) -> np.ndarray:
        """
        Moves a coordinate by the specified movement vector.

        Args:
            param original_coord: np.ndarray of shape (3,), original coordinate.
            param movement_vector: np.ndarray of shape (3,), movement vector.
        
        return: 
            np.ndarray of shape (3,), new coordinate.
        """
        return original_coord + movement_vector
    
    def interpolate_wind_vector(self, new_coord: np.ndarray, k: int = 4) -> np.ndarray:
        """
        Interpolates the wind vector at a new coordinate using inverse distance weighting.

        Args:
            new_coord: np.ndarray of shape (3,), the coordinate at which to interpolate.
            k: int, number of nearest neighbors to use.

        Returns:
            np.ndarray of shape (3,), interpolated wind vector.
        """
        if self.kd_tree is None:
            raise ValueError("KDTree is not initialized. Load wind field data first.")
        if self.velocity_vectors is None:
            raise ValueError("Velocity vectors are not initialized. Load wind field data first.")

        distances, indices = self.kd_tree.query(new_coord, k=k)
        neighbor_wind_vectors = self.velocity_vectors[indices]

        # Handle the case where distance is zero (query point exactly at a data point)
        zero_distance_mask = distances == 0
        if np.any(zero_distance_mask):
            interpolated_wind = neighbor_wind_vectors[zero_distance_mask][0]
        else:
            # Inverse distance weights
            weights = 1 / distances
            weights /= weights.sum()
            interpolated_wind = np.sum(neighbor_wind_vectors * weights[:, np.newaxis], axis=0)

        return interpolated_wind

    def process_pixel(self, y: int, x: int, movement_vector: np.ndarray) -> float:
        """
        Processes a single pixel by moving its coordinate and interpolating the vertical wind component.

        Args:
            y: int, y-coordinate of the pixel.
            x: int, x-coordinate of the pixel.
            movement_vector: np.ndarray of shape (3,), movement vector to apply.
        
        Returns:
            float: The vertical wind component at the moved coordinate.
        """
        original_coord = self.world_coordinates_image[y, x]
        # Check if the original coordinate is valid (not NaN)

        if np.isnan(original_coord).any():
            # Check coordinate above or below because the edge pixel should exist
            # for dy in range(1, 7):
            #     new_y = y - dy
            #     if new_y >= self.H:
            #         break
            #     new_coord = self.world_coordinates_image[new_y, x]
            #     if not np.isnan(new_coord).any():
            #         original_coord = new_coord
            #         print(f"originial coord found at index {dy}")
            #         print(f"edge was at {y}, {x}, but was nan so now its at {new_y}, {x}") 
            #         new_coord = self.move_coordinate(original_coord, movement_vector)
            #         interpolated_wind = self.interpolate_wind_vector(new_coord)
            #         vertical_wind = interpolated_wind[2]  # Extract the vertical component
            #         return vertical_wind
                
            return np.nan  # Return NaN for invalid coordinates

        new_coord = self.move_coordinate(original_coord, movement_vector)
        interpolated_wind = self.interpolate_wind_vector(new_coord)
        vertical_wind = interpolated_wind[2]  # Extract the vertical component
        return vertical_wind

    def process_top_edge_pixels(self, movement_vector: np.ndarray, edges: np.ndarray) -> np.ndarray:
        """
        Processes only the top-edge pixels and computes the vertical wind component for each.

        Args:
            movement_vector: np.ndarray of shape (3,), movement vector to apply to each coordinate.
            edges: List of (y, x) tuples representing the pixel coordinates of the edges.

        Returns:
            np.ndarray of shape (H, W): An array containing the vertical wind component at each pixel.
        """
        if self.world_coordinates_image is None:
            raise ValueError("World coordinates are not loaded. Load world coordinates first.")

        self.vertical_wind_image = np.full((self.H, self.W), np.nan)  # Initialize the output image with NaN

        count_nans = 0
        count_valid = 0
        for x, y in edges:
            # Move y up to contour nan problems with incorrect edge detection
            y = y +2
            if 0 <= y < self.H and 0 <= x < self.W:
                vertical_wind = self.process_pixel(y, x, movement_vector)
                if np.isnan(vertical_wind):
                    count_nans += 1
                else:
                    count_valid += 1
                self.vertical_wind_image[y, x] = vertical_wind

                 # Assign vertical wind to the 10 pixels above
                for dy in range(1, 11):
                    new_y = y - dy
                    if new_y < 0:
                        break  # Outside image bounds
                    if np.isnan(self.vertical_wind_image[new_y, x]):
                        self.vertical_wind_image[new_y, x] = vertical_wind
                    else:
                        break  # Stop if invalid coordinate
            else:
                print(f"Pixel coordinate ({y}, {x}) is out of bounds and will be skipped.")
        print(f"Processed {count_valid} valid pixels and skipped {count_nans} invalid edges pixels.")
        return self.vertical_wind_image

    def get_vertical_wind_image(self) -> np.ndarray:
        """
        Returns the vertical wind image.

        Returns:
            np.ndarray of shape (H, W): The vertical wind component image.
        """
        return self.vertical_wind_image

    def process_all_data(self, movement_vector: np.ndarray) -> None:
        """
        Processes all the world coordinate files in the specified directory and saves the vertical wind images.

        Args:
            movement_vector: np.ndarray of shape (3,), movement vector to apply to each coordinate.
        """
        # Extract wind velocity from the wind field path
        velocity_str = self.wind_field_path.split('_v_')[-1].split('.npz')[0]
        # Get a sorted list of all world coordinate files
        world_coor_files = sorted([f for f in os.listdir(self.world_coor_path) if f.startswith("reshaped_point_cloud_") and f.endswith(".npy")])

        for world_coor_file in world_coor_files:
            idx = int(world_coor_file.split('_')[-1].split('.')[0])
            # Load world coordinates for the current index
            self._load_world_coordinates(idx)

            # Load the corresponding depth image to detect edges
            depth_image_file = os.path.join(self.world_coor_path, f"distance_to_camera_{str(idx).zfill(4)}.npy")
            depth_image = np.load(depth_image_file)

            # Extract the edges
            # edges = extract_upper_edges_noconnect(depth_image)
            edges = extract_edges_pixels(depth_image)

            # Process only the top-edge pixels
            vertical_wind_image = self.process_top_edge_pixels(movement_vector, edges)

            # Save the resulting vertical wind image to the output directory
            output_file = os.path.join(self.output_path, f"vertical_wind_image_edges_{str(idx).zfill(4)}_v_{velocity_str}.npy")
            np.save(output_file, vertical_wind_image)
            print(f"Vertical wind image saved to {output_file}")

if __name__ == "__main__":
    # Set paths for the wind field data, world coordinate files, and output directory
    wind_field_path = "synthetic_data_generation/wind_fields/Random_world_test/run_20241021_210945_random_field_flow_test_10_0/flow_field_data_100_v_10_0.npz"
    world_coor_path = "synthetic_data_generation/output"
    output_path = "synthetic_data_generation/processed_output"

    # Initialize the WindFieldProcessor with the paths
    processor = WindFieldProcessor(wind_field_path, world_coor_path, output_path)

    # Load the wind field data
    processor._load_wind_field_data()

    # Define the movement vector (e.g., 1m forward in x, 1m upward in z)
    movement_vector = np.array([0.0, 0.0, 1.0])

    # Process all data and save the vertical wind images
    processor.process_all_data(movement_vector)





# def main():
#     import numpy as np
#     import os

#     # Set paths for the wind field data and world coordinate files
#     wind_field_path = "synthetic_data_generation/wind_fields/run_20241012_223409_random_field_flow_test/flow_field_data_100.npz"
#     world_coor_path = "synthetic_data_generation/output"

#     depth_image = 'synthetic_data_generation/output/distance_to_camera_0002.npy'

#     # Initialize the WindFieldProcessor with the paths
#     processor = WindFieldProcessor(wind_field_path, world_coor_path)

#     # Load the wind field data
#     processor._load_wind_field_data()

#     # Load world coordinates for a specific index (e.g., '0001')
#     idx = 2
#     processor._load_world_coordinates(idx)

#     # Define the movement vector (e.g., 1m forward in x, 1m upward in z)
#     movement_vector = np.array([0.0, 0.0, 1.0])

#     # Define 'edges' as a list of pixel coordinates (list of (y, x) tuples)
#     # You need to provide your own method or data to define 'edges'
#     # For this example, let's assume 'edges' is obtained from an edge detection function
#     # Replace 'your_edge_detection_function' with your actual edge detection function


#     # Get the list of edge pixels
#     # edges = extract_upper_edges_noconnect(processor.world_coordinates_image)
#     edges = extract_upper_edges_noconnect(depth_image)

#     # Process only the top-edge pixels
#     vertical_wind_image = processor.process_top_edge_pixels(movement_vector, edges)

#     # Now vertical_wind_image contains the vertical wind component at the edge pixels
#     # You can save or visualize this image as needed
#     idx_str  = str(idx).zfill(4)
#     output_path = os.path.join(world_coor_path, f"vertical_wind_image_edges_{idx_str}.npy")
#     np.save(output_path, vertical_wind_image)
#     print(f"Vertical wind image saved to {output_path}")

# if __name__ == "__main__":
#     main()