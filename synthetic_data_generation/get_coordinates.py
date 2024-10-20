import numpy as np 
import os 
import json

class PointCloudProcessor:
    def __init__(self, base_path: str):
        """
        Initialize the PointCloudProcessor with the base path where images and camera parameters are stored.
        
        Parameters:
        - base_path: The directory path where the images, point clouds, and camera pose JSONs are stored.
        """
        self.base_path = base_path

    def depth_to_world_coordinates(self, image_distance_from_camera: np.ndarray, point_cloud: np.ndarray):
        """
        Reshape the point cloud to match the shape of the depth image (H, W, 3).
        
        Parameters:
        - image_distance_from_camera: 2D numpy array (H, W) containing depth data.
        - point_cloud: 2D numpy array (N, 3) containing point cloud coordinates.
        
        Returns:
        - reshaped_point_cloud: 3D numpy array (H, W, 3) containing the reshaped point cloud,
                                with np.nan for pixels where no corresponding points exist.
        """
        H, W = image_distance_from_camera.shape  # Get height and width from the depth image
        reshaped_point_cloud = np.full((H, W, 3), np.nan)  # Initialize with NaN values

        point_cloud_index = 0  # Counter for iterating through the point cloud

        # Iterate through each pixel in the depth image
        for v in range(H):
            for u in range(W):
                # Check if the depth value is finite (not infinity)
                if np.isfinite(image_distance_from_camera[v, u]):
                    # Assign the corresponding point cloud coordinates to the reshaped array
                    reshaped_point_cloud[v, u] = point_cloud[point_cloud_index]
                    point_cloud_index += 1

        return reshaped_point_cloud

    def calculate_distance_from_camera(self, reshaped_point_cloud: np.ndarray, camera_position: np.ndarray):
        """
        Calculate the Euclidean distance between the camera position and each point in the reshaped point cloud.
        
        Parameters:
        - reshaped_point_cloud: 3D numpy array (H, W, 3) containing the reshaped point cloud.
        - camera_position: Tuple (x, y, z) representing the camera's position in 3D space.
        
        Returns:
        - distance_image: 2D numpy array (H, W) containing the calculated distances,
                          with np.inf where there are no valid points.
        """
        H, W, _ = reshaped_point_cloud.shape  # Get height and width from the reshaped point cloud
        distance_image = np.full((H, W), np.inf)  # Initialize distance image with np.inf

        # Iterate through each pixel in the reshaped point cloud
        for v in range(H):
            for u in range(W):
                point = reshaped_point_cloud[v, u]
                # Check if the point is valid (not NaN)
                if not np.isnan(point).any():
                    # Calculate the Euclidean distance between the camera position and the point
                    distance = np.linalg.norm(point - camera_position)  # Euclidean distance formula
                    distance_image[v, u] = distance  # Store the distance in the distance image

        return distance_image

    def load_camera_parameters(self, idx: str):
        """
        Load the camera position from the corresponding JSON file.
        
        Parameters:
        - idx: The index of the file to load, e.g., "0001" or "0002".
        
        Returns:
        - camera_position: Numpy array containing the camera position (x, y, z).
        """
        json_file_path = os.path.join(self.base_path, f"camera_pose_{idx}.json")
        with open(json_file_path, 'r') as f:
            camera_data = json.load(f)
        
        return np.array(camera_data['position'])

    def load_data_for_multiple_images(self, start_idx: int, end_idx: int):
        """
        Load depth images, point clouds, and camera parameters from a folder.
        
        Parameters:
        - start_idx: The starting index for the files (e.g., 1 for 0001).
        - end_idx: The ending index for the files.
        
        Returns:
        - image_data_list: List of 2D numpy arrays representing depth images (H, W).
        - point_cloud_list: List of 2D numpy arrays representing point cloud coordinates (N, 3).
        - camera_positions_list: List of numpy arrays representing camera positions (x, y, z).
        """
        image_data_list = []
        point_cloud_list = []
        camera_positions_list = []
        # image_paths = []

        for i in range(start_idx, end_idx + 1):
            idx_str = str(i).zfill(4)  # Pad the index to 4 digits
            
            # Load the distance image and point cloud
            dist_image_path = os.path.join(self.base_path, f"distance_to_camera_{idx_str}.npy")
            point_cloud_path = os.path.join(self.base_path, f"pointcloud_{idx_str}.npy")
            
            image_distance_from_camera = np.load(dist_image_path)
            point_cloud = np.load(point_cloud_path)
            
            # Load the camera position
            camera_position = self.load_camera_parameters(idx_str)
            
            # Append loaded data to the respective lists
            image_data_list.append(image_distance_from_camera)
            point_cloud_list.append(point_cloud)
            camera_positions_list.append(camera_position)
            # image_paths.append(dist_image_path)

        return image_data_list, point_cloud_list, camera_positions_list

    def process_and_save_images(self, start_idx: int, end_idx: int):
        """
        Process multiple images and point clouds by reshaping and calculating the distances for each,
        then save the results as .npy files in the output folder.
        
        Parameters:
        - start_idx: The starting index for the files (e.g., 1 for 0001).
        - end_idx: The ending index for the files.
        """
        # Load data from the folder
        image_data_list, point_cloud_list, camera_positions_list = self.load_data_for_multiple_images(start_idx, end_idx)

        for i, (image_distance_from_camera, point_cloud, camera_position) in enumerate(zip(image_data_list, point_cloud_list, camera_positions_list), start=start_idx):
            idx_str = str(i).zfill(4)
            
            # Process the images
            reshaped_point_cloud = self.depth_to_world_coordinates(image_distance_from_camera, point_cloud)
            distance_image = self.calculate_distance_from_camera(reshaped_point_cloud, camera_position)
            
            # Save the reshaped point cloud and distance image as .npy
            reshaped_point_cloud_path = os.path.join(self.base_path, f"reshaped_point_cloud_{idx_str}.npy")
            distance_image_path = os.path.join(self.base_path, f"distance_image_FROMWORLDCOOR{idx_str}.npy")
            
            np.save(reshaped_point_cloud_path, reshaped_point_cloud)
            np.save(distance_image_path, distance_image)

            print(f"Saved reshaped point cloud: {reshaped_point_cloud_path}")
            print(f"Saved distance image: {distance_image_path}")


# Example usage
base_path = "synthetic_data_generation/output"  # Folder where your files are stored
processor = PointCloudProcessor(base_path)

# Process and save multiple images, e.g., from 0001 to 0003
start_idx = 1
end_idx = 4

# img, point, camera = processor.load_data_for_multiple_images(start_idx, end_idx)

# print(img)
# print(point)
# print(camera)

processor.process_and_save_images(start_idx, end_idx)