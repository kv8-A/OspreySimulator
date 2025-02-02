import os
import re

class DataPreparer:
    def __init__(self, data_dir):
        """
        Args:
            data_dir (str): Directory containing the synthetic data environments.
        """
        self.data_dir = data_dir
        self.depth_image_paths = []
        self.vertical_wind_image_paths = []
        self.wind_vector_paths = []

    def prepare_data_lists(self, environment_list=['TU_Delft', 'Random_world']):
        """
        Prepares the data lists by traversing the data directory and collecting depth image paths,
        vertical wind image paths, and wind velocity values.

        Returns:
            Tuple (list, list, list): Depth image paths, vertical wind image paths, wind velocities.
        """
        # Traverse through both directories (Random_world_test and TU_Delft)
        for environment in environment_list:
        # for environment in ['Random_world_test', 'TU_Delft', 'Random_world']:
            env_path = os.path.join(self.data_dir, environment)

            # Gather all depth images
            depth_images = sorted([f for f in os.listdir(env_path) if f.startswith('distance_to_camera') and f.endswith('.npy')])

            for depth_image in depth_images:
                # Get the corresponding ID from the depth image (e.g., 0001 from distance_to_camera_0001.npy)
                depth_id = re.search(r'(\d+)', depth_image).group(1)

                # Find all matching vertical wind images for this depth image
                vertical_wind_images = sorted([f for f in os.listdir(env_path) if f.startswith(f'vertical_wind_image_edges_{depth_id}') and f.endswith('.npy')])

                # Find all matching wind vector files for this depth image
                wind_vector_files = sorted([f for f in os.listdir(env_path) if f.startswith(f'wind_vector_to_drone_body_{depth_id}') and f.endswith('.npy')])

                # Pair vertical wind images with wind vector files based on 'v_X_X' in filenames
                for vertical_wind_image in vertical_wind_images:
                    # Extract the wind condition identifier (e.g., 'v_1_0')
                    wind_condition = re.search(r'v_(\d+_\d+)', vertical_wind_image).group(1)

                    # Find the corresponding wind vector file
                    matching_wind_vector_file = [f for f in wind_vector_files if f'v_{wind_condition}' in f]
                    if matching_wind_vector_file:
                        matching_wind_vector_file = matching_wind_vector_file[0]

                        # Append the paths to the lists
                        self.depth_image_paths.append(os.path.join(env_path, depth_image))
                        self.vertical_wind_image_paths.append(os.path.join(env_path, vertical_wind_image))
                        self.wind_vector_paths.append(os.path.join(env_path, matching_wind_vector_file))

    def get_data_lists(self,environment_list=['TU_Delft', 'Random_world']):
        """
        Returns the prepared data lists.

        Returns:
            Tuple (list, list, list): Depth image paths, vertical wind image paths, wind velocities.
        """
        if not self.depth_image_paths:
            self.prepare_data_lists(environment_list=environment_list)
        return self.depth_image_paths, self.vertical_wind_image_paths, self.wind_vector_paths

if __name__ == '__main__':
    data_dir = 'data'
    data_preparer = DataPreparer(data_dir)
    depth_image_paths, vertical_wind_image_paths, wind_velocities = data_preparer.get_data_lists()

    print(f"Number of depth images: {len(depth_image_paths)}")
    print(f"Number of vertical wind images: {len(vertical_wind_image_paths)}")
    print(f"Number of wind velocities: {len(wind_velocities)}")

    # sample test 
    print(depth_image_paths[212])
    print(vertical_wind_image_paths[212])
    print(wind_velocities[212])
    print(type(wind_velocities[212]))
    print(depth_image_paths[420])
    print(vertical_wind_image_paths[420])
    print(wind_velocities[420])
    import numpy as np
    print(np.load(wind_velocities[420]).shape)

