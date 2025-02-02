import os
import glob
import numpy as np
# from synthetic_data_gen import SyntheticDataRecorder
from get_coordinates import PointCloudProcessor
from get_soaring_spots import WindFieldProcessor
# from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
import time
import subprocess
import shutil


class SyntheticDataGeneration:
    def __init__(self, environment: str, output_base_dir: str):
        """
        Initializes the SyntheticDataGeneration pipeline with the environment and output directory.

        :param environment: Name of the environment (e.g., 'TU Delft', 'Random_world_test', etc.).
        :param output_base_dir: Base directory where output data will be saved.
        """
        self.environment = environment
        self.output_base_dir = output_base_dir
        self.env_output_dir = os.path.join(output_base_dir, environment)
        os.makedirs(self.env_output_dir, exist_ok=True)

    # def run_synthetic_data_generation(self, number_of_frames: int) -> None:
    #     """
    #     Generates synthetic data including depth images, point clouds, and camera parameters.

    #     :param number_of_frames: Number of frames to record.
    #     """
    #     print(f"Running synthetic data generation for environment: {self.environment}")
    #     recorder.output_dir = os.path.join(self.env_output_dir, "output")
    #     os.makedirs(recorder.output_dir, exist_ok=True)
    #     time.sleep(5)
    #     recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS[self.environment])
    #     recorder.output_dir = os.path.join(self.env_output_dir, "output")
    #     os.makedirs(recorder.output_dir, exist_ok=True)
    #     recorder.record(number_of_frames=number_of_frames)

    def run_point_cloud_processing(self, start_idx: int, end_idx: int) -> None:
        """
        Processes and reshapes the point cloud to image files.

        :param start_idx: Starting index for processing.
        :param end_idx: Ending index for processing.
        """
        print(f"Running point cloud processing for environment: {self.environment}")
        base_path = os.path.join(self.env_output_dir, "output")
        processor = PointCloudProcessor(base_path)
        processor.process_and_save_images(start_idx, end_idx)

    def run_wind_field_processing(self, movement_vector: np.ndarray) -> None:
        """
        Processes wind fields for a specific environment.

        :param movement_vector: Movement vector to apply to each coordinate.
        """
        print(f"Running wind field processing for environment: {self.environment}")
        world_coor_path = os.path.join(self.env_output_dir, "output")
        output_path = os.path.join(self.env_output_dir, "processed_output")
        os.makedirs(output_path, exist_ok=True)

        # Get all wind field files for the specified environment using glob
        wind_field_base_path = os.path.join("synthetic_data_generation", "wind_fields", self.environment)
        wind_field_files = glob.glob(os.path.join(wind_field_base_path, "**", "*.npz"), recursive=True)

        for wind_field_path in wind_field_files:
            processor = WindFieldProcessor(wind_field_path, world_coor_path, output_path)
            processor._load_wind_field_data()
            processor.process_all_data(movement_vector)

    def copy_data_for_neural_network(self) -> None:
        """
        Copies necessary data for neural network input.
        """
        print(f"Copying data for neural network for environment: {self.environment}")
        source_depth_path = os.path.join(self.env_output_dir, "output")
        source_wind_path = os.path.join(self.env_output_dir, "processed_output")
        # source_wind_vector = os.path.join(self.env_output_dir, "processed_output")
        nn_output_dir = os.path.join(self.env_output_dir, "nn_data")
        os.makedirs(nn_output_dir, exist_ok=True)

        # Copy depth images and vertical wind images to nn_data directory
        for file_name in os.listdir(source_depth_path):
            if file_name.startswith("distance_to_camera_") and file_name.endswith(".npy"):
                source_file = os.path.join(source_depth_path, file_name)
                target_file = os.path.join(nn_output_dir, file_name)
                np.save(target_file, np.load(source_file))

        for file_name in os.listdir(source_wind_path):
            # Copy vertical wind images 
            if file_name.startswith("vertical_wind_image_edges_") and file_name.endswith(".npy"):
                source_file = os.path.join(source_wind_path, file_name)
                target_file = os.path.join(nn_output_dir, file_name)
                np.save(target_file, np.load(source_file))
            # Copy wind vectors to drone body frame files
            elif file_name.startswith("wind_vector_to_drone_body_") and file_name.endswith(".npy"):
                source_file = os.path.join(source_wind_path, file_name)
                target_file = os.path.join(nn_output_dir, file_name)
                np.save(target_file, np.load(source_file))


def main():
    # environment = "Random_world_test"
    environment = "Random_world"
    # environment = "TU_Delft"
    output_base_dir = "synthetic_data_generation"
    number_of_frames = 4
    start_idx = 1
    end_idx = 400
    movement_vector = np.array([0.0, 0.0, 2.0])

    # Initialize and run the full synthetic data generation flow
    data_generator = SyntheticDataGeneration(environment, output_base_dir)
    # data_generator.run_synthetic_data_generation(number_of_frames)
    # TODO For now copy output manually
    # TODO copy output from synthetic_data_gen.py to synthetic_data_generation/{environment}/output
    # subprocess.run("ISAACSIM_PYTHON synthetic_data_generation/synthetic_data_gen.py",shell=True,executable='/bin/bash')
    # time.sleep(10) # Wait for the data to be saved
    data_generator.run_point_cloud_processing(start_idx, end_idx)
    data_generator.run_wind_field_processing(movement_vector)
    data_generator.copy_data_for_neural_network()


if __name__ == "__main__":
    main()
