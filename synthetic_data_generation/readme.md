# Synthetic Data Generation

This repository is designed to generate synthetic data for training neural networks to estimate updraft locations. The dataset includes depth images, point clouds, camera parameters, and interpolated vertical wind fields.

Project Structure
```
synthetic_data_generation/
├── output/                     # Folder containing generated synthetic data
├── wind_fields/                # Folder storing wind field data for interpolation
├── utils/                      # Utility scripts for various data processing tasks
├── synthetic_data_gen.py       # Script for generating synthetic environments
├── get_coordinates.py          # Script to convert point cloud data to structured image files
├── get_soaring_spots.py        # Script to compute vertical wind images per wind field
├── synthetic_data_main.py      # High-level script integrating the full pipeline (excluding synthetic_data_gen.py for now)
└── README.md                   # This file
```

## Assigning Updraft Regions

The updraft assignment is based on Inverse Distance Weighting (IDW) interpolation. This method calculates the interpolated wind vector as a weighted average of the nearest wind vectors, where the weights are inversely proportional to the distance from the query point. This ensures that nearby wind vectors have a stronger influence on the interpolated value.


## Full synthetic data generation flow:

1. Run synthetic_data_gen.py
    - Inputs:
        Enviromnet
    - Outputs needed:
        - Depth images
        - Point cloud images
        - Camera parameters

2. Run get_coordinates.py -> reshapes point cloud to image file.
    - inputs:
        - Camera parameters
        - Depth image
        - Point cloud image
    - Output reshaped point cloud files

3. Run get_soaring_spots.py
    - input:
        - Multiple windfields for a single environment
        - Reshaped point cloud files
        - depth images
    - Ouput:
        - Vertical wind image per windfield 

4. Copy data for neural network:
    - input/output: 
        - depth images
        - vertical wind image

## High-Level Execution

All the above steps are integrated into `synthetic_data_main.py`. However, `synthetic_data_gen.py` is currently excluded due to Python environment conflicts. Until this issue is resolved, it is essential to manually copy the generated output from synthetic_data_gen/output/ to the main processing pipeline.

## Notes

- Ensure that the output/ folder contains all necessary data before running subsequent scripts.

- Wind field data should be structured properly to match the environment setup.

- IDW interpolation ensures that updraft locations are assigned based on realistic aerodynamics.

- This structured approach facilitates seamless synthetic data generation, ensuring consistency across all training datasets.

