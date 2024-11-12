# Readme for the Synthetic Data Generation













# Assigning updraft regions 


the interpolated wind vector in the code I provided is essentially a weighted average of the nearest wind vectors, where the weights are inversely proportional to the distance from the query point. This method is known as inverse distance weighting (IDW) interpolation. So, the interpolated wind vector is not a simple arithmetic average but a weighted one that accounts for the proximity of each neighbor.



# Full synthetic data generation flow:

1. Run synthetic_data_gen.py
    - Inputs:
        Enviromnet
    - Outputs needed:
        - Depth images
        - Point Cloud images
        - Camera parameters

2. Run get_coordinates.py -> reshapes point cloud to image file.
    - inputs:
        - camera parameters
        - depth image
        - point cloud image
    - Output reshaped point cloud files

3. Run get_soaring_spots.py
    - input:
        - Multiple windfields for 1 environment !!!
        - Reshaped point cloud files
        - depth images
    - Ouput:
        - Vertical wind image PER WINDFIELD 

4. Copy data for neural network:
    - input/output: 
        - depth images
        - vertical wind image


All of the above has been summarized in synthetic_data_main. 

Currently synthetic_data_main.py is without the syntetic_data_gen because the python enviromnets clash. Very important for now is to copy the generated output in `synthetic_data_gen` in the 'output' folder to 