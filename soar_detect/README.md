# Soar_Detect

Soar_Detect is a project for wind vector detection and inference using deep learning. The project leverages synthetic data to train and evaluate a U-Net–based neural network for predicting wind conditions based on depth and vertical wind images. The repository contains code for data preparation, training, evaluation, and inference.

## Table of Contents

- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [Usage](#usage)
  - [Data Preparation](#data-preparation)
  - [Training](#training)
  - [Evaluation/Inference](#evaluationinference)
- [File Descriptions](#file-descriptions)
- [License](#license)

## Project Structure
```
Soar_Detect/
├── checkpoints/               # Folder for storing model checkpoints
├── data/                      # Folder containing the synthetic data environments
├── inference_results/         # (Optional) Folder where inference outputs and metadata are saved
├── utils/                     # Utility scripts (if any additional helper scripts exist)
├── data_preperation.py        # Script to prepare data lists from the synthetic environments
├── eval.py                    # Script for running inference on trained models
├── neural_net.py              # Implementation of the U-Net neural network model
├── train.py                   # Training script for the U-Net model
├── visualizaiton.py           # Script for visualizing training results/inference outputs
└── README.md                  # This file
```
## Requirements

- Python 3.7+
- [PyTorch](https://pytorch.org/) (for model training and inference)
- NumPy
- Other dependencies (e.g., PIL) as required by your code

Install required Python packages:
```bash
pip install torch torchvision numpy pillow
```
## Usage 
### Data Preparations

The data_preperation.py script handles the creation of data lists by scanning the provided data directory for:

- Depth images (e.g., distance_to_camera_*.npy)
- Vertical wind images (e.g., vertical_wind_image_edges_*.npy)
- Wind vector files (e.g., wind_vector_to_drone_body_*.npy)

To test data preparation, run:

`python data_preperation.py`

This will print the number of files found for each category and some sample paths.

### Training

The train.py script trains the U-Net model using the prepared data. Training uses an MSE loss between the predicted vertical wind images and the ground truth.

To start training, run:

`python train.py`

Checkpoints will be saved to the checkpoints/ folder after each epoch.

### Evaluation / Inference

The eval.py script loads a trained model (e.g., a checkpoint from checkpoints/) and performs inference on a set of images. Predictions and associated metadata are saved in a specified directory.

Run inference by executing:

`python eval.py`

Make sure to adjust the data directory paths and checkpoint paths inside the script if needed.

### Visualization

Use the visualizaiton.py script to visualize results (training loss curves, inference outputs, etc.). Adjust this script as needed based on your preferred visualization methods.

## File Descriptions

- checkpoints/: Contains saved model states (e.g., model_epoch_25.pth). These are used for resuming training or running inference.

- data/: Contains the synthetic data environments with subdirectories (such as TU_Delft, Random_world, or tu_delft_inference).

- inference_results/: When running eval.py, predictions (as .npy files) and metadata files are saved here.

- utils/: Additional helper functions and scripts (if any) that support data processing or other common tasks.

- data_preperation.py: Defines the DataPreparer class that scans the data directory, pairs depth images, vertical wind images, and wind vector files based on naming conventions.

- eval.py: Loads the trained U-Net model, sets up a PyTorch DataLoader for the test dataset, runs inference, and saves the predictions along with metadata.

- neural_net.py: Contains the U-Net implementation with custom modifications to incorporate wind vector data into the network.

- train.py: Sets up the dataset, DataLoader, loss function, optimizer, and training loop. Also saves model checkpoints.

- visualizaiton.py: Provides functions to visualize training and inference results. (Customize this file as needed.)