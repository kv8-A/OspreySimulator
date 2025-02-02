import torch
import os 
import numpy as np

from torch.utils.data import DataLoader

from neural_net import UNet
# from train import WindDataset
from torch.utils.data import Dataset, DataLoader
from data_preparation import DataPreparer


class WindDataset(Dataset):
    def __init__(self, depth_image_paths, vertical_wind_image_paths, wind_vector_paths, transform=None):
        """
        Args:
            depth_image_paths (list): File paths to depth images (npy).
            vertical_wind_image_paths (list): File paths to vertical wind images (npy).
            wind_vecotr_paths (list): File paths to wind vector files (npy).
            transform (callable, optional): Optional transform to apply to samples.
        """
        self.depth_image_paths = depth_image_paths
        self.vertical_wind_image_paths = vertical_wind_image_paths
        self.wind_vector_paths = wind_vector_paths
        self.transform = transform

    def __len__(self):
        return len(self.depth_image_paths)
    
    def get_depth_image_paths(self):
        return self.depth_image_paths
    
    def get_wind_vector_paths(self):
        return self.wind_vector_paths

    def __getitem__(self, idx):
        # Load depth and wind images from npy files
        depth_image = np.load(self.depth_image_paths[idx])
        vertical_wind_image = np.load(self.vertical_wind_image_paths[idx])

        # Handle NaN values in vertical wind images (e.g., replace NaN with zero)
        vertical_wind_image = np.nan_to_num(vertical_wind_image, nan=0.0)

        # Handle Inf values in depth images (replace Inf with a large value like 1e6)
        depth_image = np.where(np.isinf(depth_image), 1e6, depth_image)

        # Load wind vector (already transformed to drone's frame)
        wind_vector = np.load(self.wind_vector_paths[idx])  # Shape: (2,)
        wind_vector = torch.tensor(wind_vector, dtype=torch.float32)  # Shape: (2,)

        # Normalize the depth and vertical wind images to the range [0, 1]
        epsilon = 1e-6
        depth_image = depth_image.astype(np.float32) / (depth_image.max() + epsilon)
        # vertical_wind_image = vertical_wind_image.astype(np.float32) / (vertical_wind_image.max() + epsilon)

        # Normalize the depth and vertical wind images to the range [0, 1]
        # depth_image = depth_image.astype(np.float32) / depth_image.max()  # Normalize to [0, 1]
        # vertical_wind_image = vertical_wind_image.astype(np.float32) / vertical_wind_image.max()  # Normalize to [0, 1]

        # Add channel dimension (to convert to shape: (1, H, W))
        depth_image = depth_image[np.newaxis, ...]  # Shape: (1, H, W)
        vertical_wind_image = vertical_wind_image[np.newaxis, ...]  # Shape: (1, H, W)

        # Convert to tensors
        depth_image = torch.tensor(depth_image, dtype=torch.float32)
        vertical_wind_image = torch.tensor(vertical_wind_image, dtype=torch.float32)

        sample = {
            'depth_image': depth_image,             # Shape: (1, H, W)
            'wind_vector': wind_vector,            # Shape: (2,)
            'vertical_wind_image': vertical_wind_image # Shape: (1, H, W)
        }

        if self.transform:
            sample = self.transform(sample)

        return sample

# Load the trained model (assuming you saved the final checkpoint)
checkpoint_path = 'checkpoints_bottleneck/model_epoch_25.pth'
model = UNet(n_channels=1, n_classes=1)  # Ensure the model is initialized
model.load_state_dict(torch.load(checkpoint_path))
# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')
model = model.to(device)

# Set the model to evaluation mode
model.eval()

data_dir = 'data'  # Update with the appropriate directory for test/inference data
data_preparer = DataPreparer(data_dir)
# depth_image_paths, vertical_wind_image_paths, wind_vector_paths = data_preparer.get_data_lists(['Random_world_test'])
depth_image_paths, vertical_wind_image_paths, wind_vector_paths = data_preparer.get_data_lists(['tu_delft_inference'])

# Create dataset and dataloader for inference
inference_dataset = WindDataset(depth_image_paths, vertical_wind_image_paths, wind_vector_paths)
inference_dataloader = DataLoader(inference_dataset, batch_size=2, shuffle=False, num_workers=2)

# Create a directory to save predictions
# output_dir = 'inference_bottleneck'
output_dir = 'data/tu_delft_inference'
os.makedirs(output_dir, exist_ok=True)

# print(inference_dataloader)
# print(inference_dataset)
# Disable gradient calculations
with torch.no_grad():
    for i, batch in enumerate(inference_dataloader):
        depth_images = batch['depth_image'].to(device)        # Shape: (B, 1, H, W)
        wind_vectors = batch['wind_vector'].to(device) 

        # Perform inference
        predictions = model(depth_images, wind_vectors)

        # Process and save the predictions with metadata
        for j, prediction in enumerate(predictions):
            # Convert prediction tensor to CPU and save as a numpy file
            prediction_numpy = prediction.squeeze().cpu().numpy()

            # Get the corresponding paths for metadata
            depth_image_path = inference_dataset.get_depth_image_paths()[i * len(predictions) + j]
            wind_vector_path = inference_dataset.get_wind_vector_paths()[i * len(predictions) + j]

            # Define the save path for the prediction
            output_path = os.path.join(output_dir, f'prediction_{i * len(predictions) + j}.npy')

            # Save the prediction
            np.save(output_path, prediction_numpy)

            # Save metadata alongside the prediction
            metadata_path = os.path.join(output_dir, f'prediction_{i * len(predictions) + j}_metadata.txt')
            with open(metadata_path, 'w') as f:
                f.write(f"Depth Image Path: {depth_image_path}\n")
                f.write(f"Wind Vector Path: {wind_vector_path}\n")

            print(f'Saved prediction to {output_path}')
            print(f'Saved metadata to {metadata_path}')