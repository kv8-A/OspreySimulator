import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
import os
from PIL import Image
import numpy as np

from neural_net import UNet
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

def train_model(model, dataloader, criterion, optimizer, num_epochs=25, checkpoint_dir='checkpoints_bottleneck'):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0
        for batch in dataloader:
            depth_images = batch['depth_image'].to(device)        # Shape: (B, 1, H, W)
            wind_vectors = batch['wind_vector'].to(device)        # Shape: (B, 2)
            vertical_wind_images = batch['vertical_wind_image'].to(device)  # Shape: (B, 1, H, W)

            optimizer.zero_grad()
            outputs = model(depth_images, wind_vectors)
            loss = criterion(outputs, vertical_wind_images)
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()

        avg_loss = epoch_loss / len(dataloader)
        print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}")

        # Save model checkpoint at the end of each epoch
        checkpoint_path = os.path.join(checkpoint_dir, f'model_epoch_{epoch+1}.pth')
        torch.save(model.state_dict(), checkpoint_path)
        print(f"Checkpoint saved at {checkpoint_path}")

    print("Training complete.")

# Example lists (replace with your actual data)
data_dir = 'data'
data_preparer = DataPreparer(data_dir)
depth_image_paths, vertical_wind_image_paths, wind_velocities = data_preparer.get_data_lists()

# Create Dataset and DataLoader
dataset = WindDataset(depth_image_paths, vertical_wind_image_paths, wind_velocities)
# dataloader = DataLoader(dataset, batch_size=4, shuffle=True, num_workers=2)
dataloader = DataLoader(dataset, batch_size=2, shuffle=True, num_workers=2)

# Initialize model, criterion, and optimizer
model = UNet(n_channels=1, n_classes=1)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=1e-4)

# Train the model
train_model(model, dataloader, criterion, optimizer, num_epochs=25)

# After training, you can perform inference as follows
# model.eval()
# with torch.no_grad():
#     for batch in dataloader:
#         depth_images = batch['depth_image'].to(device)
#         wind_velocities = batch['wind_velocity'].to(device)
#         outputs = model(depth_images, wind_velocities)
#         # Process outputs as needed
