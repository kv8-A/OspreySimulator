import matplotlib.pyplot as plt
import numpy as np
import os

# Directory paths
# inference_dir = 'inference_bottleneck'
# data_dir = 'data/Random_world_test'
inference_dir = 'data/tu_delft_inference'
data_dir = 'data/tu_delft_inference'

# Generate a single combined visualization
i = 3  # Example index for which to create the combined image

# Read metadata file
metadata_file = os.path.join(inference_dir, f'prediction_{i}_metadata.txt')
if not os.path.exists(metadata_file):
    raise FileNotFoundError(f"Metadata file not found: {metadata_file}")

with open(metadata_file, 'r') as f:
    lines = f.readlines()
    depth_input_file = lines[0].split(': ')[1].strip()  # Extract depth image path

# Load depth image and prediction
if not os.path.exists(depth_input_file):
    raise FileNotFoundError(f"Depth image file not found: {depth_input_file}")

vertical_wind_image = np.load(os.path.join(inference_dir, f'prediction_{i}.npy'))
distance_image = np.load(depth_input_file)
distance_image = np.nan_to_num(distance_image, nan=0.0, posinf=0.0, neginf=0)

# Normalize the depth image for visualization purposes (optional)
distance_image_scaled = distance_image / (np.max(distance_image) + 1e-6)

# Create an RGB base using the raw depth image
# Use a grayscale colormap for depth values, but keep as RGB
background_rgb = plt.cm.gray(distance_image_scaled)[..., :3]

# Initialize an overlay array with zeros (transparent)
overlay = np.zeros_like(background_rgb)  # Same shape as 'background_rgb'

# Define masks for soaring and downdraft spots
soaring_mask = vertical_wind_image > 0.1
downdraft_mask = vertical_wind_image < -0.1

# Set the color for soaring spots (green)
overlay[soaring_mask] = [0, 1, 0]  # [R, G, B]
# Set the color for downdraft spots (red)
overlay[downdraft_mask] = [1, 0, 0]  # [R, G, B]

# Combine the depth image and overlay
alpha = 0.5  # Adjust transparency of the overlay
combined_image = (1 - alpha) * background_rgb + alpha * overlay

# Plotting the combined image
plt.figure(figsize=(8, 6))
plt.imshow(combined_image)
plt.title("Depth Image with Vertical Wind Prediction")
plt.axis("off")
plt.tight_layout()

# Save the combined image
output_path = os.path.join(inference_dir, f'combined_visualization_no_contours_{str(i).zfill(4)}.png')
plt.savefig(output_path)
print(f"Saved combined visualization to {output_path}")

# Show the figure (optional)
plt.show()
