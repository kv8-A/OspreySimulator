import matplotlib.pyplot as plt
import numpy as np
import os

# Directory paths
inference_dir = 'inference_bottleneck'
data_dir = 'data/Random_world_test'

# Iterate through predictions
for i in range(1, 228):
    # Read metadata file
    metadata_file = os.path.join(inference_dir, f'prediction_{i}_metadata.txt')
    if not os.path.exists(metadata_file):
        print(f"Metadata file not found: {metadata_file}")
        continue

    with open(metadata_file, 'r') as f:
        lines = f.readlines()
        depth_input_file = lines[0].split(': ')[1].strip()  # Extract depth image path
        wind_vector_file = lines[1].split(': ')[1].strip()  # Extract wind vector path (if needed)

    # Load depth image and prediction
    if not os.path.exists(depth_input_file):
        print(f"Depth image file not found: {depth_input_file}")
        continue

    vertical_wind_image = np.load(os.path.join(inference_dir, f'prediction_{i}.npy'))
    distance_image = np.load(depth_input_file)
    distance_image = np.nan_to_num(distance_image, nan=0.0, posinf=0.0, neginf=0)

    # Create an RGBA version of the distance image
    background_rgba = plt.cm.gray(distance_image)

    # Initialize an overlay array with zeros (transparent)
    overlay = np.zeros_like(background_rgba)  # Same shape as 'background_rgba'

    # Define masks for soaring and downdraft spots
    soaring_mask = vertical_wind_image > 0.1
    downdraft_mask = vertical_wind_image < -0.1

    # Set the color for soaring spots (green) with full opacity
    overlay[soaring_mask] = [0, 1, 0, 1]  # [R, G, B, A]

    # Set the color for downdraft spots (red) with full opacity
    overlay[downdraft_mask] = [1, 0, 0, 1]  # [R, G, B, A]

    # Combine the background and overlay using alpha blending
    combined_image = background_rgba.copy()
    alpha_overlay = overlay[..., 3]

    # Where alpha is 1 (fully opaque), replace the background pixel with the overlay pixel
    mask = alpha_overlay == 1
    combined_image[mask] = overlay[mask]

    # Visualize and save the results
    plt.figure(figsize=(15, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(distance_image, cmap='gray')
    plt.title('Normalized Distance Image')
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.imshow(combined_image)
    plt.title('Vertical Wind Velocity Overlay')
    plt.axis('off')

    plt.tight_layout()
    output_image_path = os.path.join(inference_dir, f'vertical_wind_overlay_{str(i).zfill(4)}.png')
    plt.savefig(output_image_path)
    print(f"Saved visualization to {output_image_path}")
