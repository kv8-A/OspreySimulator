
import matplotlib.pyplot as plt
import numpy as np




for i in range(1, 40):
    # depth_input_file = f'synthetic_data_generation/TU_Delft/nn_data/distance_to_camera_{str(i).zfill(4)}.npy'
    # depth_input_file = f'inference_results/distance_to_camera_{str(i).zfill(4)}.npy'
    depth_input_file = f'data/Random_world_test/distance_to_camera_{str(i).zfill(4)}.npy'
    wind_str = "all"
    vertical_wind_image = np.load(f'inference_bottleneck/prediction_{i}.npy')

    distance_image = np.load(depth_input_file)
    distance_image = np.nan_to_num(distance_image, nan=0.0, posinf=0.0, neginf=0)

    
    # Create an RGBA version of the distance image
    background_rgba = plt.cm.gray(distance_image)
    # 'background_rgba' is an (H, W, 4) array with RGBA values

    # Initialize an overlay array with zeros (transparent)
    overlay = np.zeros_like(background_rgba)  # Same shape as 'background_rgba'

    # Define masks for soaring and downdraft spots
    soaring_mask = vertical_wind_image > 0.1
    downdraft_mask = vertical_wind_image < -0.1
    between_mask = (vertical_wind_image >= -1) & (vertical_wind_image <= 1)

    # Set the color for soaring spots (green) with full opacity
    overlay[soaring_mask] = [0, 1, 0, 1]  # [R, G, B, A]

    # Set the color for downdraft spots (red) with full opacity
    overlay[downdraft_mask] = [1, 0, 0, 1]  # [R, G, B, A]

    # Set the color for values between -1 and 1 (orange) with full opacity
    # overlay[between_mask] = [1, 0.65, 0, 1]  # [R, G, B, A]

    # Combine the background and overlay using alpha blending
    combined_image = background_rgba.copy()
    alpha_overlay = overlay[..., 3]

    # Where alpha is 1 (fully opaque), replace the background pixel with the overlay pixel
    mask = alpha_overlay == 1
    combined_image[mask] = overlay[mask]

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
    plt.savefig(f'inference_bottleneck/vertical_wind_overlay_{str(i).zfill(4)}_v_{wind_str}.png')