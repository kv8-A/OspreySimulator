# import matplotlib.pyplot as plt
# import numpy as np
# from edge_detection import process_distance_to_camera_image, extract_upper_edges_noconnect
# import cv2


# depth_input_file = 'synthetic_data_generation/output/distance_to_camera_0002.npy'

# vertical_wind_image = np.load('synthetic_data_generation/output/vertical_wind_image_edges_0002.npy')

# edges, distance_normalized, contours = process_distance_to_camera_image(depth_input_file)
# # contour_image = np.zeros_like(edges)

# edges_upper = extract_upper_edges_noconnect(depth_input_file)
# color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
#     # Draw the upper edges in red
# for edge in edges_upper:
#     color_edges[edge[1], edge[0]] = (255, 0, 0)  # Red color in BGR format

# # cv2.drawContours(contour_image, contours, -1, (255), 1)
# assert distance_normalized.shape == vertical_wind_image.shape, "Image shapes do not match."

# # Create an RGBA version of the distance image
# background_rgba = plt.cm.gray(distance_normalized)
# # 'background_rgba' is an (H, W, 4) array with RGBA values

# # Initialize an overlay array with zeros (transparent)
# overlay = np.zeros_like(background_rgba)  # Same shape as 'background_rgba'

# # Define masks for soaring and downdraft spots
# soaring_mask = vertical_wind_image > 1
# downdraft_mask = vertical_wind_image < -1

# # Set the color for soaring spots (green) with full opacity
# overlay[soaring_mask] = [0, 1, 0, 1]  # [R, G, B, A]

# # Set the color for downdraft spots (red) with full opacity
# overlay[downdraft_mask] = [1, 0, 0, 1]  # [R, G, B, A]

# # Combine the background and overlay using alpha blending
# combined_image = background_rgba.copy()
# alpha_overlay = overlay[..., 3]

# # Where alpha is 1 (fully opaque), replace the background pixel with the overlay pixel
# mask = alpha_overlay == 1
# combined_image[mask] = overlay[mask]

# plt.figure(figsize=(15, 5))
# plt.subplot(1, 3, 1)
# plt.imshow(distance_normalized, cmap='gray')
# plt.title('Normalized Distance Image')
# plt.axis('off')

# plt.subplot(1, 3, 2)
# plt.imshow(edges, cmap='gray')
# plt.title('Edges Detected')
# plt.axis('off')

# plt.subplot(1, 3, 3)
# plt.imshow(combined_image)
# plt.title('Vertical Wind Velocity Overlay')
# plt.axis('off')

# plt.tight_layout()
# plt.savefig('synthetic_data_generation/output/vertical_wind_overlay.png')

import matplotlib.pyplot as plt
import numpy as np
from edge_detection import process_distance_to_camera_image, extract_upper_edges_noconnect
import cv2


for i in range(1, 2):
    i = 84
    depth_input_file = f'synthetic_data_generation/TU_Delft/nn_data/distance_to_camera_{str(i).zfill(4)}.npy'
    # depth_input_file = f'synthetic_data_generation/Random_world_test/nn_data/distance_to_camera_{str(i).zfill(4)}.npy'
    wind_str = "10_0"
    vertical_wind_image = np.load(f'synthetic_data_generation/TU_Delft/nn_data/vertical_wind_image_edges_{str(i).zfill(4)}_v_{wind_str}.npy')
    # vertical_wind_image = np.load(f'synthetic_data_generation/Random_world_test/nn_data/vertical_wind_image_edges_{str(i).zfill(4)}_v_{wind_str}.npy')

    edges, distance_normalized, contours = process_distance_to_camera_image(depth_input_file)

    edges_upper = extract_upper_edges_noconnect(depth_input_file)
    color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    # Draw the upper edges in red
    for edge in edges_upper:
        color_edges[edge[1], edge[0]] = (255, 0, 0)  # Red color in BGR format

    assert distance_normalized.shape == vertical_wind_image.shape, "Image shapes do not match."

    # Create an RGBA version of the distance image
    background_rgba = plt.cm.gray(distance_normalized)
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

    plt.figure(figsize=(14, 4))
    plt.subplot(1, 3, 1)
    plt.imshow(distance_normalized, cmap='gray')
    plt.title('Distance Image', fontsize=20)
    plt.axis('off')

    plt.subplot(1, 3, 2)
    plt.imshow(edges, cmap='gray')
    plt.title('Detected Edges', fontsize=20)
    plt.axis('off')

    plt.subplot(1, 3, 3)
    plt.imshow(combined_image)
    plt.title('Labeled Vertical Wind Velocity', fontsize=20)
    plt.axis('off')

    plt.tight_layout()
    # plt.savefig(f'synthetic_data_generation/Random_world_test/validation/vertical_wind_overlay_{str(i).zfill(4)}_v_{wind_str}.png')
    plt.savefig(f'synthetic_data_generation/TU_Delft/validation/vertical_wind_overlay_{str(i).zfill(4)}_v_{wind_str}.png')