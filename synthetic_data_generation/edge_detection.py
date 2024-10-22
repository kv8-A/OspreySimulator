import cv2
import numpy as np
import matplotlib.pyplot as plt
from typing import Union, List, Tuple

# Use non-interactive backend for headless environments
import matplotlib
matplotlib.use('Agg')

def process_distance_to_camera_image(image_input: Union[str,np.ndarray]) -> np.ndarray:
    """
    Load the distance_to_camera image and apply edge detection to identify upper borders.
    
    Args:
        image_path (str): Path to the npy distance-to-camera file.

    Returns:
        np.ndarray: Binary edge image.
    """
    if isinstance(image_input, str):
        # Load the distance-to-camera image
        distance_image = np.load(image_input)
    elif isinstance(image_input, np.ndarray):
        distance_image = image_input
    else:
        raise TypeError("Expected input to be str or np.ndarray")
    # Load the distance-to-camera image
    # distance_image= np.load(image_path)
    distance_image = np.nan_to_num(distance_image, nan=0.0, posinf=0.0, neginf=0)

    # Normalize for better visualization (optional)
    distance_normalized= cv2.normalize(distance_image, None, 0, 255, cv2.NORM_MINMAX)
    distance_normalized = np.uint8(distance_normalized)
    

    # Use edge detection (Canny)
    edges = cv2.Canny(distance_normalized, threshold1=50, threshold2=150)
    # edges = cv2.Canny(distance_image, threshold1=50, threshold2=500)
    # edges = cv2.Canny(distance_normalized, threshold1=20, threshold2=40)

    # Find contours (optional, if you want to process specific regions)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return edges, distance_normalized, contours

def extract_upper_edges(contours, y_threshold=1):
    upper_edges = []

    for contour in contours:
        # Approximate contour to a simpler polygon
        epsilon = 1e-10 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Loop through points in the approximated polygon
        for i in range(len(approx) - 1):
            pt1 = approx[i][0]
            pt2 = approx[i + 1][0]

            # Calculate the angle of the line segment
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1]
            angle = np.arctan2(dy, dx) * (180.0 / np.pi)

            # Check if the line segment is mostly horizontal
            if abs(angle) < y_threshold or abs(angle) > (180 - y_threshold):
                # Consider this line as part of the upper edge
                if pt1[1] < pt2[1]:  # Choose the higher (top) point
                    upper_edges.append(pt1)
                else:
                    upper_edges.append(pt2)

    return upper_edges

def connect_points(points, max_distance=0.1):
    """
    Connect points that are close to each other to form continuous lines.
    
    Args:
        points (list): List of points representing the upper edge.
        max_distance (int): Maximum distance to connect points.

    Returns:
        list: List of points forming a continuous line.
    """
    continuous_line = [points[0]]
    for i in range(1, len(points)):
        prev_point = continuous_line[-1]
        curr_point = points[i]
        # Calculate the Euclidean distance between the current point and the last point in the line
        distance = np.sqrt((curr_point[0] - prev_point[0])**2 + (curr_point[1] - prev_point[1])**2)
        
        # If the distance is small enough, add the current point to the line
        if distance <= max_distance:
            continuous_line.append(curr_point)
        else:
            # Interpolate points between disconnected segments to make them continuous
            num_points = int(distance // max_distance)
            x_values = np.linspace(prev_point[0], curr_point[0], num_points, dtype=int)
            y_values = np.linspace(prev_point[1], curr_point[1], num_points, dtype=int)
            for x, y in zip(x_values, y_values):
                continuous_line.append([x, y])
            continuous_line.append(curr_point)

    return continuous_line

def extract_upper_edges_noconnect(input_file: str):
    edges, distance_normalized, contours = process_distance_to_camera_image(input_file)
    upper_edges = extract_upper_edges(contours)
    return upper_edges
def extract_upper_edges_main(input_file: str):
    edges, distance_normalized, contours = process_distance_to_camera_image(input_file)
    upper_edges = extract_upper_edges(contours)
    continuous_upper_edges = connect_points(upper_edges)
    return upper_edges

def extract_edges_pixels(input_file: str):
    edges, distance_normalized, contours = process_distance_to_camera_image(input_file)
    edge_pixels = []
    # Find non-zero pixels (these are the edge pixels)
    edge_coords = np.transpose(np.nonzero(edges))
    
    # Convert each coordinate to a tuple and add to the list
    for coord in edge_coords:
        edge_pixels.append((coord[1], coord[0]))

    return edge_pixels
# Use this function to get the improved upper edges


# input_file = 'synthetic_data_generation/output/distance_to_camera_0001.npy'
# input_file = 'synthetic_data_generation/output/distance_to_camera_0004.npy'
input_file = 'synthetic_data_generation/output/distance_to_camera_0034.npy'
# input_file = 'synthetic_data_generation/output/distance_to_camera_0003.npy'
# input_file = 'synthetic_data_generation/output/distance_image002_FROMPOINTCLOUD.npy'
edges, distance_normalized, contours = process_distance_to_camera_image(input_file)
edge_pixels = extract_edges_pixels(input_file)
contour_image = np.zeros_like(edges)

def plot_edge_pixels(edge_pixels: List[Tuple[int, int]], depth_image: np.array, edges: np.array) -> None:
    """
    Plots the collected edge pixels as a scatter plot and overlays the edges mask.

    Args:
        edge_pixels (List[Tuple[int, int]]): List of (x, y) coordinates for each edge pixel.
        depth_image (np.array): The depth image to use as the background.
        edges (np.array): The edges mask to overlay.
    """
    x_coords, y_coords = zip(*edge_pixels)
    plt.figure(figsize=(10, 6))
    plt.imshow(depth_image, cmap='gray')
    plt.imshow(edges, cmap='jet', alpha=0.1)  # Overlay edges mask with some transparency
    plt.scatter(x_coords, y_coords, s=1, c='red', alpha=0.1)
    plt.scatter(300, 250, s=0.1, c='orange', alpha=0.5)
    plt.xlabel('X Coordinate (Width)')
    plt.ylabel('Y Coordinate (Height)')
    plt.title('Scatter Plot of Edge Pixels with Edges Overlay')
    plt.savefig('synthetic_data_generation/output/edge_pixels_overlay.png')
    # plt.show()

# Example usage:
# edges, _, _ = process_distance_to_camera_image('your_image_file.npy')
edge_pixels = extract_edges_pixels(input_file)
depth_image = np.load(input_file)
plot_edge_pixels(edge_pixels, depth_image, edges)






# cv2.drawContours(contour_image, contours, -1, (255), 1)

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
# plt.imshow(contour_image, cmap='gray')
# plt.title('Contours on Edges')
# plt.axis('off')

# # plt.show()
# plt.savefig('synthetic_data_generation/output/edge_detection.png')

# upper_edges = extract_upper_edges(contours)
# continuous_upper_edges = connect_points(upper_edges)
# # upper_edges = extract_top_horizontal_edges(contours)

# color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
#     # Draw the upper edges in red
# for edge in upper_edges:
#     color_edges[edge[1], edge[0]] = (255, 0, 0)  # Red color in BGR format



# # Draw the upper edges in red, ensuring they are within bounds
# color_edges2 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

# # for line in continuous_upper_edges:
# #     for edge in line:
# #         color_edges2[edge[1], edge[0]] = (0, 0, 255)  # Red color in BGR format
# for edge in continuous_upper_edges:
#     color_edges2[edge[1], edge[0]] = (0, 0, 255)  # Red color in BGR format

# # Plot the original edges and the color overlay
# plt.figure(figsize=(10, 5))
# plt.subplot(1, 3, 1)
# plt.imshow(edges, cmap='gray')
# plt.title('Original Edges')
# plt.axis('off')

# plt.subplot(1, 3, 2)
# plt.imshow(color_edges)
# plt.title('Filtered Upper Edges (Red)')
# plt.axis('off')

# plt.subplot(1, 3, 3)
# plt.imshow(color_edges2)
# plt.title('Continuous Filtered Upper Edges (blue)')
# plt.axis('off')


# plt.savefig('synthetic_data_generation/output/filtered_edges.png')
# upper_edges = np.array(upper_edges)
# continuous_upper_edges = np.array(continuous_upper_edges)
# # print(edges.shape)
# # print(edges)
# # for row in edges:
# #     for pixel in row:
# #         if pixel > 0:
# #             print(pixel)
#     # print(row)
# print(upper_edges.shape)
# # print(upper_edges)
# print(type(upper_edges))
# # print(continuous_upper_edges)
# # print(continuous_upper_edges.shape)
# # print(color_edges.shape)