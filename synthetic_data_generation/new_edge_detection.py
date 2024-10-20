import cv2
import numpy as np
import matplotlib.pyplot as plt

# Use non-interactive backend for headless environments
import matplotlib
matplotlib.use('Agg')

def process_distance_to_camera_image(image_path: str) -> np.ndarray:
    """
    Load the distance_to_camera image and apply edge detection to identify upper borders.
    
    Args:
        image_path (str): Path to the npy distance-to-camera file.

    Returns:
        np.ndarray: Binary edge image.
    """
    # Load the distance-to-camera image
    distance_image = np.load(image_path)
    distance_image = np.nan_to_num(distance_image, nan=0.0, posinf=255, neginf=0)

    # Normalize for better visualization (optional)
    distance_normalized = cv2.normalize(distance_image, None, 0, 255, cv2.NORM_MINMAX)
    distance_normalized = np.uint8(distance_normalized)

    # Use edge detection (Canny)
    edges = cv2.Canny(distance_normalized, threshold1=50, threshold2=150)
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

def connect_points(points, max_distance=10):
    """
    Connect points that are close to each other to form continuous lines.

    Args:
        points (list): List of points representing the upper edge.
        max_distance (int): Maximum distance to connect points.

    Returns:
        list: List of continuous lines, each line is a list of points.
    """
    if not points:
        return []

    # Sort the points based on x-coordinate
    points = sorted(points, key=lambda x: x[0])

    continuous_lines = []
    current_line = [points[0]]
    for i in range(1, len(points)):
        prev_point = current_line[-1]
        curr_point = points[i]
        # Calculate the Euclidean distance between the current point and the last point in the line
        distance = np.hypot(curr_point[0] - prev_point[0], curr_point[1] - prev_point[1])
        
        # If the distance is small enough, add the current point to the line
        if distance <= max_distance:
            current_line.append(curr_point)
        else:
            # Finish current line and start a new one
            continuous_lines.append(current_line)
            current_line = [curr_point]

    # Append the last line
    continuous_lines.append(current_line)

    return continuous_lines

# Use this function to get the improved upper edges

input_file = 'synthetic_data_generation/output/distance_to_camera_0001.npy'
# input_file = 'synthetic_data_generation/output/distance_to_camera_0002.npy'
edges, distance_normalized, contours = process_distance_to_camera_image(input_file)
contour_image = np.zeros_like(edges)
cv2.drawContours(contour_image, contours, -1, (255), 1)

plt.figure(figsize=(15, 5))
plt.subplot(1, 3, 1)
plt.imshow(distance_normalized, cmap='gray')
plt.title('Normalized Distance Image')
plt.axis('off')

plt.subplot(1, 3, 2)
plt.imshow(edges, cmap='gray')
plt.title('Edges Detected')
plt.axis('off')

plt.subplot(1, 3, 3)
plt.imshow(contour_image, cmap='gray')
plt.title('Contours on Edges')
plt.axis('off')

# plt.show()
plt.savefig('synthetic_data_generation/output/edge_detection.png')

upper_edges = extract_upper_edges(contours)
continuous_upper_edges = connect_points(upper_edges, max_distance=0.1)
# upper_edges = extract_top_horizontal_edges(contours)

# Draw the upper edges in red
color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
for edge in upper_edges:
    x, y = edge
    if 0 <= x < color_edges.shape[1] and 0 <= y < color_edges.shape[0]:
        color_edges[y, x] = (0, 0, 255)  # Red color in BGR format

# Draw the continuous upper edges in blue
color_edges2 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

for line in continuous_upper_edges:
    for edge in line:
        x, y = edge
        if 0 <= x < color_edges2.shape[1] and 0 <= y < color_edges2.shape[0]:
            color_edges2[y, x] = (255, 0, 0)  # Blue color in BGR format

# Plot the original edges and the color overlay
plt.figure(figsize=(10, 5))
plt.subplot(1, 3, 1)
plt.imshow(edges, cmap='gray')
plt.title('Original Edges')
plt.axis('off')

plt.subplot(1, 3, 2)
plt.imshow(color_edges)
plt.title('Filtered Upper Edges (Red)')
plt.axis('off')

plt.subplot(1, 3, 3)
plt.imshow(color_edges2)
plt.title('Continuous Filtered Upper Edges (Blue)')
plt.axis('off')

plt.savefig('synthetic_data_generation/output/filtered_edges.png')

# print(upper_edges)
