import numpy as np 


def depth_to_world_coordinates(image_distance_from_camera:np.ndarray, point_cloud):
    """
    Reshape the point cloud to match the shape of the depth image (H, W, 3).
    
    Parameters:
    - image_distance_from_camera: 2D numpy array (H, W) containing depth data.
    - point_cloud: 2D numpy array (N, 3) containing point cloud coordinates.
    
    Returns:
    - reshaped_point_cloud: 3D numpy array (H, W, 3) containing the reshaped point cloud,
                            with np.nan for pixels where no corresponding points exist.
    """
    H, W = image_distance_from_camera.shape  # Get height and width from the depth image
    reshaped_point_cloud = np.full((H, W, 3), np.nan)  # Initialize with NaN values

    point_cloud_index = 0  # Counter for iterating through the point cloud

    # Iterate through each pixel in the depth image
    for v in range(H):
        for u in range(W):
            # Check if the depth value is finite (not infinity)
            if np.isfinite(image_distance_from_camera[v, u]):
                # Assign the corresponding point cloud coordinates to the reshaped array
                reshaped_point_cloud[v, u] = point_cloud[point_cloud_index]
                point_cloud_index += 1

    return reshaped_point_cloud


def calculate_distance_from_camera(reshaped_point_cloud, camera_position):
    """
    Calculate the Euclidean distance between the camera position and each point in the reshaped point cloud.
    
    Parameters:
    - reshaped_point_cloud: 3D numpy array (H, W, 3) containing the reshaped point cloud.
    - camera_position: Tuple (x, y, z) representing the camera's position in 3D space.
    
    Returns:
    - distance_image: 2D numpy array (H, W) containing the calculated distances,
                      with np.inf where there are no valid points.
    """
    H, W, _ = reshaped_point_cloud.shape  # Get height and width from the reshaped point cloud
    distance_image = np.full((H, W), np.inf)  # Initialize distance image with np.inf

    # Iterate through each pixel in the reshaped point cloud
    for v in range(H):
        for u in range(W):
            point = reshaped_point_cloud[v, u]
            # Check if the point is valid (not NaN)
            if not np.isnan(point).any():
                # Calculate the Euclidean distance between the camera position and the point
                distance = np.linalg.norm(point - camera_position)  # Euclidean distance formula
                distance_image[v, u] = distance  # Store the distance in the distance image

    return distance_image

dist_image0002 = np.load('synthetic_data_generation/output/distance_to_camera_0002.npy')
point_cloud002 = np.load('synthetic_data_generation/output/pointcloud_0002.npy')

reshaped_point_cloud002 = depth_to_world_coordinates(dist_image0002, point_cloud002)

print(reshaped_point_cloud002.shape)

camera_position = np.array([78, 625, 62])  

distance_image002 = calculate_distance_from_camera(reshaped_point_cloud002, camera_position)

np.save('synthetic_data_generation/output/distance_image002_FROMPOINTCLOUD.npy', distance_image002)