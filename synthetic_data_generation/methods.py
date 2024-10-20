from omni.isaac.synthetic_utils import SyntheticDataHelper


# Step 3 
def get_camera_intrinsics(camera_path: str) -> np.ndarray:
    """
    Get the camera intrinsic matrix.

    Args:
        camera_path (str): USD path to the camera.

    Returns:
        np.ndarray: Camera intrinsic matrix (3x3).
    """
    sdh = SyntheticDataHelper()
    intrinsics: np.ndarray = sdh.get_intrinsic_matrix(camera_path)
    return intrinsics

def pixel_to_world(u: int, v: int, distance: float, intrinsics: np.ndarray, camera_pose: np.ndarray) -> np.ndarray:
    """
    Convert pixel coordinates and distance to world coordinates.

    Args:
        u (int): Pixel x-coordinate.
        v (int): Pixel y-coordinate.
        distance (float): Distance from camera to the point.
        intrinsics (np.ndarray): Camera intrinsic matrix (3x3).
        camera_pose (np.ndarray): Camera pose matrix (4x4).

    Returns:
        np.ndarray: 3D point in world coordinates (3,).
    """
    # Convert pixel coordinates (u, v) and distance to camera coordinates
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    x = (u - cx) * distance / fx
    y = (v - cy) * distance / fy
    z = distance

    # Camera coordinates in homogeneous coordinates
    point_camera: np.ndarray = np.array([x, y, z, 1], dtype=np.float32).reshape(4, 1)

    # Transform to world coordinates using camera_pose (4x4 matrix)
    point_world_homogeneous: np.ndarray = np.dot(camera_pose, point_camera)

    point_world: np.ndarray = point_world_homogeneous[:3, 0] / point_world_homogeneous[3, 0]

    return point_world



## Step 4

def load_wind_field_data(wind_field_path: str) -> np.ndarray:
    """
    Load wind-field data from a .npy file.

    Args:
        wind_field_path (str): Path to the wind-field data.

    Returns:
        np.ndarray: Wind-field data array.
    """
    wind_field: np.ndarray = np.load(wind_field_path)
    return wind_field

def get_wind_velocity_at_point(
    point_world: np.ndarray,
    wind_field: np.ndarray,
    wind_field_origin: np.ndarray,
    wind_field_resolution: np.ndarray
) -> float:
    """
    Get the upward wind velocity at a given 3D point.

    Args:
        point_world (np.ndarray): 3D point in world coordinates (3,).
        wind_field (np.ndarray): Wind-field data array.
        wind_field_origin (np.ndarray): Origin of the wind-field grid in world coordinates (3,).
        wind_field_resolution (np.ndarray): Resolution of the wind-field grid (dx, dy, dz) (3,).

    Returns:
        float: Upward wind velocity at the point.
    """
    # Convert world coordinates to wind-field grid indices
    indices: np.ndarray = ((point_world - wind_field_origin) / wind_field_resolution).astype(int)

    # Ensure indices are within bounds
    indices = np.clip(indices, 0, np.array(wind_field.shape[:3]) - 1)

    # Get the upward (z) component of the wind velocity
    upward_velocity: float = wind_field[indices[0], indices[1], indices[2], 2]  # Assuming last dim is [vx, vy, vz]

    return upward_velocity