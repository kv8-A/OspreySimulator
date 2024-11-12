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



#  def process_top_edge_pixels(self, movement_vector: np.ndarray, edges: np.ndarray) -> np.ndarray:
#         """
#         Processes only the top-edge pixels and computes the vertical wind component for each.

#         Args:
#             movement_vector: np.ndarray of shape (3,), movement vector to apply to each coordinate.
#             edges: List of (y, x) tuples representing the pixel coordinates of the edges.

#         Returns:
#             np.ndarray of shape (H, W): An array containing the vertical wind component at each pixel.
#         """
#         if self.world_coordinates_image is None:
#             raise ValueError("World coordinates are not loaded. Load world coordinates first.")

#         self.vertical_wind_image = np.full((self.H, self.W), np.nan)  # Initialize the output image with NaN

#         for x, y in edges:
#             if 0 <= y < self.H and 0 <= x < self.W:
#                 vertical_wind = self.process_pixel(y, x, movement_vector)
#                 self.vertical_wind_image[y, x] = vertical_wind

#                  # Assign vertical wind to the 10 pixels above
#                 self._propagate_vertical_wind(y, x, vertical_wind)

#                 # Assign horizontal wind to the 4 pixels to the right
#                 # Initialize flags for stopping horizontal propagation
#                 left_break = False
#                 right_break = False

#                 # Assign vertical wind to the 4 pixels in the x direction (both left and right)
#                 # for dx in range(1, 2):
#                 #     # Move left in the x direction
                    
#                 #     new_x_left = x - dx

#                 #     if new_x_left >= 0 and not np.isnan(self.vertical_wind_image[y, new_x_left]):
#                 #         left_break = True

#                 #     # if new_x_left >= 0 and not left_break:  # Check if within bounds and not broken
#                 #     if new_x_left >= 0:  # Check if within bounds and not broken
#                 #         if np.isnan(self.vertical_wind_image[y, new_x_left]):  # Only assign if NaN
#                 #             self.vertical_wind_image[y, new_x_left] = vertical_wind
#                 #             # Propagate vertical wind upward for the left pixel
#                 #             self._propagate_vertical_wind(y, new_x_left, vertical_wind)
#                 #         else:
#                 #             left_break = True  # Set flag to stop further leftward propagation

#                 #     # Move right in the x direction
#                 #     new_x_right = x + dx
#                 #     if new_x_right < self.W and not np.isnan(self.vertical_wind_image[y, new_x_right]):
#                 #         right_break = True

#                 #     # if new_x_right < self.W and not right_break:  # Check if within bounds and not broken
#                 #     if new_x_right < self.W:  # Check if within bounds and not broken
#                 #         if np.isnan(self.vertical_wind_image[y, new_x_right]):  # Only assign if NaN
#                 #             self.vertical_wind_image[y, new_x_right] = vertical_wind
#                 #             # Propagate vertical wind upward for the right pixel
#                 #             self._propagate_vertical_wind(y, new_x_right, vertical_wind)
#                 #         else:
#                 #             right_break = True  # Set flag to stop further rightward propagation
#             else:
#                 print(f"Pixel coordinate ({y}, {x}) is out of bounds and will be skipped.")

#         return self.vertical_wind_image
    
# def _propagate_vertical_wind(y: int, x: int, vertical_wind: float) -> None:
#     """
#     Propagates the vertical wind value upward for up to 10 pixels along the y-axis.

#     Args:
#         y: int, starting y-coordinate.
#         x: int, x-coordinate of the pixel.
#         vertical_wind: float, vertical wind component to assign.
#     """
#     # Assign vertical wind to the 10 pixels above (y direction)
#     for dy in range(1, 11):
#         new_y = y - dy  # Move upward by decreasing y
#         if new_y < 0:  # Check if new_y goes above the image
#             break  # Stop if out of bounds (above the top edge)

#         # Only assign the value if the current pixel is still np.nan
#         if np.isnan(self.vertical_wind_image[new_y, x]):
#             self.vertical_wind_image[new_y, x] = vertical_wind

#         # right_break = False
#         # for dx in range(1, 10):
#         #     new_x_right = x + dx
#         #     if new_x_right >= self.W:
#         #         break  # Stop if out of bounds

#         #     # If the pixel is already assigned a value, stop further rightward propagation
#         #     if not np.isnan(self.vertical_wind_image[new_y, new_x_right]):
#         #         right_break = True

#         #     if not right_break:
#         #         if np.isnan(self.vertical_wind_image[new_y, new_x_right]):  # Only assign if NaN
#         #             self.vertical_wind_image[new_y, new_x_right] = vertical_wind
#         #         else:
#         #             right_break = True  # Stop further rightward propagation
#         else:
#             break  # Stop if a value has already been assigned
