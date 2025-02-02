# Python
import numpy as np
import matplotlib.pyplot as plt

# Load the npz file
npz_file_path = "/home/kjell/Documents/Repositories/Soar_EnvGen/wind_fields/run_20240609_224721_buildings/flow_field_data_50.npz"
data = np.load(npz_file_path)

# Extract the data
velocity_vectors = data['velocity_vectors']
cell_positions = data['cell_positions']


# # Plot the data points
# plt.figure(figsize=(10, 10))
# plt.quiver(cell_positions[:, 0], cell_positions[:, 1], velocity_vectors[:, 0], velocity_vectors[:, 1])
# plt.title('Wind Vectors')
# plt.xlabel('X Position')
# plt.ylabel('Y Position')
# plt.grid(True)
# plt.show()

# import matplotlib.pyplot as plt

# Assuming you have already defined the necessary variables and imported the required modules

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Assuming cell_positions is a 2D array with 3 columns for x, y, and z coordinates
ax.scatter(cell_positions[:, 0], cell_positions[:, 1], cell_positions[:, 2])

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
plt.title('Cell Positions')
plt.show()