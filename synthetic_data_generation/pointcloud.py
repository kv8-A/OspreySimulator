import numpy as np

file_path = 'synthetic_data_generation/output/pointcloud_rgb_0002.npy'
point_cloud = np.load(file_path)

print(point_cloud.shape)
print(point_cloud)