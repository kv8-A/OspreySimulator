import numpy as np

def read_npy_file(file_path):
    """
    Reads a .npy file and returns the numpy array.
    
    Parameters:
    file_path (str): The path to the .npy file.

    Returns:
    np.ndarray: The numpy array stored in the .npy file.
    """
    try:
        data = np.load(file_path)
        return data
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

if __name__ == "__main__":
    file_path = 'distance_to_camera_0002.npy'  # Replace with your .npy file path
    data = read_npy_file(file_path)
    
    if data is not None:
        print("Data shape:", data.shape)
        print("Data type:", data.dtype)
        print("Data contents:\n", data)

        # If the data is 2D or 3D, you can visualize it using matplotlib
        import matplotlib.pyplot as plt
        
        if data.ndim == 2:
            plt.imshow(data, cmap='gray')
            plt.colorbar()
            plt.title("2D Distance to Camera Data")
            plt.show()
        elif data.ndim == 3:
            for i in range(data.shape[0]):
                plt.imshow(data[i], cmap='gray')
                plt.colorbar()
                plt.title(f"Frame {i} - 2D Distance to Camera Data")
                plt.show()
        else:
            print("The data has more than 3 dimensions, visualization is not supported.")
    else:
        print("Failed to load the data.")