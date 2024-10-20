import numpy as np
import matplotlib.pyplot as plt
import os 

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

def save_depth_image(data, output_path):
    """
    Saves a 2D numpy array as a grayscale image.

    Parameters:
    data (np.ndarray): The 2D numpy array to visualize.
    output_path (str): The path to save the output image.
    """
    if data.ndim == 2:
        plt.imshow(data, cmap='gray')
        plt.colorbar()
        plt.title("2D Distance to Camera Data")
        plt.savefig(output_path)
        plt.close()
    elif data.ndim == 3:
        for i in range(data.shape[0]):
            frame_output_path = output_path.replace('.png', f'_frame_{i}.png')
            plt.imshow(data[i], cmap='gray')
            plt.colorbar()
            plt.title(f"Frame {i} - 2D Distance to Camera Data")
            plt.savefig(frame_output_path)
            plt.close()
    else:
        print("The data has more than 3 dimensions, visualization is not supported.")

if __name__ == "__main__":
    directory = 'synthetic_data_generation/output'
    
    # Loop through all .npy files in the directory
    for filename in os.listdir(directory):
        if filename.endswith('.npy'):
            file_path = os.path.join(directory, filename)
            data = read_npy_file(file_path)
            
            if data is not None:
                print(f"Processing file: {filename}")
                output_path = os.path.join(directory, f"depth_image_{filename.replace('.npy', '.png')}")
                save_depth_image(data, output_path)
                print(f"Saved depth image as: {output_path}")
            else:
                print(f"Failed to load data from {filename}")