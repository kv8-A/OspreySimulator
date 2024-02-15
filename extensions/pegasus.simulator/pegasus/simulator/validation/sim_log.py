"""
This file will contain the data logger that will be used to plot all kind of graphs to look at the performance of the
dynamics. Will also be used for the results.
"""

import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd


class SimulationDataLogger:

    def __init__(self):
        self.data = {
            'time' : [],
            'altitude' : [],
            
        }

    def log(self,time,altitude, **kwargs):
        """Logs the provided argumetns at every step"""
        self.data['time'].append(time)
        self.data['altitude'].append(altitude)

        #Log if other variable gets logged...
        for key, value in kwargs.items():
            if key in self.data:
                self.data[key].append(value)
            else:
                self.data[key] = [value]
            
    
    def save_to_csv(self, filepath):
        """
        Saves the logged files, should only be implemented once simulation has stopped
        --> This means implement in the "stop()" method inside the fixedwing class.
        """

        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            # Write the header
            writer.writerow(self.data.keys())
            # Write the data rows
            for i in range(len(self.data['time'])):
                row = [self.data[key][i] for key in self.data.keys()]
                writer.writerow(row)


    def plot_altitude_over_time(self):
        """Plots the altitude of the drone over time."""
        plt.figure(figsize=(10, 6))
        plt.plot(self.data['time'], self.data['altitude'], label='Altitude')
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (m)')
        plt.title('Altitude Over Time')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_from_csv(self, filepath, x_column, y_column,output_figure):
        """Plots specified columns from a CSV file."""
        # Read the CSV file
        data = pd.read_csv(filepath)

        # Plot the data
        plt.figure(figsize=(10, 6))
        plt.plot(data[x_column], data[y_column], label=y_column)
        plt.xlabel(x_column)
        plt.ylabel(y_column)
        plt.title(f'{y_column} over {x_column}')
        plt.legend()
        plt.grid(True)
        # plt.show()
        plt.savefig(output_figure)
        plt.close()  # Clo


# # Use the plotting method
# logger = SimulationDataLogger()
# filepath = "/home/kjell/Documents/Repositories/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/controller/validation/simulation_data.csv"
# logger.plot_from_csv(filepath, "time", "altitude")
