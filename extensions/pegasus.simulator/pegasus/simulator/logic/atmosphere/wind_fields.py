"""
This file will make sure that the wind field is initialized insdie a dataclass at the start of the simulation.
This will make sure that the wind field is calculated once and then used throughout the simulation.
The simulation will have to access the wind field at different points in time and at different positions.
This will mainly be postions of the drone and the wind field will be used to calculate the wind velocity at the drone's position.
"""
import numpy as np

from dataclasses import dataclass

@dataclass
class FlowField:   
    points: np.ndarray[float]
    wind_vectors: np.ndarray[float]

@dataclass
class FlowFieldData:
    time: float
    flow_field: FlowField

def load_flow_field_data(filepath):
    with np.load(filepath) as data:
        velocity_vectors = data['velocity_vectors']
        cell_positions = data['cell_positions']

    flow_field = FlowField(points=cell_positions, wind_vectors=velocity_vectors)
    # For simplicity, assume time is embedded in the file name or managed elsewhere
    flow_field_data = FlowFieldData(time=50.0, flow_field=flow_field)
    return flow_field_data

# Example usage
# loaded_flow_field_data = load_flow_field_data('flow_field_data.npz')
# print(loaded_flow_field_data)
