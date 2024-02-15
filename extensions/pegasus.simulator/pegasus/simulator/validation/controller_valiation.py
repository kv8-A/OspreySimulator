import numpy as np
import matplotlib.pyplot as plt
from sim_log import SimulationDataLogger
logger = SimulationDataLogger()

filepath = "/home/kjell/Documents/Repositories/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/validation/controller_val/simulation_data.csv"

output_figure = "extensions/pegasus.simulator/pegasus/simulator/validation/controller_val/altitude_over_time.png"

logger.plot_from_csv(filepath=filepath, x_column="time", y_column="altitude",output_figure=output_figure)

