import numpy as np
from SALib.sample import saltelli
from SALib.analyze import sobol
# import fot_wrapper
import FrenetOptimalTrajectory.fot_wrapper_sobol as fot_wrapper_sobol
import argparse
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import pandas as pd
import os
import math
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Sobol import recordData
import FrenetOptimalTrajectory.py_cpp_struct as py_cpp_struct

# Define the problem: ranges for input parameters
problem = {
    'num_vars': 10,  # Number of input variables
    'names': ['o_position', 'wp_position', 'd','d_d','d_dd','d_ddd','s','s_d','s_dd','s_ddd'],  # Variable names
    'bounds': [[0, 16], [0, 16], [0, 16], [0, 16], [0, 16], [0, 16], [0, 16], [0, 16], [0, 16], [0, 16]]  # Ranges for each variable
}

hyperparameters = {
    "max_speed": 25.0,
    "max_accel": 15.0,
    "max_curvature": 15.0,
    "max_road_width_l": 6.0,
    "max_road_width_r": 6.0,
    "d_road_w": 0.5,
    "dt": 0.5,
    "maxt": 5,
    "mint": 2,
    "d_t_s": 0.1,
    "n_s_sample": 1.0,
    "obstacle_clearance": 0.1,
    "kd": 10.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.1,
    "kt": 0.1,
    "ko": 10.0,
    "klat": 1.0,
    "klon": 1.0,
    "num_threads": 0,  # set 0 to avoid using threaded algorithm
}


# Function Wrapper
def calculate_error(x):
    initial_conditions = recordData.readInitialConditions()
    hyperparameters = recordData.readHyperparameters()
    # Convert x to FractionBitWidth structure
    # bit_width = py_cpp_struct.FractionBitWidth(
    #     o_position=int(x[0]),
    #     wp_position=int(x[1]),
    #     d=int(x[2]),
    #     d_d=int(x[3]),
    #     d_dd=int(x[4]),
    #     d_ddd=int(x[5]),
    #     s=int(x[6]),
    #     s_d=int(x[7]),
    #     s_dd=int(x[8]),
    #     s_ddd=int(x[9])
    # )
    print(os.getpid())
    fot_wrapper_sobol.run_fot(initial_conditions[0], hyperparameters[0]) 
    # run the fixed point version and calculate the error
    # evaluate the error via average error?
    error = 0
    return error


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open")
    parser.add_argument("-v",
                        "--verbose",
                        action="store_true",
                        help="verbose mode, show all state info")
    parser.add_argument("-s",
                        "--save",
                        action="store_true",
                        help="save each frame of simulation")
    parser.add_argument("-t",
                        "--thread",
                        type=int,
                        default=0,
                        help="set number of threads to run with")
    parser.add_argument(
        "-dsp",
        "--display_sampling_paths",
        action="store_true",
        help="show sampling paths in animation")
    args = parser.parse_args()

    # # sampling the bitwidths of fractional part of the fixed-point numbers
    # param_values = saltelli.sample(problem, 1024)
    # # TODO: hier will be a loop later for all time steps
    # Y = np.array([calculate_error(x) for x in param_values])

    # # Compute Sobol sensitivity indices
    # Si = sobol.analyze(problem, Y)

    # # Display results
    # print("First-order Sobol indices:", Si['S1'])
    # print("Total Sobol indices:", Si['ST'])
    # print("Second-order Sobol indices:", Si['S2'])

    # # Interpretation:
    # # - S1: First-order effect (direct impact of each input on output variance)
    # # - ST: Total effect (direct + interaction effects)
    # # - S2: Second-order effect (interaction between pairs of inputs)
    calculate_error(0)