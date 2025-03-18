import numpy as np
from SALib.sample import saltelli
from SALib.analyze import sobol
import fot_wrapper
import fot_wrapper_sobol
import argparse
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import pandas as pd
import os
import math

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
    #initial_conditions=?
    fot_wrapper.run_fot(initial_conditions, hyperparameters,x) 
    # run the fixed point version and calculate the error
    # evaluate the error via average error?
    return error


def fot(show_animation=True,
        show_info=False,
        num_threads=0,
        save_frame=False,
        show_sampling_path=False):
    conds = {
        's0':
        0,
        'target_speed':
        20,
        'wp': [[0, 0], [50, 0], [120, 0]],  #way point
        # 'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
        #         [128, 2, 132, 6]],
        'obs': [[25,-2,28,2],[29,2,30,3],[29,-1,30,1],[30.5,1,31,4],[31,-1,33,1],
                [32,3,33,4],[35,4,40,6],[48, -5, 52, -6], [53,-4,55,-5],[53,3,55,5],
                [56,-4,58,-5],[56,2,58,5],[59,-4,61,-5],[59,2,61,5],[62,-4,63,-5],
                [62,2,63,5],[65,-4,68,-5],[65,2,68,5],[85, -4, 90, 1], [85, 6, 90, 10]],
        'pos': [0, 0],
        'vel': [0, 0],
    }  # paste output from debug log

    initial_conditions = {
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']),
        'vel': np.array(conds['vel']),
        'wp': np.array(conds['wp']),
        'obs': np.array(conds['obs'])
    }

    # static elements of planner
    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]
    obs = np.array(conds['obs'])

    print(os.getpid())

    # simulation config
    sim_loop = 200
    area = 40
    total_time = 0
    total_time_c = 0
    time_list = []
    initial_conditions_floating_point = []
    calculated_data_floating_point = []
    initial_conditions_floating_point.append([
            initial_conditions['pos'][0],
            initial_conditions['pos'][1],
            initial_conditions['vel'][0],
            initial_conditions['vel'][1],
            initial_conditions['ps'],
        ])

    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        if show_sampling_path:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c, sample_x, sample_y = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)  
        else:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)
                 
        end_time = time.time() - start_time
        print("Time taken: {} s".format(end_time))
        # print("Time take by c module:{} ms".format(runtime_c))
        total_time += end_time
        total_time_c += runtime_c
        # time_list.append(runtime_c)
        time_list.append(end_time)

        # reconstruct initial_conditions
        if success:
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            if show_info:
                print(costs)
            # Save initial conditions data
            initial_conditions_floating_point.append([
                initial_conditions['pos'][0],
                initial_conditions['pos'][1],
                initial_conditions['vel'][0],
                initial_conditions['vel'][1],
                initial_conditions['ps'],
            ])
            # Save calculated data
            num_elements = len(result_x)
            for j in range(num_elements):
                calculated_data_floating_point.append([
                    i,  # step
                    result_x[j],
                    result_y[j],
                    speeds[j],
                    iyaw[j]
                ])
            
        else:
            print("Failed unexpectedly")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.plot(wx, wy)
            if obs.shape[0] == 0:
                obs = np.empty((0, 4))
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            if show_sampling_path:
                for path_x, path_y in zip(sample_x, sample_y):
                    plt.plot(path_x, path_y, color='grey', linestyle='--', linewidth=0.5)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title("v[m/s]:" +
                      str(np.linalg.norm(initial_conditions['vel']))[0:4])
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

    print("Finish")

    # Save initial conditions data to CSV
    df = pd.DataFrame(initial_conditions_floating_point, columns=['x', 'y', 'v_x', 'v_y', 's'])
    df.to_csv('FloatingPoint_InitData.csv', index=False)
    # Save calculated data to CSV
    df_calculated = pd.DataFrame(calculated_data_floating_point, columns=['step', 'result_x', 'result_y', 'speeds', 'yaw'])
    df_calculated.to_csv('FloatingPoint_Calculated.csv', index=False)

    return time_list

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

    # run_floating_point version and record data
    fot_wrapper.USING_FLOOTING_POINT=True
    fot(args.display, args.verbose, args.thread, args.save, args.display_sampling_paths)

    # # sampling the bitwidths of fractional part of the fixed-point numbers
    param_values = saltelli.sample(problem, 1000)
    # # TODO: hier will be a loop later for all time steps
    Y = np.array([calculate_error(x) for x in param_values])

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