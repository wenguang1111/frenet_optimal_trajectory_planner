import pandas as pd
from ctypes import Structure, c_float, c_int, c_size_t, POINTER, cast, pointer
import FrenetOptimalTrajectory.py_cpp_struct as py_cpp_struct
initial_conditions_floating_point = []
calculated_data_floating_point = []
record_hyperparameters =[]
MAX_PATH_LENGTH = py_cpp_struct.MAX_PATH_LENGTH
    
def writeInitalConditions(initial_conditions):
    initial_conditions_floating_point.append([
    initial_conditions.s0,
    initial_conditions.c_speed,
    initial_conditions.c_d,
    initial_conditions.c_d_d,
    initial_conditions.c_d_dd,
    initial_conditions.target_speed,
    [initial_conditions.wx[i] for i in range(initial_conditions.nw)],
    [initial_conditions.wy[i] for i in range(initial_conditions.nw)],
    initial_conditions.nw,
    [initial_conditions.o_llx[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_lly[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_urx[i] for i in range(initial_conditions.no)],
    [initial_conditions.o_ury[i] for i in range(initial_conditions.no)],
    initial_conditions.no
])

def writeCalculatedData(returnValues, step_number):
    for i in range(returnValues.path_length):
        calculated_data_floating_point.append([
            step_number,
            returnValues.success,
            returnValues.x_path[i],
            returnValues.y_path[i],
            returnValues.speeds[i],
            returnValues.ix[i],
            returnValues.iy[i],
            returnValues.iyaw[i],
            returnValues.d[i],
            returnValues.s[i],
            returnValues.speeds_x[i],
            returnValues.speeds_y[i],
            returnValues.params[i],
            returnValues.costs[i],
            returnValues.runtime
        ])

def writeHyperparameters(hyperparameters):
    record_hyperparameters.append([
        hyperparameters.max_speed,
        hyperparameters.max_accel,
        hyperparameters.max_curvature,
        hyperparameters.max_road_width_l,
        hyperparameters.max_road_width_r,
        hyperparameters.d_road_w,
        hyperparameters.dt,
        hyperparameters.maxt,
        hyperparameters.mint,
        hyperparameters.d_t_s,
        hyperparameters.n_s_sample,
        hyperparameters.obstacle_clearance,
        hyperparameters.kd,
        hyperparameters.kv,
        hyperparameters.ka,
        hyperparameters.kj,
        hyperparameters.kt,
        hyperparameters.ko,
        hyperparameters.klat,
        hyperparameters.klon,
        hyperparameters.num_threads
    ])

def saveInitialConditions():
    df = pd.DataFrame(initial_conditions_floating_point, columns=[
        's0', 'c_speed', 'c_d', 'c_d_d', 'c_d_dd', 'target_speed', 
        'wx', 'wy', 'nw', 'o_llx', 'o_lly', 'o_urx', 'o_ury', 'no'
    ])
    df.to_csv('FloatingPoint_InitData.csv', index=False, mode='w')

def saveCalculatedData():
    df_calculated = pd.DataFrame(calculated_data_floating_point, columns=[
        'step', 'success', 'x_path', 'y_path', 'speeds', 'ix', 'iy', 'iyaw', 
        'd', 's', 'speeds_x', 'speeds_y', 'params', 'costs', 'runtime'
    ])
    df_calculated.to_csv('FloatingPoint_Calculated.csv', index=False, mode='w')

def saveHyperparameter():
    hp_df = pd.DataFrame(record_hyperparameters, columns=[
        'max_speed', 'max_accel', 'max_curvature', 'max_road_width_l', 'max_road_width_r', 
        'd_road_w', 'dt', 'maxt', 'mint', 'd_t_s', 'n_s_sample', 'obstacle_clearance', 
        'kd', 'kv', 'ka', 'kj', 'kt', 'ko', 'klat', 'klon', 'num_threads'
    ])
    hp_df.to_csv('FloatingPoint_Hyperparameters.csv', index=False, mode='w')

def readInitialConditions(filename='FloatingPoint_InitData.csv'):
    df = pd.read_csv(filename)
    initial_conditions_list = []
    for _, row in df.iterrows():
        initial_conditions = py_cpp_struct.FrenetInitialConditions(
            s0=row['s0'],
            c_speed=row['c_speed'],
            c_d=row['c_d'],
            c_d_d=row['c_d_d'],
            c_d_dd=row['c_d_dd'],
            target_speed=row['target_speed'],
            wx=cast((c_float * row['nw'])(*eval(row['wx'])), POINTER(c_float)),
            wy=cast((c_float * row['nw'])(*eval(row['wy'])), POINTER(c_float)),
            nw=row['nw'],
            o_llx=cast((c_float * row['no'])(*eval(row['o_llx'])), POINTER(c_float)),
            o_lly=cast((c_float * row['no'])(*eval(row['o_lly'])), POINTER(c_float)),
            o_urx=cast((c_float * row['no'])(*eval(row['o_urx'])), POINTER(c_float)),
            o_ury=cast((c_float * row['no'])(*eval(row['o_ury'])), POINTER(c_float)),
            no=row['no']
        )
        initial_conditions_list.append(initial_conditions)
    return initial_conditions_list

def readCalculatedData(filename='FloatingPoint_Calculated.csv'):
    df = pd.read_csv(filename)
    calculated_data_list = []
    for _, row in df.iterrows():
        return_values = py_cpp_struct.FrenetReturnValues(
            success=row['success'],
            path_length=row['path_length'],
            x_path=(c_float * MAX_PATH_LENGTH)(*row['x_path']),
            y_path=(c_float * MAX_PATH_LENGTH)(*row['y_path']),
            speeds=(c_float * MAX_PATH_LENGTH)(*row['speeds']),
            ix=(c_float * MAX_PATH_LENGTH)(*row['ix']),
            iy=(c_float * MAX_PATH_LENGTH)(*row['iy']),
            iyaw=(c_float * MAX_PATH_LENGTH)(*row['iyaw']),
            d=(c_float * MAX_PATH_LENGTH)(*row['d']),
            s=(c_float * MAX_PATH_LENGTH)(*row['s']),
            speeds_x=(c_float * MAX_PATH_LENGTH)(*row['speeds_x']),
            speeds_y=(c_float * MAX_PATH_LENGTH)(*row['speeds_y']),
            params=(c_float * MAX_PATH_LENGTH)(*row['params']),
            costs=(c_float * MAX_PATH_LENGTH)(*row['costs']),
            runtime=row['runtime']
        )
        calculated_data_list.append(return_values)
    return calculated_data_list

def readHyperparameters(filename='FloatingPoint_Hyperparameters.csv'):
    df = pd.read_csv(filename)
    hyperparameters_list = []
    for _, row in df.iterrows():
        hyperparameters = py_cpp_struct.FrenetHyperparameters(
            max_speed=row['max_speed'],
            max_accel=row['max_accel'],
            max_curvature=row['max_curvature'],
            max_road_width_l=row['max_road_width_l'],
            max_road_width_r=row['max_road_width_r'],
            d_road_w=row['d_road_w'],
            dt=row['dt'],
            maxt=row['maxt'],
            mint=row['mint'],
            d_t_s=row['d_t_s'],
            n_s_sample=row['n_s_sample'],
            obstacle_clearance=row['obstacle_clearance'],
            kd=row['kd'],
            kv=row['kv'],
            ka=row['ka'],
            kj=row['kj'],
            kt=row['kt'],
            ko=row['ko'],
            klat=row['klat'],
            klon=row['klon'],
            num_threads=int(row['num_threads'])
        )
        hyperparameters_list.append(hyperparameters)
    return hyperparameters_list