from ctypes import c_float, c_int, c_size_t, POINTER, Structure, CDLL

_c_float_p = POINTER(c_float)

MAX_PATH_LENGTH = 100
MAX_SAMPLE_SIZE = 100

class FrenetInitialConditions(Structure):
    _fields_ = [
        ("s0", c_float),
        ("c_speed", c_float),
        ("c_d", c_float),
        ("c_d_d", c_float),
        ("c_d_dd", c_float),
        ("target_speed", c_float),
        ("wx", _c_float_p),
        ("wy", _c_float_p),
        ("nw", c_int),
        ("o_llx", _c_float_p),
        ("o_lly", _c_float_p),
        ("o_urx", _c_float_p),
        ("o_ury", _c_float_p),
        ("no", c_int)
    ]
    
class FrenetReturnValues(Structure):
    _fields_ = [
        ("success", c_int),
        ("path_length", c_size_t),
        ("x_path", c_float * MAX_PATH_LENGTH),
        ("y_path", c_float * MAX_PATH_LENGTH),
        ("speeds", c_float * MAX_PATH_LENGTH),
        ("ix", c_float * MAX_PATH_LENGTH),
        ("iy", c_float * MAX_PATH_LENGTH),
        ("iyaw", c_float * MAX_PATH_LENGTH),
        ("d", c_float * MAX_PATH_LENGTH),
        ("s", c_float * MAX_PATH_LENGTH),
        ("speeds_x", c_float * MAX_PATH_LENGTH),
        ("speeds_y", c_float * MAX_PATH_LENGTH),
        ("params", c_float * MAX_PATH_LENGTH),
        ("costs", c_float * MAX_PATH_LENGTH),
        ("runtime", c_float)
        ("sample_x", c_float*MAX_PATH_LENGTH*MAX_SAMPLE_SIZE),
        ("sample_y", c_float*MAX_PATH_LENGTH*MAX_SAMPLE_SIZE),
        ("sample_lengths", c_size_t*MAX_SAMPLE_SIZE),
        ("sample_size", c_size_t)
    ]

class FrenetHyperparameters(Structure):
    _fields_ = [
        ("max_speed", c_float),
        ("max_accel", c_float),
        ("max_curvature", c_float),
        ("max_road_width_l", c_float),
        ("max_road_width_r", c_float),
        ("d_road_w", c_float),
        ("dt", c_float),
        ("maxt", c_float),
        ("mint", c_float),
        ("d_t_s", c_float),
        ("n_s_sample", c_float),
        ("obstacle_clearance", c_float),
        ("kd", c_float),
        ("kv", c_float),
        ("ka", c_float),
        ("kj", c_float),
        ("kt", c_float),
        ("ko", c_float),
        ("klat", c_float),
        ("klon", c_float),
        ("num_threads", c_int)
    ]
