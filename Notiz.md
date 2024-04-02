1. In FrenetPath.cpp and fot_wrapper, a lot of fixed point are converted to double. This may help reducing runtime for later.
2. The files in polynomials forder are not using fixed point for calculation

Runtime measurment:

1. Configuration:

```python
def fot(show_animation=True,
        show_info=False,
        num_threads=0,
        save_frame=False):
    conds = {
        's0':
        0,
        'target_speed':
        20,
        'wp': [[0, 0], [50, 0], [150, 0]],  #way point
        # 'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
        #         [128, 2, 132, 6]],
        'obs': [[25,-2,28,2],[35,4,40,6],[48, -5, 52, -6], [62,-4,68,-5],[62,2,68,5],[98, -4, 102, 2], [98, 6, 102, 10],
                [145, 2, 150, 6]],
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

    hyperparameters = { #！the datatye of following variable is int_5_10 in c, so be sure that tha value is between -32 and 32
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 6.0,
        "max_road_width_r": 6.0,
        "d_road_w": 0.1,
        "dt": 0.1,
        "maxt": 5,
        "mint": 2,
        "d_t_s": 0.1,
        "n_s_sample": 5.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 20.0,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
    }
```

before:
