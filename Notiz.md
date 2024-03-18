1. In FrenetPath.cpp and fot_wrapper, a lot of fixed point are converted to double. This may help reducing runtime for later.
2. The files in polynomials forder are not using fixed point for calculation

Runtime measurment:

1. Configuration:

```python
hyperparameters= {

"max_speed": 25.0,

"max_accel": 15.0,

"max_curvature": 15.0,

"max_road_width_l": 5.0,

"max_road_width_r": 5.0,

"d_road_w": 0.1,

"dt": 0.1,

"maxt": 10.0,

"mint": 2.0,

"d_t_s": 0.5,

"n_s_sample": 2.0,

"obstacle_clearance": 0.1,

"kd": 1.0,

"kv": 0.1,

"ka": 0.1,

"kj": 0.1,

"kt": 0.1,

"ko": 0.1,

"klat": 1.0,

"klon": 1.0,

"num_threads": num_threads,  # set 0 to avoid using threaded algorithm

    }
```

before:
