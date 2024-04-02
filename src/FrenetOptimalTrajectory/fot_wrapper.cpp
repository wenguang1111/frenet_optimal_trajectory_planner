#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "utils.h"
#include "tool/recorder.h"

#include <stddef.h>
#include <vector>
#include <time.h>

using namespace std;

// C++ wrapper to expose the FrenetOptimalTrajectory class to python
extern "C" {
    // Compute the frenet optimal trajectory given initial conditions
    // in frenet space.
    //
    // Arguments:
    //      fot_ic_d (FrenetInitialConditions *):
    //          struct ptr containing relevant initial conditions to compute
    //          Frenet Optimal Trajectory
    //      fot_hp_d (FrenetHyperparameters *):
    //          struct ptr containing relevant hyperparameters to compute
    //          Frenet Optimal Trajectory
    //      x_path, y_path, speeds (double *):
    //          ptr to storage arrays for Frenet Optimal Trajectory
    //      params (double *):
    //          ptr to store initial conditions for debugging
    //
    // Returns:
    //      1 if successful, 0 if failure
    //      Also stores the Frenet Optimal Trajectory into x_path, y_path,
    //      speeds if it exists
    void run_fot(
            FrenetInitialConditions_double *fot_ic_d, FrenetHyperparameters_double *fot_hp_d,
            FrenetReturnValues_double *fot_rv_d
            ) {
        FrenetHyperparameters* fot_hp = new FrenetHyperparameters();
        FrenetInitialConditions* fot_ic = new FrenetInitialConditions();
        
        // data type convertion for interface
        //-----------------FrenetInitialConditions------------------------------
        fot_ic->s0 = fot_ic_d->s0;
        fot_ic->c_speed = fot_ic_d->c_speed;
        fot_ic->c_d = fot_ic_d->c_d;
        fot_ic->c_d_d = fot_ic_d->c_d_d;
        fot_ic->c_d_dd = fot_ic_d->c_d_dd;
        fot_ic->target_speed = fot_ic_d->target_speed;
        fot_ic->nw = fot_ic_d->nw;
        fot_ic->no = fot_ic_d->no;
        
        fot_ic->wx = fot_ic_d->wx;
        fot_ic->wy = fot_ic_d->wy;
        fot_ic->o_llx = fot_ic_d->o_llx;
        fot_ic->o_lly = fot_ic_d->o_lly;
        fot_ic->o_urx = fot_ic_d->o_urx;
        fot_ic->o_ury = fot_ic_d->o_ury;
        // fot_ic->wx = new fixp_x[fot_ic->nw];
        // fot_ic->wy = new fixp_y[fot_ic->nw];
        // fot_ic->o_llx = new fixp_x[fot_ic->no];
        // fot_ic->o_lly = new fixp_y[fot_ic->no];
        // fot_ic->o_urx = new fixp_x[fot_ic->no];
        // fot_ic->o_ury = new fixp_y[fot_ic->no];

        // assignValueToFixedPoint<fixp_x>(fot_ic->wx, fot_ic_d->wx, fot_ic->nw);
        // assignValueToFixedPoint<fixp_y>(fot_ic->wy, fot_ic_d->wy, fot_ic->nw);
        // assignValueToFixedPoint<fixp_x>(fot_ic->o_llx, fot_ic_d->o_llx, fot_ic->no);
        // assignValueToFixedPoint<fixp_y>(fot_ic->o_lly, fot_ic_d->o_lly, fot_ic->no);
        // assignValueToFixedPoint<fixp_x>(fot_ic->o_urx, fot_ic_d->o_urx, fot_ic->no);
        // assignValueToFixedPoint<fixp_y>(fot_ic->o_ury, fot_ic_d->o_ury, fot_ic->no);        
        //-----------------FrenetHyperparameters--------------------------------
        fot_hp->max_speed = fot_hp_d->max_speed;
        fot_hp->max_accel = fot_hp_d->max_accel;
        fot_hp->max_curvature = fot_hp_d->max_curvature;
        fot_hp->max_road_width_l = fot_hp_d->max_road_width_l;
        fot_hp->max_road_width_r = fot_hp_d->max_road_width_r;
        fot_hp->d_road_w = fot_hp_d->d_road_w; 
        fot_hp->dt = fot_hp_d->dt;
        fot_hp->maxt = fot_hp_d->maxt;
        fot_hp->mint = fot_hp_d->mint;
        fot_hp->d_t_s = fot_hp_d->d_t_s;
        fot_hp->n_s_sample = fot_hp_d->n_s_sample;
        fot_hp->obstacle_clearance = fot_hp_d->obstacle_clearance;
        fot_hp->kd = fot_hp_d->kd;
        fot_hp->kv = fot_hp_d->kv;
        fot_hp->ka = fot_hp_d->ka;
        fot_hp->kj = fot_hp_d->kj;
        fot_hp->kt = fot_hp_d->kt;
        fot_hp->ko = fot_hp_d->ko;
        fot_hp->klat = fot_hp_d->klat;
        fot_hp->klon = fot_hp_d->klon;
        fot_hp->num_threads = fot_hp_d->num_threads;
        //----------------------------------------------------------------------

        clock_t start, end;
        start = clock();
        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic, fot_hp);
        end = clock();
        double time_taken = ((double)end-start)/CLOCKS_PER_SEC * 1000; // multiply by 1000 to convert to milliseconds
        
        #ifdef USE_RECORDER
            // write recorded data to csv file
            Recorder::getInstance()->writeDataToCSV();
        #endif
        
        FrenetPath* best_frenet_path = fot.getBestPath();
        // if(!best_frenet_path)
        // {
        //     std::cout << "!!Empty path returned" << std::endl;
        // }
        // else if(best_frenet_path->x.empty())
        // {
        //     std::cout << "!!there is not any x position in path" << std::endl;
        // }

        if (best_frenet_path && !best_frenet_path->x.empty()){
            fot_rv_d->success = 1;
            fot_rv_d->path_length = std::min(best_frenet_path->x.size(), MAX_PATH_LENGTH);
            for (size_t i = 0; i < fot_rv_d->path_length; i++) {
                fot_rv_d->x_path[i] = static_cast<double>(best_frenet_path->x[i]);
                fot_rv_d->y_path[i] = static_cast<double>(best_frenet_path->y[i]);
                fot_rv_d->speeds[i] = static_cast<double>(best_frenet_path->s_d[i]);
                fot_rv_d->ix[i] = static_cast<double>(best_frenet_path->ix[i]);
                fot_rv_d->iy[i] = static_cast<double>(best_frenet_path->iy[i]);
                fot_rv_d->iyaw[i] = static_cast<double>(best_frenet_path->iyaw[i]);
                fot_rv_d->d[i] = static_cast<double>(best_frenet_path->d[i]);
                fot_rv_d->s[i] = static_cast<double>(best_frenet_path->s[i]);
                fot_rv_d->speeds_x[i] = cos(best_frenet_path->yaw[i]) *
                    fot_rv_d->speeds[i];
                fot_rv_d->speeds_y[i] = sin(best_frenet_path->yaw[i]) *
                    fot_rv_d->speeds[i];
            }


            // store info for debug
            fot_rv_d->params[0] = static_cast<double>(best_frenet_path->s[1]);
            fot_rv_d->params[1] = static_cast<double>(best_frenet_path->s_d[1]);
            fot_rv_d->params[2] = static_cast<double>(best_frenet_path->d[1]);
            fot_rv_d->params[3] = static_cast<double>(best_frenet_path->d_d[1]);
            fot_rv_d->params[4] = static_cast<double>(best_frenet_path->d_dd[1]);

            // store costs for logging
            fot_rv_d->costs[0] = static_cast<double>(best_frenet_path->c_lateral_deviation);
            fot_rv_d->costs[1] = static_cast<double>(best_frenet_path->c_lateral_velocity);
            fot_rv_d->costs[2] = static_cast<double>(best_frenet_path->c_lateral_acceleration);
            fot_rv_d->costs[3] = static_cast<double>(best_frenet_path->c_lateral_jerk);
            fot_rv_d->costs[4] = static_cast<double>(best_frenet_path->c_lateral);
            fot_rv_d->costs[5] = static_cast<double>(best_frenet_path->c_longitudinal_acceleration);
            fot_rv_d->costs[6] = static_cast<double>(best_frenet_path->c_longitudinal_jerk);
            fot_rv_d->costs[7] = static_cast<double>(best_frenet_path->c_time_taken);
            fot_rv_d->costs[8] = static_cast<double>(best_frenet_path->c_end_speed_deviation);
            fot_rv_d->costs[9] = static_cast<double>(best_frenet_path->c_longitudinal);
            fot_rv_d->costs[10] = static_cast<double>(best_frenet_path->c_inv_dist_to_obstacles);
            fot_rv_d->costs[11] = static_cast<double>(best_frenet_path->cf);
        }
        fot_rv_d->runtime = time_taken;
        delete fot_hp;
        delete fot_ic;
    }

    // Convert the initial conditions from cartesian space to frenet space
    void to_frenet_initial_conditions(
            double s0, double x, double y, double vx,
            double vy, double forward_speed, double* xp, double* yp, int np,
            double* initial_conditions
            ) {
        vector<double> wx (xp, xp + np); //np=len(wx), see declartion of _to_frenet_initial_conditions() in fot_wrapper.py 
        vector<double> wy (yp, yp + np);
        CubicSpline2D* csp = new CubicSpline2D(wx, wy);

        // get distance from car to spline and projection
        double s = csp->find_s(x, y, s0);
        double distance = norm(csp->calc_x(s) - x, csp->calc_y(s) - y);
        tuple<double, double> bvec ((csp->calc_x(s) - x) / distance,
                (csp->calc_y(s) - y) / distance);

        // normal spline vector
        double x0 = csp->calc_x(s0);
        double y0 = csp->calc_y(s0);
        double x1 = csp->calc_x(s0 + 2);
        double y1 = csp->calc_y(s0 + 2);

        // unit vector orthog. to spline
        tuple<double, double> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // compute tangent / normal car vectors
        tuple<double, double> fvec (vx, vy);
        as_unit_vector(fvec);

        // get initial conditions in frenet frame
        initial_conditions[0] = s; // current longitudinal position s
        initial_conditions[1] = forward_speed; // speed [m/s]
        // lateral position c_d [m]
        initial_conditions[2] = copysign(distance, dot(tvec, bvec));
        // lateral speed c_d_d [m/s]
        initial_conditions[3] = -forward_speed * dot(tvec, fvec);
        initial_conditions[4] = 0.0; // lateral acceleration c_d_dd [m/s^2]
        // TODO: add lateral acceleration when CARLA 9.7 is patched (IMU)

        delete csp;
    }

    // template<typename T>
    // void assignValueToFixedPoint(T* output, double* input, int num)
    // {
    //     for(int i = 0; i < num; i++){
    //         output[i]=static_cast<T>(input);
    //     }
    // }
}
