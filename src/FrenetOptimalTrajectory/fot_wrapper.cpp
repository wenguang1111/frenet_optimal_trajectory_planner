#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "utils.h"
#include "tool/recorder.h"
#include "cordic.h"

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
    //      fot_ic (FrenetInitialConditions *):
    //          struct ptr containing relevant initial conditions to compute
    //          Frenet Optimal Trajectory
    //      fot_hp (FrenetHyperparameters *):
    //          struct ptr containing relevant hyperparameters to compute
    //          Frenet Optimal Trajectory
    //      x_path, y_path, speeds (float *):
    //          ptr to storage arrays for Frenet Optimal Trajectory
    //      params (float *):
    //          ptr to store initial conditions for debugging
    //
    // Returns:
    //      1 if successful, 0 if failure
    //      Also stores the Frenet Optimal Trajectory into x_path, y_path,
    //      speeds if it exists
    void run_fot(
            FrenetInitialConditions_Float *fot_ic_f, FrenetHyperparameters_Float *fot_hp_f,
            FrenetReturnValues_FLoat *fot_rv_f
            ) {
        FrenetHyperparameters_FP* fot_hp_fp = new FrenetHyperparameters_FP();
        FrenetInitialConditions_FP* fot_ic_fp = new FrenetInitialConditions_FP();
        //----------------------------------------------------------------------
        // data type convertion for interface
        //-----------------FrenetInitialConditions------------------------------
        fot_ic_fp->s0 = fot_ic_f->s0;
        fot_ic_fp->c_speed = fot_ic_f->c_speed;
        fot_ic_fp->c_d = fot_ic_f->c_d;
        fot_ic_fp->c_d_d = fot_ic_f->c_d_d;
        fot_ic_fp->c_d_dd = fot_ic_f->c_d_dd;
        fot_ic_fp->target_speed = fot_ic_f->target_speed;
        fot_ic_fp->nw = fot_ic_f->nw;
        fot_ic_fp->no = fot_ic_f->no;

        fot_ic_fp->wx = new fixp_x[fot_ic_f->nw];
        fot_ic_fp->wy = new fixp_y[fot_ic_f->nw];
        fot_ic_fp->o_llx = new fixp_x[fot_ic_f->no];
        fot_ic_fp->o_lly = new fixp_y[fot_ic_f->no];
        fot_ic_fp->o_urx = new fixp_x[fot_ic_f->no];
        fot_ic_fp->o_ury = new fixp_y[fot_ic_f->no];

        assignValueToFixedPoint<fixp_x>(fot_ic_fp->wx, fot_ic_f->wx, fot_ic_fp->nw);
        assignValueToFixedPoint<fixp_y>(fot_ic_fp->wy, fot_ic_f->wy, fot_ic_fp->nw);
        assignValueToFixedPoint<fixp_x>(fot_ic_fp->o_llx, fot_ic_f->o_llx, fot_ic_fp->no);
        assignValueToFixedPoint<fixp_y>(fot_ic_fp->o_lly, fot_ic_f->o_lly, fot_ic_fp->no);
        assignValueToFixedPoint<fixp_x>(fot_ic_fp->o_urx, fot_ic_f->o_urx, fot_ic_fp->no);
        assignValueToFixedPoint<fixp_y>(fot_ic_fp->o_ury, fot_ic_f->o_ury, fot_ic_fp->no);
          
        //-----------------FrenetHyperparameters--------------------------------
        fot_hp_fp->max_speed = fot_hp_f->max_speed;
        fot_hp_fp->max_accel = fot_hp_f->max_accel;
        fot_hp_fp->max_curvature = fot_hp_f->max_curvature;
        fot_hp_fp->max_road_width_l = fot_hp_f->max_road_width_l;
        fot_hp_fp->max_road_width_r = fot_hp_f->max_road_width_r;
        fot_hp_fp->d_road_w = fot_hp_f->d_road_w; 
        fot_hp_fp->dt = fot_hp_f->dt;
        fot_hp_fp->maxt = fot_hp_f->maxt;
        fot_hp_fp->mint = fot_hp_f->mint;
        fot_hp_fp->d_t_s = fot_hp_f->d_t_s;
        fot_hp_fp->n_s_sample = fot_hp_f->n_s_sample;
        fot_hp_fp->obstacle_clearance = fot_hp_f->obstacle_clearance;
        fot_hp_fp->kd = fot_hp_f->kd;
        fot_hp_fp->kv = fot_hp_f->kv;
        fot_hp_fp->ka = fot_hp_f->ka;
        fot_hp_fp->kj = fot_hp_f->kj;
        fot_hp_fp->kt = fot_hp_f->kt;
        fot_hp_fp->ko = fot_hp_f->ko;
        fot_hp_fp->klat = fot_hp_f->klat;
        fot_hp_fp->klon = fot_hp_f->klon;
        fot_hp_fp->num_threads = fot_hp_f->num_threads;
        //----------------------------------------------------------------------
        clock_t start, end;
        start = clock();
        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(fot_ic_fp, fot_hp_fp);
        end = clock();
        float time_taken = ((float)end-start)/CLOCKS_PER_SEC * 1000; // multiply by 1000 to convert to milliseconds
        
        #ifdef USE_RECORDER
            // write recorded data to csv file
            Recorder::getInstance()->writeDataToCSV();
        #endif
        FrenetPath* best_frenet_path = fot.getBestPath();
        if (best_frenet_path && !best_frenet_path->x.empty()){
            fot_rv_f->success = 1;
            fot_rv_f->path_length = std::min(best_frenet_path->x.size(), MAX_PATH_LENGTH);
            for (size_t i = 0; i < fot_rv_f->path_length; i++) {
                fot_rv_f->x_path[i] = static_cast<float>(best_frenet_path->x[i]);
                fot_rv_f->y_path[i] = static_cast<float>(best_frenet_path->y[i]);
                fot_rv_f->speeds[i] = static_cast<float>(best_frenet_path->s_d[i]);
                fot_rv_f->ix[i] = static_cast<float>(best_frenet_path->ix[i]);
                fot_rv_f->iy[i] = static_cast<float>(best_frenet_path->iy[i]);
                fot_rv_f->iyaw[i] = static_cast<float>(best_frenet_path->iyaw[i]);
                fot_rv_f->d[i] = static_cast<float>(best_frenet_path->d[i]);
                fot_rv_f->s[i] = static_cast<float>(best_frenet_path->s[i]);
                fot_rv_f->speeds_x[i] = cos(best_frenet_path->yaw[i]) *
                    fot_rv_f->speeds[i];
                fot_rv_f->speeds_y[i] = sin(best_frenet_path->yaw[i]) *
                    fot_rv_f->speeds[i];
                // #ifdef USE_RECORDER
                //     Recorder::getInstance()->saveData<float>("index", float(i));
                //     Recorder::getInstance()->saveData<float>("time", static_cast<float>(best_frenet_path->t[i]));
                //     Recorder::getInstance()->saveData<float>("x", static_cast<float>(best_frenet_path->x[i]));
                //     Recorder::getInstance()->saveData<float>("y", static_cast<float>(best_frenet_path->y[i]));
                //     Recorder::getInstance()->saveData<float>("d", static_cast<float>(best_frenet_path->d[i]));
                //     Recorder::getInstance()->saveData<float>("d_d", static_cast<float>(best_frenet_path->d_d[i]));
                //     Recorder::getInstance()->saveData<float>("d_dd", static_cast<float>(best_frenet_path->d_dd[i]));
                //     Recorder::getInstance()->saveData<float>("d_ddd", static_cast<float>(best_frenet_path->d_ddd[i]));
                //     Recorder::getInstance()->saveData<float>("s", static_cast<float>(best_frenet_path->s[i]));
                //     Recorder::getInstance()->saveData<float>("s_d", static_cast<float>(best_frenet_path->s_d[i]));
                //     Recorder::getInstance()->saveData<float>("s_dd", static_cast<float>(best_frenet_path->s_dd[i]));
                //     Recorder::getInstance()->saveData<float>("s_ddd", static_cast<float>(best_frenet_path->s_ddd[i]));
                //     Recorder::getInstance()->saveData<float>("yaw", static_cast<float>(best_frenet_path->yaw[i]));
                //     Recorder::getInstance()->saveData<float>("ix", static_cast<float>(best_frenet_path->ix[i]));
                //     Recorder::getInstance()->saveData<float>("iy", static_cast<float>(best_frenet_path->iy[i]));
                //     Recorder::getInstance()->saveData<float>("iyaw", static_cast<float>(best_frenet_path->iyaw[i]));
                //     Recorder::getInstance()->saveData<float>("ds", static_cast<float>(best_frenet_path->ds[i]));
                //     Recorder::getInstance()->saveData<float>("c", static_cast<float>(best_frenet_path->c[i]));
                //     Recorder::getInstance()->saveData<float>("speed_x", fot_rv_f->speeds_x[i]);
                //     Recorder::getInstance()->saveData<float>("speed_y", fot_rv_f->speeds_y[i]);
                // #endif
            }

            // store info for debug
            fot_rv_f->params[0] = static_cast<float>(best_frenet_path->s[1]);
            fot_rv_f->params[1] = static_cast<float>(best_frenet_path->s_d[1]);
            fot_rv_f->params[2] = static_cast<float>(best_frenet_path->d[1]);
            fot_rv_f->params[3] = static_cast<float>(best_frenet_path->d_d[1]);
            fot_rv_f->params[4] = static_cast<float>(best_frenet_path->d_dd[1]);

            // store costs for logging
            fot_rv_f->costs[0] = static_cast<float>(best_frenet_path->c_lateral_deviation);
            fot_rv_f->costs[1] = static_cast<float>(best_frenet_path->c_lateral_velocity);
            fot_rv_f->costs[2] = static_cast<float>(best_frenet_path->c_lateral_acceleration);
            fot_rv_f->costs[3] = static_cast<float>(best_frenet_path->c_lateral_jerk);
            fot_rv_f->costs[4] = static_cast<float>(best_frenet_path->c_lateral);
            fot_rv_f->costs[5] = static_cast<float>(best_frenet_path->c_longitudinal_acceleration);
            fot_rv_f->costs[6] = static_cast<float>(best_frenet_path->c_longitudinal_jerk);
            fot_rv_f->costs[7] = static_cast<float>(best_frenet_path->c_time_taken);
            fot_rv_f->costs[8] = static_cast<float>(best_frenet_path->c_end_speed_deviation);
            fot_rv_f->costs[9] = static_cast<float>(best_frenet_path->c_longitudinal);
            fot_rv_f->costs[10] = static_cast<float>(best_frenet_path->c_inv_dist_to_obstacles);
            fot_rv_f->costs[11] = static_cast<float>(best_frenet_path->cf);
        }
        fot_rv_f->runtime = time_taken;
        delete fot_ic_fp->wx;
        delete fot_ic_fp->wy;
        delete fot_ic_fp->o_llx;
        delete fot_ic_fp->o_lly;
        delete fot_ic_fp->o_urx;
        delete fot_ic_fp->o_ury;
    }

    // Convert the initial conditions from cartesian space to frenet space
    void to_frenet_initial_conditions(
            float s0, float x, float y, float vx,
            float vy, float forward_speed, float* xp, float* yp, int np,
            float* initial_conditions
            ) {
        fixp_x* xp_fp = new fixp_x[np];
        fixp_y* yp_fp = new fixp_y[np];
        assignValueToFixedPoint<fixp_x>(xp_fp, xp, np);
        assignValueToFixedPoint<fixp_y>(yp_fp, yp, np);
        vector<fixp_x> wx (xp_fp, xp_fp + np); //np=len(wx), see declartion of _to_frenet_initial_conditions() in fot_wrapper.py 
        vector<fixp_y> wy (yp_fp, yp_fp + np);
        CubicSpline2D* csp = new CubicSpline2D(wx, wy);
        // get distance from car to spline and projection
        fixp_x x_fp = x;
        fixp_y y_fp = y;
        fixp_s s0_fp = s0;
        fixp_s s_fp = csp->find_s(x_fp, y_fp, s0_fp);
        fixp_x distance = norm(csp->calc_x(s_fp) - x_fp, csp->calc_y(s_fp) - y_fp);
        tuple<float, float> bvec ((static_cast<float>(csp->calc_x(s_fp) - x_fp) / distance),
                (static_cast<float>(csp->calc_y(s_fp) - y_fp) / distance));
        // normal spline vector
        float x0 = static_cast<float>(csp->calc_x(s0_fp));
        float y0 = static_cast<float>(csp->calc_y(s0_fp));
        float x1 = static_cast<float>(csp->calc_x(s0_fp + 2));
        float y1 = static_cast<float>(csp->calc_y(s0_fp + 2));
        // unit vector orthog. to spline
        tuple<float, float> tvec (y1-y0, -(x1-x0));
        as_unit_vector(tvec);

        // compute tangent / normal car vectors
        tuple<float, float> fvec (vx, vy);
        as_unit_vector(fvec);
        // get initial conditions in frenet frame
        initial_conditions[0] = static_cast<float>(s_fp); // current longitudinal position s
        initial_conditions[1] = forward_speed; // speed [m/s]
        // lateral position c_d [m]
        initial_conditions[2] = copysign(static_cast<float>(distance), dot(tvec, bvec));
        // lateral speed c_d_d [m/s]
        initial_conditions[3] = -forward_speed * dot(tvec, fvec);
        initial_conditions[4] = 0.0; // lateral acceleration c_d_dd [m/s^2]
        // TODO: add lateral acceleration when CARLA 9.7 is patched (IMU)

        delete csp;
        delete xp_fp;
        delete yp_fp;
    }
}
