#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>

#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "QuarticPolynomial_fx.h"
#include "QuinticPolynomial_fx.h"
#include "CubicSpline1D.h"
#include "CubicSpline2D.h"
#include "CubicSpline1D_fx.h"
#include "CubicSpline2D_fx.h"
#include "TriDiagonalMatrixSolver.h"
#include "TriDiagonalMatrixSolver_fx.h"
#include "FrenetPath_fx.h"
#include "FrenetPath.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

using namespace std;

int main() {
    float t, ti, tv;
    float lateral_deviation, lateral_velocity, lateral_acceleration,
        lateral_jerk;
    float longitudinal_acceleration, longitudinal_jerk;
    FrenetPath *fp, *tfp;
    FrenetPath_fx *fp_fx, *tfp_fx;
    CubicSpline2D 
    int num_paths = 0;
    int num_viable_paths = 0;
    int start_di_index = 0;
    int end_di_index =24;
    float max_road_width_l=6.0;
    float max_road_width_r=6.0;
    float d_road_w = 0.5;
    float maxt = 5;
    float mint = 2;
    //-------------Parameters-----------------------
    float c_d=0;
    float c_d_d =0;
    float c_d_dd=0;
    float target_speed = 50;
    float s0=0;
    float c_speed=0;
    //----------------------------------------
    std::vector<float> x;
    std::vector<float> y;
    x.push_back(20.0);
    x.push_back(40.0);
    x.push_back(60.0);
    y.push_back(0.0);
    y.push_back(0.0);
    y.push_back(0.0);
    CubicSpline2D* csp = new CubicSpline2D(x, y);
    CubicSpline2D_fx* csp_fx = new CubicSpline2D_fx(x, y);
    //---------------------------------------
    FrenetHyperparameters fot_hp;
    FrenetHyperparameters_fx fot_hp_fx;
    fot_hp.max_speed=25.0
    fot_hp.max_accel=15.0
    fot_hp.max_curvature=15.0
    fot_hp.max_road_width_l=6.0
    fot_hp.max_road_width_r=6.0
    fot_hp.d_road_w=0.5
    fot_hp.dt=0.5
    fot_hp.maxt=5
    fot_hp.mint=2
    fot_hp.d_t_s=0.1
    fot_hp.n_s_sample=2

    fot_hp_fx.max_speed=25.0
    fot_hp_fx.max_accel=15.0
    fot_hp_fx.max_curvature=15.0
    fot_hp_fx.max_road_width_l=6.0
    fot_hp_fx.max_road_width_r=6.0
    fot_hp_fx.d_road_w=0.5
    fot_hp_fx.dt=0.5
    fot_hp_fx.maxt=5
    fot_hp_fx.mint=2
    fot_hp_fx.d_t_s=0.1
    fot_hp_fx.n_s_sample=2
    // float valid_path_time = 0;

    // initialize di, with start_di_index
    float di = max_road_width_l> + start_di_index * d_road_w;

    while ((di < max_road_width_l + end_di_index * d_road_w) &&
           (di <= max_road_width_r)) {
        ti = mint;

        while (ti <= maxt) {
            lateral_deviation = 0;
            lateral_velocity = 0;
            lateral_acceleration = 0;
            lateral_jerk = 0;

            fp = new FrenetPath(&fot_hp);
            fp_fx = new FrenetPath_fx(&fot_hp_fx);

            QuinticPolynomial lat_qp = QuinticPolynomial(
                c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);
            QuinticPolynomial_fx lat_qp_fx = QuinticPolynomial_fx(
                c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);
        
            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_d.push_back(lat_qp.calc_first_derivative(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                fp_fx->t.push_back(t);
                fp_fx->d.push_back(lat_qp_fx.calc_point(t));
                fp_fx->d_d.push_back(lat_qp_fx.calc_first_derivative(t));
                fp_fx->d_dd.push_back(lat_qp_fx.calc_second_derivative(t));
                fp_fx->d_ddd.push_back(lat_qp_fx.calc_third_derivative(t));
                t += fot_hp->dt;
            }

            // velocity keeping
            tv = target_speed - fot_hp->d_t_s * fot_hp->n_s_sample;
            while (tv <=
                   target_speed + fot_hp->d_t_s * fot_hp->n_s_sample) {
                longitudinal_acceleration = 0;
                longitudinal_jerk = 0;

                // copy frenet path
                tfp = new FrenetPath(fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());

                tfp_fx = new FrenetPath(fot_hp_fx);
                tfp_fx->t.assign(fp_fx->t.begin(), fp_fx->t.end());
                tfp_fx->d.assign(fp_fx->d.begin(), fp_fx->d.end());
                tfp_fx->d_d.assign(fp_fx->d_d.begin(), fp_fx->d_d.end());
                tfp_fx->d_dd.assign(fp_fx->d_dd.begin(), fp_fx->d_dd.end());
                tfp_fx->d_ddd.assign(fp_fx->d_ddd.begin(), fp_fx->d_ddd.end());

                QuarticPolynomial lon_qp = QuarticPolynomial(
                    s0, c_speed, 0.0, tv, 0.0, ti);
                QuarticPolynomial_fx lon_qp_fx = QuarticPolynomial_fx(
                    s0, c_speed, 0.0, tv, 0.0, ti);
                
                // longitudinal motion
                for (float tp : tfp->t) {
                    tfp->s.push_back(lon_qp.calc_point(tp));
                    tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));
                    tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp));
                    tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp));
                    
                    tfp_fx->s.push_back(lon_qp_fx.calc_point(tp));
                    tfp_fx->s_d.push_back(lon_qp_fx.calc_first_derivative(tp));
                    tfp_fx->s_dd.push_back(lon_qp_fx.calc_second_derivative(tp));
                    tfp_fx->s_ddd.push_back(lon_qp_fx.calc_third_derivative(tp));
                }
                num_paths++;
                // delete if failure or invalid path
                Trajectory traj = tfp->to_global_path(csp);
                Trajectory_fx traj_fx = tfp_fx->to_global_path(csp_fx);        
                num_viable_paths++;
                if (!success) {
                    // deallocate memory and continue
                    delete tfp;
                    tv += fot_hp->d_t_s;
                    continue;
                }

                tv += fot_hp->d_t_s;
            }
            ti += fot_hp->dt;
            // make sure to deallocate
            delete fp;
        }
        di += fot_hp->d_road_w;
    }
}