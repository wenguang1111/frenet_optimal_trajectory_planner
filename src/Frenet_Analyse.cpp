#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>
#include <random>

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
#include <cmath>
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

using namespace std;
#define delta_distance 0.25
#define time_interval 0.1
#define TARGET_SPEED 50
#define DISTANCE 50
#define START_SPEED 0

// fp_type calculateY_type(fp_type x, fp_type displacement)
// {
//     float y = sqrt(100.0 - pow(static_cast<float>(x-displacement),2));
//     fp_type ans=y;
//     return ans;
// }

struct POS
{
    std::vector<fp_type> x;
    std::vector<fp_type> y;
};

POS calculateWayPoint(float displacement_referencepath)
{
    POS ans;
    float distance = DISTANCE;
    for(float i=0;i<distance;i+=delta_distance)
    {
        QuinticPolynomial* test = new QuinticPolynomial(0, 0 , 0, displacement_referencepath,0,0,distance);
        ans.x.push_back(i);
        ans.y.push_back(static_cast<fp_type>(test->calc_point(i)));
    }

    return ans;
}

void generatePath(float long_v_1,float long_v_2, float displacement_referencepath) {
    float t, ti, tv;
    FrenetPath *fp, *tfp;
    FrenetPath_fx *fp_fx, *tfp_fx;
    int num_paths = 0;
    int num_viable_paths = 0;
    float max_road_width_l=10.0;
    float max_road_width_r=10.0;
    float d_road_w = 0.2;
    int start_di_index = 0;
     int end_di_index =(max_road_width_l+max_road_width_r)/d_road_w;
    float maxt = 2*DISTANCE/(long_v_1+long_v_2);
    float mint = maxt;
    //-------------Parameters-----------------------
    float c_d=0;
    float c_d_d =0;
    float c_d_dd=0;
    float target_speed = long_v_2;
    float s0=0;
    float c_speed=long_v_1;
    //----------------------------------------
    std::vector<float> wp_x;
    std::vector<float> wp_y;
    std::vector<fp_type> wp_x_fx;
    std::vector<fp_type> wp_y_fx;
    wp_x_fx = calculateWayPoint(displacement_referencepath).x;
    wp_y_fx = calculateWayPoint(displacement_referencepath).y;
    for(int i=0; i<wp_x_fx.size();i++)
    {
        wp_x.push_back(static_cast<float>(wp_x_fx[i]));
        wp_y.push_back(static_cast<float>(wp_y_fx[i]));
    }
    #ifdef USE_RECORDER
        for(int i=0; i<wp_x_fx.size();i++)
        {
            Recorder::getInstance()->saveData<float>("WP_x",wp_x[i]);
            Recorder::getInstance()->saveData<float>("WP_y",wp_y[i]);
        }
    #endif 

    CubicSpline2D* cubicspline = new CubicSpline2D(wp_x, wp_y);
    CubicSpline2D_fx* cubicspline_fx = new CubicSpline2D_fx(wp_x_fx, wp_y_fx);
    //---------------------------------------
    FrenetHyperparameters fot_hp;
    FrenetHyperparameters_fx fot_hp_fx;
    fot_hp.max_speed=400.0;
    fot_hp.max_accel=500.0;
    fot_hp.max_curvature=500.0;
    fot_hp.max_road_width_l=max_road_width_l;
    fot_hp.max_road_width_r=max_road_width_r;
    fot_hp.d_road_w=d_road_w;
    fot_hp.dt=time_interval;
    fot_hp.maxt=maxt;
    fot_hp.mint=maxt;
    fot_hp.d_t_s=5;
    fot_hp.n_s_sample=0;

    fot_hp_fx.max_speed=fot_hp.max_speed;
    fot_hp_fx.max_accel=fot_hp.max_accel;
    fot_hp_fx.max_curvature=fot_hp.max_curvature;
    fot_hp_fx.max_road_width_l=fot_hp.max_road_width_l;
    fot_hp_fx.max_road_width_r=fot_hp.max_road_width_r;
    fot_hp_fx.d_road_w=fot_hp.d_road_w;
    fot_hp_fx.dt=fot_hp.dt;
    fot_hp_fx.maxt=fot_hp.maxt;
    fot_hp_fx.mint=fot_hp.mint;
    fot_hp_fx.d_t_s=fot_hp.d_t_s;
    fot_hp_fx.n_s_sample=fot_hp.n_s_sample;
    // float valid_path_time = 0;

    // initialize di, with start_di_index
    float di = -max_road_width_l + start_di_index * d_road_w;

    while ((di < -max_road_width_l + end_di_index * d_road_w) &&
           (di <= max_road_width_r)) {
        ti = mint;
        #ifdef USE_RECORDER
                Recorder::getInstance()->saveData<float>("ti",ti);
        #endif
        while (ti <= maxt) {
            fp = new FrenetPath(&fot_hp);
            fp_fx = new FrenetPath_fx(&fot_hp_fx);
 
            #ifdef USE_RECORDER
                Recorder::getInstance()->saveData<float>("generatePath::generatePath::ti", ti);
            #endif 

            QuinticPolynomial lat_qp = QuinticPolynomial(
                c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);
            QuinticPolynomial_fx lat_qp_fx = QuinticPolynomial_fx(
                c_d, c_d_d, c_d_dd, di, 0.0, 0.0, ti);

            // construct frenet path
            t = 0;
            while (t <= ti) {
                fp->t.push_back(t);
                fp->d.push_back(lat_qp.calc_point(t));
                fp->d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp->d_ddd.push_back(lat_qp.calc_third_derivative(t));
                fp_fx->t.push_back(t);
                fp_fx->d.push_back(lat_qp_fx.calc_point_fx(t));
                fp_fx->d_d.push_back(lat_qp_fx.calc_first_derivative_fx(t));
                fp_fx->d_dd.push_back(lat_qp_fx.calc_second_derivative_fx(t));
                fp_fx->d_ddd.push_back(lat_qp_fx.calc_third_derivative_fx(t));
                t += fot_hp.dt;
            }
            
            // velocity keeping
            tv = long_v_2;
            while (tv <=
                   target_speed + fot_hp.d_t_s * fot_hp.n_s_sample) {
                // copy frenet path
                tfp = new FrenetPath(&fot_hp);
                tfp->t.assign(fp->t.begin(), fp->t.end());
                tfp->d.assign(fp->d.begin(), fp->d.end());
                tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
                tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
                tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());

                tfp_fx = new FrenetPath_fx(&fot_hp_fx);
                tfp_fx->t.assign(fp_fx->t.begin(), fp_fx->t.end());
                tfp_fx->d.assign(fp_fx->d.begin(), fp_fx->d.end());
                tfp_fx->d_d.assign(fp_fx->d_d.begin(), fp_fx->d_d.end());
                tfp_fx->d_dd.assign(fp_fx->d_dd.begin(), fp_fx->d_dd.end());
                tfp_fx->d_ddd.assign(fp_fx->d_ddd.begin(), fp_fx->d_ddd.end());

                #ifdef USE_RECORDER
                    Recorder::getInstance()->saveData<float>("generatePath::generatePath::ti", ti);
                #endif 
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
                bool success = tfp->to_global_path(cubicspline);
                bool success_fx = tfp_fx->to_global_path(cubicspline_fx);
                
                if (!success ||!success_fx) {
                    // deallocate memory and continue
                    delete tfp;
                    delete tfp_fx;
                    tv += fot_hp.d_t_s;
                    continue;
                }
                num_viable_paths++;

                // std::cout << "tfp->t size = "<< tfp->t.size()<<std::endl;
                int length = tfp->t.size();
                for(int k=0; k<length; k++)
                {
                    #ifdef USE_RECORDER
                        Recorder::getInstance()->saveData<float>("k",k);
                        Recorder::getInstance()->saveData<float>("time",tfp->t[k]);
                        Recorder::getInstance()->saveData<float>("s",tfp->s[k]);
                        Recorder::getInstance()->saveData<float>("x",tfp->x[k]);
                        Recorder::getInstance()->saveData<float>("y",tfp->y[k]);
                        Recorder::getInstance()->saveData<float>("yaw",tfp->yaw[k]);
                        Recorder::getInstance()->saveData<float>("x_fx",static_cast<float>(tfp_fx->x[k]));
                        Recorder::getInstance()->saveData<float>("y_fx",static_cast<float>(tfp_fx->y[k]));
                        Recorder::getInstance()->saveData<float>("yaw_fx",static_cast<float>(tfp_fx->yaw[k]));
                        Recorder::getInstance()->saveData<float>("zelt_x",std::abs(tfp->x[k]-static_cast<float>(tfp_fx->x[k])));
                        Recorder::getInstance()->saveData<float>("zelt_y",std::abs(tfp->y[k]-static_cast<float>(tfp_fx->y[k])));
                        Recorder::getInstance()->saveData<float>("zelt_yaw",std::abs(tfp->yaw[k]-static_cast<float>(tfp_fx->yaw[k])));
                    #endif 
                }

                tv += fot_hp.d_t_s;
                delete tfp;
            }
            ti += fot_hp.dt;
            delete fp;
        }
        di += fot_hp.d_road_w;
    }
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
    // std::cout << "total path number is" <<num_viable_paths<<std::endl;
}

int main()
{
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    std::uniform_real_distribution<> lateral_position(-50.0, 50.0);
    std::uniform_real_distribution<> long_velocity(10.0, 50.0);

    for(int i=0; i<1; i++)
    {
        float sv_1 = long_velocity(eng);
        float sv_2 = long_velocity(eng);
        float displacement = lateral_position(eng);

        generatePath(sv_1, sv_2, displacement);
    }

}