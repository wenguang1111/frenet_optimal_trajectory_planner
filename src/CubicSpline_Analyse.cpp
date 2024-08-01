#include "cnl/all.h"
#include "CubicSpline2D_fx.h"
#include "CubicSpline2D.h"
#include "CubicSpline1D_fx.h"
#include "CubicSpline1D.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <random>
#include <iostream>
#define size 10
#define waypoint_size 10

std::vector<float> getWaypoints_x()
{
    std::vector<float> positions;
    positions.push_back(0.0);
    positions.push_back(20.0);
    positions.push_back(35.0);

    for(int i=50;i<100;i++)
    {
        positions.push_back(i);
    }
    return positions;
}

std::vector<float> getWaypoints_y()
{
    std::vector<float> positions;
    positions.push_back(50.0);
    positions.push_back(50.0);
    positions.push_back(50.0);

    for(int i=50;i<100;i++)
    {
        positions.push_back(std::sqrt(2500.0-std::pow(i-50.0,2)));
    }
    return positions;
}

int main()
{
    std::vector<float> way_x,way_y;
    std::vector<fp_type> way_x_fx,way_y_fx;

    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    // Define the range
    std::uniform_real_distribution<> distr_waypoint_x(0.0, 10.0);
    std::uniform_real_distribution<> distr_waypoint_y(-10.0, 10.0);
    // std::uniform_real_distribution<> distr_t(0.0, 10.0);

    for(int i=0; i<size;i++)
    {
        way_x_fx.push_back(0.0);
        way_y_fx.push_back(0.0);
        for(int j=0;j<10;j++)
        {
            way_x_fx.push_back(way_x_fx.back() + distr_waypoint_x(eng));
            way_y_fx.push_back(way_y_fx.back() + distr_waypoint_y(eng));
            // t_fx[i] = 10;
            // way_x[j] += static_cast<float>(way_x_fx[j]);
            // way_y[j] += static_cast<float>(way_y_fx[j]);
            way_x.push_back(static_cast<float>(way_x_fx.back()));
            way_y.push_back(static_cast<float>(way_y_fx.back()));
        }
        CubicSpline2D a = CubicSpline2D(way_x,way_y);
        CubicSpline2D_fx b = CubicSpline2D_fx(way_x_fx,way_y_fx);
        
        fp_type dert = b.getEndOfS();
        float test = a.getEndOfS();
        std::cout << "s_fx=" << dert << std::endl;
        std::cout << "s=" << test << std::endl;


        // QuinticPolynomial_fx b = QuinticPolynomial_fx(xs_fx[i], vxs_fx[i], axs_fx[i], xe_fx[i], vxe_fx[i], axe_fx[i], t_fx[i]);
        // float dert_t = t[i]/100;
        // for(int j=0;j<100;j++)
        // {
        //     float time = dert_t*j;
        //     float d_s = std::abs(a.calc_point(time)-static_cast<float>(b.calc_point_fx(time)))/std::abs(a.calc_point(time));
        //     float d_v = std::abs(a.calc_first_derivative(time)-static_cast<float>(b.calc_first_derivative_fx(time)))/std::abs(a.calc_first_derivative(time));
        //     float d_a = std::abs(a.calc_second_derivative(time)-static_cast<float>(b.calc_second_derivative_fx(time)))/std::abs(a.calc_second_derivative(time));
        //     float d_jerk = std::abs(a.calc_third_derivative(time)-static_cast<float>(b.calc_third_derivative_fx(time)))/std::abs(a.calc_third_derivative(time));
        //     #ifdef USE_RECORDER
        //         Recorder::getInstance()->saveData<float>("d_0[%]", d_s*100);
        //     #endif
        // }
    }
    
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
}