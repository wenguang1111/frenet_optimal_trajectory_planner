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
#define loop_n 1000
#define waypoint_size 10
#define N 100
#define Distance_x 50
#define delta_distance 0.25


struct POS
{
    std::vector<fp_type> x;
    std::vector<fp_type> y;
};

fp_type calculateY_type_one(fp_type x)
{
    float y = sqrt(100.0 - pow(static_cast<float>(x),2));
    fp_type ans=y;
    return ans;
}

fp_type calculateY_type_two(fp_type x)
{
    float y = -sqrt(100.0 - pow(static_cast<float>(x-20),2));
    fp_type ans=y;
    return ans;
}

fp_type calculateY_type_three(fp_type x)
{
    float y = -sqrt(100.0 - pow(static_cast<float>(x-30),2));
    fp_type ans=y;
    return ans;
}

fp_type calculateY_type_four(fp_type x)
{
    float y = sqrt(100.0 - pow(static_cast<float>(x-50),2));
    fp_type ans=y;
    return ans;
}

POS calculateWayPoint()
{
    POS ans;
    for(float i=0;i<10;i+=delta_distance)
    {
        ans.x.push_back(i);
        ans.y.push_back(calculateY_type_one(i));
    }
    for(float i=10;i<20;i+=delta_distance)
    {
        ans.x.push_back(i);
        ans.y.push_back(calculateY_type_two(i));
    }
    for(float i=20;i<30;i+=delta_distance)
    {
        ans.x.push_back(i);
        ans.y.push_back(-10);
    }
    for(float i=30;i<40;i+=delta_distance)
    {
        ans.x.push_back(i);
        ans.y.push_back(calculateY_type_three(i));
    }
    for(float i=40;i<=50;i+=delta_distance)
    {
        ans.x.push_back(i);
        ans.y.push_back(calculateY_type_four(i));
    }
    return ans;
}

// std::vector<float> getWaypoints_x()
// {
//     std::vector<float> positions;
//     positions.push_back(0.0);
//     positions.push_back(10.0);
//     positions.push_back(20.0);

//     for(int i=50;i<100;i++)
//     {
//         positions.push_back(i);
//     }
//     return positions;
// }

// std::vector<float> getWaypoints_y()
// {
//     std::vector<float> positions;
//     positions.push_back(50.0);
//     positions.push_back(50.0);
//     positions.push_back(50.0);

//     for(int i=50;i<100;i++)
//     {
//         positions.push_back(std::sqrt(2500.0-std::pow(i-50.0,2)));
//     }
//     return positions;
// }

int main()
{


    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    // Define the range
    // std::uniform_real_distribution<> distr_waypoint_x(0.0, 5.0);
    // std::uniform_real_distribution<> distr_waypoint_y(-10.0, 10.0);
    // std::uniform_real_distribution<> distr_t(0.0, 10.0);

    for(int i=0; i<loop_n;i++)
    {
        std::vector<float> way_x,way_y;
        std::vector<fp_type> way_x_fx,way_y_fx;
        POS wp_fx =calculateWayPoint();
        
        way_x_fx=wp_fx.x;
        way_y_fx=wp_fx.y;

        for(int j=0;j<way_x_fx.size();j++)
        {
            way_x.push_back(static_cast<float>(way_x_fx[j]));
            way_y.push_back(static_cast<float>(way_y_fx[j]));
            #ifdef USE_RECORDER
                    Recorder::getInstance()->saveData<float>("way_x", static_cast<float>(way_x.back()));
                    Recorder::getInstance()->saveData<float>("way_y", static_cast<float>(way_y.back()));
            #endif
        }
        CubicSpline2D a = CubicSpline2D(way_x,way_y);
        CubicSpline2D_fx b = CubicSpline2D_fx(way_x_fx,way_y_fx);

        fp_type s_total = std::min(static_cast<float>(b.getEndOfS()), a.getEndOfS());
        fp_type dert = (s_total-1)/N;
        if(dert==0)
        {
            continue;
        }
        fp_type start = 1.0;
        while(start<s_total)
        {
            float x = a.calc_x(static_cast<float>(start));
            float y = a.calc_y(static_cast<float>(start));
            float yaw =a.calc_yaw(static_cast<float>(start));
            fp_type x_fx = b.calc_x(start);
            fp_type y_fx = b.calc_y(start);
            fp_type yaw_fx = b.calc_yaw(start);

                #ifdef USE_RECORDER
                    Recorder::getInstance()->saveData<float>("time", static_cast<float>(start));
                    Recorder::getInstance()->saveData<float>("delta_time", static_cast<float>(dert));
                    Recorder::getInstance()->saveData<float>("x", x);
                    Recorder::getInstance()->saveData<float>("y", y);
                    Recorder::getInstance()->saveData<float>("yaw", yaw);
                    Recorder::getInstance()->saveData<float>("x_fx", static_cast<float>(x_fx));
                    Recorder::getInstance()->saveData<float>("y_fx", static_cast<float>(y_fx));
                    Recorder::getInstance()->saveData<float>("yaw_fx", static_cast<float>(yaw_fx));
                    Recorder::getInstance()->saveData<float>("zelt_x", std::abs(static_cast<float>(x_fx)-x));
                    Recorder::getInstance()->saveData<float>("zelt_y", std::abs(static_cast<float>(y_fx)-y));
                    Recorder::getInstance()->saveData<float>("zelt_yaw", std::abs(static_cast<float>(yaw_fx)-yaw));
                #endif

            start+=dert;
        }

        // for(int i=0;i<b.getX_D().size();i++)
        // {
        //     #ifdef USE_RECORDER
        //         Recorder::getInstance()->saveData<float>("X_A", a.getX_A()[i]);
        //         Recorder::getInstance()->saveData<float>("X_B", a.getX_B()[i]);
        //         Recorder::getInstance()->saveData<float>("X_C", a.getX_C()[i]);
        //         Recorder::getInstance()->saveData<float>("X_D", a.getX_D()[i]);
        //         Recorder::getInstance()->saveData<float>("X_X", a.getX_X()[i]);
        //         Recorder::getInstance()->saveData<float>("X_Y", a.getX_Y()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_A", a.getY_A()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_B", a.getY_B()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_C", a.getY_C()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_D", a.getY_D()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_X", a.getY_X()[i]);
        //         Recorder::getInstance()->saveData<float>("Y_Y", a.getY_Y()[i]);
        //         Recorder::getInstance()->saveData<float>("X_A_fx", static_cast<float>(b.getX_A()[i]));
        //         Recorder::getInstance()->saveData<float>("X_B_fx", static_cast<float>(b.getX_B()[i]));
        //         Recorder::getInstance()->saveData<float>("X_C_fx", static_cast<float>(b.getX_C()[i]));
        //         Recorder::getInstance()->saveData<float>("X_D_fx", static_cast<float>(b.getX_D()[i]));
        //         Recorder::getInstance()->saveData<float>("X_X_fx", static_cast<float>(b.getX_X()[i]));
        //         Recorder::getInstance()->saveData<float>("X_Y_fx", static_cast<float>(b.getX_Y()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_A_fx", static_cast<float>(b.getY_A()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_B_fx", static_cast<float>(b.getY_B()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_C_fx", static_cast<float>(b.getY_C()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_D_fx", static_cast<float>(b.getY_D()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_X_fx", static_cast<float>(b.getY_X()[i]));
        //         Recorder::getInstance()->saveData<float>("Y_Y_fx", static_cast<float>(b.getY_Y()[i]));

        //         Recorder::getInstance()->saveData<float>("d_X_A", std::abs(a.getX_A()[i]-static_cast<float>(b.getX_A()[i])));
        //         Recorder::getInstance()->saveData<float>("d_X_B", std::abs(a.getX_B()[i]-static_cast<float>(b.getX_B()[i])));
        //         Recorder::getInstance()->saveData<float>("d_X_C", std::abs(a.getX_C()[i]-static_cast<float>(b.getX_C()[i])));
        //         Recorder::getInstance()->saveData<float>("d_X_D", std::abs(a.getX_D()[i]-static_cast<float>(b.getX_D()[i])));
        //         Recorder::getInstance()->saveData<float>("d_X_X", std::abs(a.getX_X()[i]-static_cast<float>(b.getX_X()[i])));
        //         Recorder::getInstance()->saveData<float>("d_X_Y", std::abs(a.getX_Y()[i]-static_cast<float>(b.getX_Y()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_A", std::abs(a.getY_A()[i]-static_cast<float>(b.getY_A()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_B", std::abs(a.getY_B()[i]-static_cast<float>(b.getY_B()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_C", std::abs(a.getY_C()[i]-static_cast<float>(b.getY_C()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_D", std::abs(a.getY_D()[i]-static_cast<float>(b.getY_D()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_X", std::abs(a.getY_X()[i]-static_cast<float>(b.getY_X()[i])));
        //         Recorder::getInstance()->saveData<float>("d_Y_Y", std::abs(a.getY_Y()[i]-static_cast<float>(b.getY_Y()[i])));
        //     #endif
        // }
    }
    
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
}