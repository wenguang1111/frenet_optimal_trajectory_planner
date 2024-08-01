#include "cnl/all.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <random>
#include <iostream>
#define size 10000
#define waypoint_size 10


int main()
{
    float angle;
    Trignometric angle_fx;
    float ans;
    Trignometric ans_fx;

    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    // Define the range
    std::uniform_real_distribution<> distr_waypoint_angle(-20.0, 20.0);
    // std::uniform_real_distribution<> distr_t(0.0, 10.0);

    for(int i=0; i<size;i++)
    {
        angle_fx= distr_waypoint_angle(eng);
        angle = static_cast<float>(angle_fx);
        ans_fx = cordic_cos(angle_fx);
        ans=std::cos(angle);
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<float>("ans_fx", static_cast<float>(ans_fx));
            Recorder::getInstance()->saveData<float>("ans", ans);
            Recorder::getInstance()->saveData<float>("dert", std::abs(static_cast<float>(ans_fx)-ans));
        #endif
    }
    
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
}