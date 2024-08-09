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
    std::uniform_real_distribution<> distr_waypoint_tan(-10.0, 10.0);
    // std::uniform_real_distribution<> distr_t(0.0, 10.0);

    for(int i=0; i<size;i++)
    {
        angle_fx= distr_waypoint_angle(eng);
        angle = static_cast<float>(angle_fx);
        ans_fx = cordic_cos(angle_fx);
        ans=std::cos(angle);
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<float>("cos_fx", static_cast<float>(ans_fx));
            Recorder::getInstance()->saveData<float>("cos", ans);
            Recorder::getInstance()->saveData<float>("dert_cos", std::abs(static_cast<float>(ans_fx)-ans));
        #endif
        ans_fx = cordic_sin(angle_fx);
        ans=std::sin(angle);
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<float>("sin_fx", static_cast<float>(ans_fx));
            Recorder::getInstance()->saveData<float>("sin", ans);
            Recorder::getInstance()->saveData<float>("dert_sin", std::abs(static_cast<float>(ans_fx)-ans));
        #endif
        ans_fx = cordic_atan<Trignometric>(angle_fx, 1.0);
        ans=std::atan(angle);
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<float>("actan_fx", static_cast<float>(ans_fx));
            Recorder::getInstance()->saveData<float>("actan", ans);
            Recorder::getInstance()->saveData<float>("dert_actan", std::abs(static_cast<float>(ans_fx)-ans));
        #endif
    }
    
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
}