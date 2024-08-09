#include "cnl/all.h"
#include "QuarticPolynomial.h"
#include "QuarticPolynomial_fx.h"
#include "tool/fp_datatype.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <random>
#include <iostream>
#define size 1000
#define Timelength 5

 
int main()
{
    float xs[size],vxs[size],axs[size],vxe[size],axe[size],t[size];
    fp_type xs_fx[size],vxs_fx[size],axs_fx[size],vxe_fx[size],axe_fx[size],t_fx[size];

    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    // Define the range
    std::uniform_real_distribution<> distr_xs(-100.0, 100.0);
    std::uniform_real_distribution<> distr_vxs(-300.0, 300.0);
    std::uniform_real_distribution<> distr_axs(-50.0, 50.0);
    // std::uniform_real_distribution<> distr_xs(0.0, 100.0);
    // std::uniform_real_distribution<> distr_vxs(0.0, 100.0);
    // std::uniform_real_distribution<> distr_axs(-50.0, 50.0);


    for(int i=0; i<size;i++)
    {
        xs_fx[i] = distr_xs(eng);
        // xe_fx[i] = xs_fx[i]+50;
        vxs_fx[i] = distr_vxs(eng);
        vxe_fx[i] = distr_vxs(eng);
        // vxe_fx[i] = vxs_fx[i]+40;
        axs_fx[i] = distr_axs(eng);
        axe_fx[i] = distr_axs(eng);
        t_fx[i] = Timelength;
        xs[i] = static_cast<float>(xs_fx[i]);
        vxs[i] = static_cast<float>(vxs_fx[i]);
        vxe[i] = static_cast<float>(vxe_fx[i]);
        axs[i] = static_cast<float>(axs_fx[i]);
        axe[i] = static_cast<float>(axe_fx[i]);
        t[i] =static_cast<float>(t_fx[i]);
        QuarticPolynomial a = QuarticPolynomial(xs[i], vxs[i], axs[i], vxe[i], axe[i], t[i]);
        
        QuarticPolynomial_fx b = QuarticPolynomial_fx(xs_fx[i], vxs_fx[i], axs_fx[i], vxe_fx[i], axe_fx[i], t_fx[i]);
        float dert_t = t[i]/100;
        for(int j=0;j<100;j++)
        {
            float time = dert_t*j;
            // float d_s = std::abs(a.calc_point(time)-static_cast<float>(b.calc_point_fx(time)))/std::abs(a.calc_point(time));
            // float d_v = std::abs(a.calc_first_derivative(time)-static_cast<float>(b.calc_first_derivative_fx(time)))/std::abs(a.calc_first_derivative(time));
            // float d_a = std::abs(a.calc_second_derivative(time)-static_cast<float>(b.calc_second_derivative_fx(time)))/std::abs(a.calc_second_derivative(time));
            // float d_jerk = std::abs(a.calc_third_derivative(time)-static_cast<float>(b.calc_third_derivative_fx(time)))/std::abs(a.calc_third_derivative(time));
            #ifdef USE_RECORDER
                // Recorder::getInstance()->saveData<float>("d_0[%]", d_s*100);
                // Recorder::getInstance()->saveData<float>("d_1[%]", d_v*100);
                // Recorder::getInstance()->saveData<float>("d_2[%]", d_a*100);
                // Recorder::getInstance()->saveData<float>("d_3[%]", d_jerk*100);
                Recorder::getInstance()->saveData<float>("w_0", a.calc_point(time));
                Recorder::getInstance()->saveData<float>("w_1", a.calc_first_derivative(time));
                Recorder::getInstance()->saveData<float>("w_2", a.calc_second_derivative(time));
                Recorder::getInstance()->saveData<float>("w_3", a.calc_third_derivative(time));
                Recorder::getInstance()->saveData<float>("w_0_0", static_cast<float>(b.calc_point(time)));
                Recorder::getInstance()->saveData<float>("w_1_1", static_cast<float>(b.calc_first_derivative(time)));
                Recorder::getInstance()->saveData<float>("w_2_2", static_cast<float>(b.calc_second_derivative(time)));
                Recorder::getInstance()->saveData<float>("w_3_3", static_cast<float>(b.calc_third_derivative(time)));
                Recorder::getInstance()->saveData<float>("t", time);
                Recorder::getInstance()->saveData<float>("zelt_0", static_cast<float>(std::abs(a.calc_point(time)-static_cast<float>(b.calc_point(time)))));
                Recorder::getInstance()->saveData<float>("zelt_1", static_cast<float>(std::abs(a.calc_first_derivative(time)-static_cast<float>(b.calc_first_derivative(time)))));
                Recorder::getInstance()->saveData<float>("zelt_2", static_cast<float>(std::abs(a.calc_second_derivative(time)-static_cast<float>(b.calc_second_derivative(time)))));
                Recorder::getInstance()->saveData<float>("zelt_3", static_cast<float>(std::abs(a.calc_third_derivative(time)-static_cast<float>(b.calc_third_derivative(time)))));
            #endif
        }
    }
    #ifdef USE_RECORDER
        Recorder::getInstance()->writeDataToCSV();
    #endif
}