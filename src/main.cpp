#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "tool/fp_datatype.h"

#include <iostream>

using namespace std;

#include "cordic.h"
void test_cordic_cos() {
    float error = 0.01;
    // Test values
    int_3_12 angles[] = {int_3_12(-6), int_3_12(-3), int_3_12(-1.5), int_3_12(-0.8), int_3_12(-0.3), int_3_12(-0.1), int_3_12(-0.01), int_3_12(-0.001),
                          int_3_12(6), int_3_12(3), int_3_12(1.5), int_3_12(0.8), int_3_12(0.3), int_3_12(0.1), int_3_12(0.01), int_3_12(0.001), int_3_12(0.1015625), int_3_12(0.026123046875)};
    double angles_float[] = {-6, -3, -1.5, -0.8, -0.3, -0.1, -0.01, -0.001, 6, 3, 1.5, 0.8, 0.3, 0.1, 0.01, 0.001, 0.1015625, 0.0859375, 0.026123046875};

    for(int i=0; i<18; i++) {
        fixp_yaw cordic_result_cos = cordic_cos(angles[i]);
        fixp_yaw cordic_result_sin = cordic_sin(angles[i]);
        double standard_result_cos = cos(angles_float[i]);
        double standard_result_sin = sin(angles_float[i]);

        std::cout<<"Angle: "<<angles[i]<<" radians"<<std::endl;
        std::cout<<"CORDIC Cos: "<<cordic_result_cos<<std::endl;
        std::cout<<"Standard Cos: "<<standard_result_cos<<std::endl;
        std::cout<<"CORDIC Sin: "<<cordic_result_sin<<std::endl;
        std::cout<<"Standard Sin: "<<standard_result_sin<<std::endl;

        // Allow for small error due to precision of floating point
        if(std::abs(cordic_result_cos - standard_result_cos) < error) {
              std::cout<<"Cos Test passed!"<<std::endl;
        } else {
              std::cout<<"Cos Test failed!"<<std::endl;
        }
        if(std::abs(cordic_result_sin - standard_result_sin) < error) {
              std::cout<<"Sin Test passed!"<<std::endl;
        } else {
              std::cout<<"Sin Test failed!"<<std::endl;
        }

        std::cout<<"--------------------------------"<<std::endl;
        std::cout << int_8_7(16.0625)+cordic_cos(int_3_12(0.026123046875))*int_8_7(4.9296875)*0.5<<std::endl;
    }
}
//yaw=.026123046875; head_x_float = 18.5265; head_x = -13.46875; x_float = 16.0625; x = 16.0625;length = 4.9296875

int main() {
    fixp_x wx [25] = {0,50,120};
    fixp_y wy [25] = {0,0,0};
    fixp_x o_llx[1] = {92.89};
    fixp_y o_lly[1] = {191.75};
    fixp_x o_urx[1] = {92.89};
    fixp_y o_ury[1] = {191.75};

    // set up experiment
    FrenetInitialConditions_FP fot_ic = {
        fixp_s{34.6},
        fixp_s_d{7.10964962},
        -1.35277168,
        -1.86,
        0.0,
        10,
        wx,
        wy,
        25,
        o_llx,
        o_lly,
        o_urx,
        o_ury,
        1
    };
    FrenetHyperparameters_FP fot_hp = {
        fixp_s_d{25.0},
        fixp_s_d{15.0},
        fixp_s_d{15.0},
        fixp_s_d{6.0},
        fixp_s_d{6.0},
        fixp_s_d{0.1},
        fixp_s_d{0.1},
        fixp_s_d{5.0},
        2.0,
        fixp_s_d{0.1},
        fixp_s_d{5.0},
        0.1,
        fixp_s_d{1.0},
        fixp_s_d{0.1},
        fixp_s_d{0.1},
        fixp_s_d{0.1},
        fixp_s_d{0.1},
        fixp_s_d{10},
        fixp_s_d{1.0},
        fixp_s_d{1.0},
        0 // num thread
    };
    test_cordic_cos();
    // run experiment
    FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
    FrenetPath* best_frenet_path = fot.getBestPath();
    if (best_frenet_path) {
        cout << "Success\n";
        return 1;
    }
    cout << "Failure\n";
    return 0;
}