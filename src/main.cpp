#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"

#include <iostream>

using namespace std;

int main() {
    float wx [25] = {0,50,120};
    float wy [25] = {0,0,0};
    float o_llx[1] = {92.89};
    float o_lly[1] = {191.75};
    float o_urx[1] = {92.89};
    float o_ury[1] = {191.75};

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