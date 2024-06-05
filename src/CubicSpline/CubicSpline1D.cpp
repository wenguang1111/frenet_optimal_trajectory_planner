#include "CubicSpline1D.h"
#include "TriDiagonalMatrixSolver.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

// Default constructor
CubicSpline1D::CubicSpline1D() = default;

// Construct the 1-dimensional cubic spline.
CubicSpline1D::CubicSpline1D(const vector<fixp_s>& v1, //s
                             const vector<fixp_x>& v2): //x or y
                             nx(v1.size()), a(v2), x(v1), y(v2){
    // compute elementwise difference
    vector<fixp_s> deltas (nx);
    adjacent_difference(x.begin(), x.end(), deltas.begin());
    deltas.erase(deltas.begin());
    
    //compute c with thomas algorithmus
    c.resize(nx);
    std::fill(c.begin(), c.end(), 0);
    tridionalmatrix_a.resize(nx);
    tridionalmatrix_b.resize(nx);
    tridionalmatrix_c.resize(nx);
    tridionalmatrix_d.resize(nx);
    assignValue(tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d, deltas);
    solveTriDiagonalMatrix(tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d, c, nx);

    // construct attribute b, d
    for (int i = 0; i < nx - 1; i++) {
        d.push_back((c[i + 1] - c[i]) / (3.0 * deltas[i]));
        b.push_back((a[i + 1] - a[i]) / deltas[i] - deltas[i] *
        (c[i + 1] + 2.0 * c[i]) / 3.0);
    }
}

// Calculate the 0th derivative evaluated at t
fixp_x CubicSpline1D::calc_der0(fixp_s t) {
    if (t < x.front() || t >= x.back()) {
        return NAN;
    }

    int i = search_index(t) - 1;
    fixp_s dx = t - x[i];
    return a[i] + b[i] * dx + c[i] * pow_2<fixp_s>(dx) + d[i] * pow_3<fixp_s>(dx);
}

// Calculate the 1st derivative evaluated at t
fixp_dx CubicSpline1D::calc_der1(fixp_s t) {
    if (t < x.front() || t >= x.back()) {
        return NAN;
    }

    int i = search_index(t) - 1;
    fixp_s dx = t - x[i];

    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow_2<fixp_s>(dx);
}

// // Calculate the 2nd derivative evaluated at
// float CubicSpline1D::calc_der2(float t) {
//     if (t < x.front() || t >= x.back()) {
//         return NAN;
//     }

//     int i = search_index(t) - 1;
//     float dx = t - x[i];

//     return 2.0 * c[i] + 6.0 * d[i] * dx;
// }

void CubicSpline1D::assignValue(std::vector<fixp_TM_a> &TM_a, std::vector<fixp_TM_b> &TM_b, 
                                std::vector<fixp_TM_c> &TM_c, std::vector<fixp_TM_d> &TM_d, 
                                std::vector<fixp_s> &deltas) {
    //initalize the vector for first and last elements
    TM_a[0] = 0;
    TM_a[nx-1] = 0;
    TM_b[0] = 1;
    TM_b[nx-1] = 1;
    TM_c[0] = 0;
    TM_c[nx-1] = 0;
    TM_d[0]=0;
    TM_d[nx-1]=0;
    for (int i = 0; i < nx - 2; i++) {
        TM_a[i+1] = deltas[i];
        TM_b[i+1] = 2*(deltas[i] + deltas[i+1]);
        TM_c[i+1] = deltas[i+1];
        TM_d[i+1] = 3.0 * (a[i + 2] - a[i + 1]) / deltas[i + 1] - 3.0 * 
            (a[i + 1] - a[i]) / deltas[i];
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<double>("assignValue::TM_a", TM_a[i+i]);
            Recorder::getInstance()->saveData<double>("assignValue::TM_b", TM_b[i+1]);
            Recorder::getInstance()->saveData<double>("assignValue::TM_c", TM_c[i+1]);
            Recorder::getInstance()->saveData<double>("assignValue::TM_d", TM_d[i+1]);
        #endif
    }
}

// Search the spline for index closest to t
int CubicSpline1D::search_index(fixp_s t) {
    return std::upper_bound (x.begin(), x.end(), t) - x.begin();
}