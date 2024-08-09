#include "CubicSpline1D_fx.h"
#include "TriDiagonalMatrixSolver_fx.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

// Default constructor
CubicSpline1D_fx::CubicSpline1D_fx() = default;

#include <iostream>
// Construct the 1-dimensional cubic spline.
CubicSpline1D_fx::CubicSpline1D_fx(const vector<fp_type>& v1, //s
                             const vector<fp_type>& v2): //x or y
                             nx(v1.size()), a(v2), x(v1), y(v2),validPath(true){
    // compute elementwise difference
    vector<fp_type> deltas (nx);
    adjacent_difference(x.begin(), x.end(), deltas.begin());
    deltas.erase(deltas.begin());
    
    //compute c with thomas algorithmus
    c.resize(nx);
    std::fill(b.begin(), b.end(), 0);
    std::fill(c.begin(), c.end(), 0);
    std::fill(d.begin(), d.end(), 0);
    tridionalmatrix_a.resize(nx);
    tridionalmatrix_b.resize(nx);
    tridionalmatrix_c.resize(nx);
    tridionalmatrix_d.resize(nx);
    assignValue(tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d, deltas);
    // #ifdef USE_RECORDER
    // for(int i=0;i<nx;i++)
    //     {
    //         Recorder::getInstance()->saveData<float>("deltas_fx", static_cast<float>(deltas[i]));
    //         Recorder::getInstance()->saveData<float>("TM_a_fx",static_cast<float>(tridionalmatrix_a[i]));
    //         Recorder::getInstance()->saveData<float>("TM_b_fx",static_cast<float>(tridionalmatrix_b[i]));
    //         Recorder::getInstance()->saveData<float>("TM_c_fx",static_cast<float>(tridionalmatrix_c[i]));
    //         Recorder::getInstance()->saveData<float>("TM_d_fx",static_cast<float>(tridionalmatrix_d[i]));
    //     }  
    // #endif
    solveTriDiagonalMatrix(tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d, c, nx);

    // construct attribute b, d
    for (int i = 0; i < nx - 1; i++) {
        if(deltas[i]==0)
        {
            std::cout << "CubicSpline1D Line 39 deltas is zero"<< std::endl;
        }
        fp_type dummy = 3*deltas[i];
        d.push_back(cnl::quotient((c[i + 1] - c[i]), dummy));
        dummy = c[i + 1] + 2* c[i];
        dummy = cnl::quotient(dummy,fp_type(3.0));
        dummy = dummy*deltas[i];
        fp_type test = a[i + 1] - a[i];
        test = cnl::quotient(test, deltas[i]);
        b.push_back(test - dummy);
    }
    // #ifdef USE_RECORDER
    // for(int i=0;i<nx;i++)
    //     {   Recorder::getInstance()->saveData<float>("i_fx", static_cast<float>(i));
    //         Recorder::getInstance()->saveData<float>("deltas_fx", static_cast<float>(deltas[i]));
    //         Recorder::getInstance()->saveData<float>("a_fx", static_cast<float>(a[i]));
    //         Recorder::getInstance()->saveData<float>("b_fx", static_cast<float>(b[i]));
    //         Recorder::getInstance()->saveData<float>("d_fx", static_cast<float>(d[i]));
    //         Recorder::getInstance()->saveData<float>("c_fx", static_cast<float>(c[i]));
    //     }  
    // #endif
}

// Calculate the 0th derivative evaluated at t
fp_type CubicSpline1D_fx::calc_der0(fp_time t) {
    if (t < x.front() || t >= (x.back())) {
        validPath = false;
        return std::numeric_limits<fp_type>::max();//FIXME: there could make bug
    }

    int i = search_index(t) - 1;
    fp_time dx = t - x[i];
    fp_type ans = c[i] + d[i]*dx;
    ans*=dx;
    ans+=b[i];
    ans*=dx;
    ans+=a[i];
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("time_fx", static_cast<float>(t));
    //     Recorder::getInstance()->saveData<float>("a_fx", static_cast<float>(a[i]));
    //     Recorder::getInstance()->saveData<float>("b_fx", static_cast<float>(b[i]));
    //     Recorder::getInstance()->saveData<float>("c_fx", static_cast<float>(c[i]));
    //     Recorder::getInstance()->saveData<float>("d_fx", static_cast<float>(d[i]));
    // #endif
    return ans;
}

// Calculate the 1st derivative evaluated at t
fp_type CubicSpline1D_fx::calc_der1(fp_time t) {
    if (t < x.front() || t >= (x.back())) {
        validPath = false;
        return std::numeric_limits<fp_type>::max();//FIXME: there could make bug
    }

    int i = search_index(t) - 1;
    fp_time dx = t - x[i];
    fp_type ans = 2*c[i]+3*d[i]*dx;
    ans*=dx;
    ans+=b[i];
    // return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow_2<fp_type>(dx);
    return ans;
}

#include <iostream>
void CubicSpline1D_fx::assignValue(std::vector<fp_type> &TM_a, std::vector<fp_type> &TM_b, 
                                std::vector<fp_type> &TM_c, std::vector<fp_type> &TM_d, 
                                std::vector<fp_type> &deltas) {
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
        if(deltas[i]==0 || deltas[i + 1]==0)
        {
            std::cout << "CubicSpline1D Line 102 Divisior is zero"<< std::endl;
        }
        fp_type dummy = 0.0;
        // TM_d[i+1] = cnl::quotient(fp_type(3.0) * (a[i + 2] - a[i + 1]), deltas[i + 1]) - 
        //                 cnl::quotient(fp_type(3.0)*(a[i + 1] - a[i]), deltas[i]);
        dummy = 3.0 * (a[i + 2] - a[i + 1]);           
        dummy = cnl::quotient(dummy, deltas[i + 1]);
        TM_d[i+1] = 3.0 *(a[i + 1] - a[i]);
        TM_d[i+1] = cnl::quotient(TM_d[i+1], deltas[i]);
        TM_d[i+1] =dummy-TM_d[i+1]; 
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<float>("deltas_fx", static_cast<float>(deltas[i]));
        //     Recorder::getInstance()->saveData<float>("TM_a_fx",static_cast<float>(TM_a[i+i]));
        //     Recorder::getInstance()->saveData<float>("TM_b_fx",static_cast<float>(TM_b[i+1]));
        //     Recorder::getInstance()->saveData<float>("TM_c_fx",static_cast<float>(TM_c[i+1]));
        //     Recorder::getInstance()->saveData<float>("TM_d_fx",static_cast<float>(TM_d[i+1]));
        // #endif
    }
}

// Search the spline for index closest to t
int CubicSpline1D_fx::search_index(fp_type t) {
    return std::upper_bound (x.begin(), x.end(), t) - x.begin();
}

bool CubicSpline1D_fx::isValidPath()
{
    return validPath;
}