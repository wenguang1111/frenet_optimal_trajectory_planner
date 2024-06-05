//Implementation based on thomas algorithmn http://www.industrial-maths.com/ms6021_thomas.pdf
#include "TriDiagonalMatrixSolver.h"
#include "tool/fp_datatype.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <vector>

void solveTriDiagonalMatrix(const std::vector<fixp_TM_a>& a, const std::vector<fixp_TM_b>& b,
                                     const std::vector<fixp_TM_c>& c, const std::vector<fixp_TM_d>& d,
                                     std::vector<fixp_cubicspline_c>& result, short n) {
    std::vector<fixp_TM_c> c_star(n, 0);
    std::vector<fixp_TM_d> d_star(n, 0);

    // Forward sweep
    c_star[0] = c[0] / b[0];
    d_star[0] = d[0] / b[0];
    
    for (short i = 1; i < n; ++i) {
        fixp_cubicspline_m m = 1.0 / (b[i] - a[i] * c_star[i - 1]);
        c_star[i] = c[i] * m;
        d_star[i] = (d[i] - a[i] * d_star[i - 1]) * m;
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("solveTriDiagonalMatrix::c_star", c_star[i]);
        //     Recorder::getInstance()->saveData<double>("solveTriDiagonalMatrix::d_star", d_star[i]);
        //     Recorder::getInstance()->saveData<double>("solveTriDiagonalMatrix::m", m);
        // #endif
    }

    // Backward sweep
    result.back() = d_star.back();
    for (short i = n - 1; i-- > 0; ) {
        result[i] = d_star[i] - c_star[i] * result[i + 1];
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("solveTriDiagonalMatrix::result", result[i]);
        // #endif
    }
}
