//Implementation based on thomas algorithmn http://www.industrial-maths.com/ms6021_thomas.pdf
#include "TriDiagonalMatrixSolver_fx.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif
#include <vector>
#include <iostream>

void solveTriDiagonalMatrix(const std::vector<fp_type>& a, const std::vector<fp_type>& b,
                                     const std::vector<fp_type>& c, const std::vector<fp_type>& d,
                                     std::vector<fp_type>& result, short n) {
    std::vector<fp_type> c_star(n, 0);
    std::vector<fp_type> d_star(n, 0);

    // Forward sweep
    if(b[0]==0)
    {
        std::cout << "TriDiagonalMatrixSolver Line 18 Divisior is zero"<< std::endl;
    }
    c_star[0] = cnl::quotient(c[0], b[0]);
    d_star[0] = cnl::quotient(d[0], b[0]);
    
    for (short i = 1; i < n; ++i) {
        fp_type m = cnl::quotient(fp_type(1.0), (b[i] - a[i] * c_star[i - 1]));
        c_star[i] = c[i] * m;
        d_star[i] = d[i] - a[i] * d_star[i - 1];
        d_star[i] *=  m;
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
