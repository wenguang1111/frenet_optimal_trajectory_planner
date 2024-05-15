//Implementation based on thomas algorithmn http://www.industrial-maths.com/ms6021_thomas.pdf
#include "TriDiagonalMatrixSolver.h"
#include <vector>

void solveTriDiagonalMatrix(const std::vector<double>& a, const std::vector<double>& b,
                                     const std::vector<double>& c, const std::vector<double>& d,
                                     std::vector<double>& result, short n) {
    std::vector<double> c_star(n, 0);
    std::vector<double> d_star(n, 0);

    // Forward sweep
    c_star[0] = c[0] / b[0];
    d_star[0] = d[0] / b[0];
    
    for (short i = 1; i < n; ++i) {
        double m = 1.0 / (b[i] - a[i] * c_star[i - 1]);
        c_star[i] = c[i] * m;
        d_star[i] = (d[i] - a[i] * d_star[i - 1]) * m;
    }

    // Backward sweep
    result.back() = d_star.back();
    for (short i = n - 1; i-- > 0; ) {
        result[i] = d_star[i] - c_star[i] * result[i + 1];
    }
}
