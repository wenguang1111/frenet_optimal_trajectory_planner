#include "CubicSpline1D_fx.h"
#include <vector>

void solveTriDiagonalMatrix(const std::vector<fp_type>& a, const std::vector<fp_type>& b,
                                     const std::vector<fp_type>& c, const std::vector<fp_type>& d,
                                     std::vector<fp_type>& result, short n);