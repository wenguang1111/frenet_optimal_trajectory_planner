#include <vector>
#include "tool/fp_datatype.h"

void solveTriDiagonalMatrix(const std::vector<fixp_TM_a>& a, const std::vector<fixp_TM_b>& b,
                                     const std::vector<fixp_TM_c>& c, const std::vector<fixp_TM_d>& d,
                                     std::vector<fixp_cubicspline_c>& result, short n);