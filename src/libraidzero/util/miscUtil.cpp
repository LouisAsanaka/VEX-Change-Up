#include "libraidzero/util/miscUtil.hpp"
#include <cmath>

double constrainAngle(double angle) {
    return angle - floor(angle / (2 * M_PI) + 0.5) * (2 * M_PI);
}