#pragma once

#include <limits>
#include <cmath>
#include <algorithm>

namespace data_validation
{
    bool float_compare(float a, float b, float epsilon = 128 * std::numeric_limits<float>::epsilon(), float abs_th = std::numeric_limits<float>::min());
}
