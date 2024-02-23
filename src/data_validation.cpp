// source: https://stackoverflow.com/questions/4915462/how-should-i-do-floating-point-comparison
#include <rosparam_utils/data_validation.hpp>

namespace data_validation
{
    bool float_compare(float a, float b, float epsilon, float abs_th)
    {
        if (a == b) return true;

        float diff = std::abs(a-b);
        float norm = std::min((std::abs(a) + std::abs(b)), std::numeric_limits<float>::max());
        return diff < std::max(abs_th, epsilon * norm);
    }
}