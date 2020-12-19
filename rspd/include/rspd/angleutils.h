#ifndef ANGLEUTILS_H
#define ANGLEUTILS_H

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include <Eigen/Dense>

class AngleUtils
{
public:
    inline static float deg2rad(float deg)
    {
        return static_cast<float>(deg * M_PI / 180);
    }

    inline static float rad2deg(float rad)
    {
        return static_cast<float>(rad * 180 / M_PI);
    }

    static Eigen::Vector3f rotate(const Eigen::Vector3f &v, float degrees, const Eigen::Vector3f &axis)
    {
        Eigen::Affine3f t;
        t = Eigen::AngleAxisf(AngleUtils::deg2rad(degrees), axis);
        return t * v;
    }

};

#endif // ANGLEUTILS_H
