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
    inline static double deg2rad(double deg)
    {
        return static_cast<double>(deg * M_PI / 180);
    }

    static Eigen::Vector3d rotate(const Eigen::Vector3d &v, double degrees, const Eigen::Vector3d &axis)
    {
        Eigen::Affine3d t;
        t = Eigen::AngleAxisd(AngleUtils::deg2rad(degrees), axis);
        return t * v;
    }

};

#endif // ANGLEUTILS_H
