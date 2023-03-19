#ifndef _GEOMETRY_LIBRARY_H_
#define _GEOMETRY_LIBRARY_H_

#include <iostream>
#include "eigen3/Eigen/Dense"

using NumericType = double;
using Rot3 = Eigen::Matrix<NumericType,3,3>;
using Vec3 = Eigen::Matrix<NumericType,3,1>;
using Vec4 = Eigen::Matrix<NumericType,4,1>;

namespace geometry
{
    Vec4 r2q(const Rot3& R);
    Rot3 q2r(const Vec4& q);
    Vec4 q1_mul_q2(const Vec4& q1, const Vec4& q2);

    Rot3 a2r(NumericType roll, NumericType pitch, NumericType yaw);
    Rot3 a2r(const Vec3& rpy);

};

#endif