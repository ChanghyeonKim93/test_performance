#ifndef _GEOMETRY_LIBRARY_H_
#define _GEOMETRY_LIBRARY_H_

#include <iostream>
#include "eigen3/Eigen/Dense"

using NumericType = float;
using Rot3 = Eigen::Matrix<NumericType,3,3>;
using Vec3 = Eigen::Matrix<NumericType,3,1>;
using Vec4 = Eigen::Matrix<NumericType,4,1>;

namespace geometry
{
    Vec4 rot2quat(const Rot3& R);
    Rot3 quat2rot(const Vec4& q);
    Vec4 quat1_mul_quat2(const Vec4& q1, const Vec4& q2);

    Rot3 euler2rot(NumericType roll, NumericType pitch, NumericType yaw);
    Rot3 euler2rot(const Vec3& rpy);

    Rot3 axisangle2rot(const Vec3& axis, NumericType angle);
    Vec4 axisangle2quat(const Vec3& axis, NumericType angle);

};

#endif