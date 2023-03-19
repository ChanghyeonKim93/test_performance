#include <iostream>
#include <eigen3/Eigen/Dense>

#include "geometry_library.h"

// using NumericType = float;
// using Rot3 = Eigen::Matrix<NumericType,3,3>;

int main()
{
    Rot3 R_true(Rot3::Identity());

    Vec4 q;
    q = geometry::r2q(R_true);
    Rot3 R = geometry::q2r(q);

    std::cout << q << std::endl;

    std::cout << R_true << std::endl;
    std::cout << R << std::endl;


    return 0;
};