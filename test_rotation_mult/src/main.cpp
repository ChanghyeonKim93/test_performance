#include <iostream>
#include <eigen3/Eigen/Dense>

#include "geometry_library.h"

// using NumericType = float;
// using Rot3 = Eigen::Matrix<NumericType,3,3>;

int main()
{
    int num_accum = 10000000;
    double angle_step = 0.03;

    Rot3 R1(Rot3::Identity());
    Rot3 dR = geometry::euler2rot(angle_step, 0.0, 0.0);
    
    Rot3 R_true(Rot3::Identity());
    R_true = geometry::euler2rot(angle_step*(double)num_accum,0,0);

    // Result 1) rotation multiplication
    for(int iter = 0; iter < num_accum  ; ++iter)
    {
        R1 = R1*dR;
    }
    std::cout << "Det 1 : " << R1.determinant();
    Rot3 delta_R1 = R_true.transpose()*R1;
    std::cout << ", Error : " << abs(acos(delta_R1(0,0))) << std::endl;

    // Result 2) quaternion multiplication
    Rot3 R2(Rot3::Identity());
    Vec4 q2 = geometry::rot2quat(R2);
    Vec4 dq = geometry::rot2quat(dR);
    for(int iter = 0; iter < num_accum; ++iter)
        q2 = geometry::quat1_mul_quat2(q2,dq);

    R2 = geometry::quat2rot(q2);

    std::cout << "Det 2 : " << R2.determinant();
    Rot3 delta_R2 = R_true.transpose()*R2;
    std::cout << ", Error : " << abs(acos(delta_R2(0,0))) << std::endl;

    // Result 3) rotation multiplication and quaternion conversion
    Rot3 R3(Rot3::Identity());
    for(int iter = 0; iter < num_accum  ; ++iter)
        R3 = R3*dR;

    R3 = geometry::quat2rot(geometry::rot2quat(R3)); // conversion
    std::cout << "Det 3 : " << R3.determinant();
    Rot3 delta_R3 = R_true.transpose()*R3;
    std::cout << ", Error : " << abs(acos(delta_R3(0,0))) << std::endl;

    return 0;
};