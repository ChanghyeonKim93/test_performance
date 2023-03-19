#include "../include/geometry_library.h"

namespace geometry
{
    Vec4 r2q(const Rot3& R)
    {
        // Ref: Accurate Computation of Quaternions from Rotation Matrices
        double eta = 0.0; // threshold

        double r11 = R(0,0), r12 = R(0,1), r13 = R(0,3);
        double r21 = R(1,0), r22 = R(1,1), r23 = R(1,2);
        double r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);

        double val1 =  r11 + r22 + r33;
        double val2 =  r11 - r22 - r33;
        double val3 = -r11 + r22 - r33;
        double val4 = -r11 - r22 + r33;

        Vec4 q;
        // calculate q1
        if(val1 > eta) q(0) = 0.5*sqrt(1+val1);
        else q(0) = 0.5*sqrt((( r32-r23)*(r32-r23) + (r13-r31)*(r13-r31) + (r21-r12)*(r21-r12) )/(3.0-val1));

        // calculate q2
        if(val2 > eta) q(1) = 0.5*sqrt(1+val2);
        else q(1) = 0.5*sqrt(( (r32-r23)*(r32-r23) + (r12+r21)*(r12+r21) + (r31+r13)*(r31+r13) )/(3.0-val2));

        // calculate q3
        if(val3 > eta) q(2) = 0.5*sqrt(1+val3);
        else q(2) = 0.5*sqrt(( (r13-r31)*(r13-r31) + (r12+r21)*(r12+r21) + (r23+r32)*(r23+r32) )/(3.0-val3) );
        
        // calculate q4
        if(val4 > eta) q(3) = 0.5*sqrt(1+val4);
        else q(3) = 0.5*sqrt(( (r21-r12)*(r21-r12) + (r31+r13)*(r31+r13) + (r32+r23)*(r32+r23) )/(3.0-val4));
       
        return q;  
    };
    
    Rot3 q2r(const Vec4& q)
    {
        // double inv_norm = 1.0/q.norm();

        // double qw = q(0)*inv_norm;
        // double qx = q(1)*inv_norm;
        // double qy = q(2)*inv_norm;
        // double qz = q(3)*inv_norm;

        // double qw2 = qw*qw;
        // double qx2 = qx*qx;
        // double qy2 = qy*qy;
        // double qz2 = qz*qz;

        // double qxqy = qx*qy;
        // double qwqz = qw*qz;
        // double qxqz = qx*qz;
        // double qwqy = qw*qy;
        // double qwqx = qw*qx;
        // double qyqz = qy*qz;

        // Rot3 R;
        // R <<  qw2+qx2-qy2-qz2, 2.0*(qxqy-qwqz), 2.0*(qxqz+qwqy),
        //       2.0*(qxqy+qwqz), qw2-qx2+qy2-qz2, 2.0*(qyqz-qwqx),
        //       2.0*(qxqz-qwqy), 2.0*(qyqz+qwqx), qw2-qx2-qy2+qz2;    
        
        // The below is the more efficient method to calculate the rotation matrix
        // with unnormalized quaternion.
        double a = q(0);
        double b = q(1);
        double c = q(2);
        double d = q(3);

        double s = 2.0/(a*a + b*b + c*c + d*d);

        double bs = b*s;
        double cs = c*s;
        double ds = d*s;

        double ab = a*bs, ac = a*cs, ad = a*ds;
        double bb = b*bs, bc = b*cs, bd = b*ds;
        double cc = c*cs, cd = c*ds, dd = d*ds;
        
        Rot3 R;
        R << 1-cc-dd, bc-ad, bd+ac, 
             bc+ad, 1-bb-dd, cd-ab,
             bd-ac, cd+ab, 1-bb-cc;
        
        return R;
    };

    Vec4 q1_mul_q2(const Vec4& q1, const Vec4& q2)
    {
        Vec4 q_res;
        return q_res;
    };

    Rot3 a2r(NumericType roll, NumericType pitch, NumericType yaw)
    {
        Rot3 R;
        NumericType cr = cos(roll), sr = sin(roll);
        NumericType cp = cos(pitch), sp = sin(pitch);
        NumericType cy = cos(yaw), sy = sin(yaw);

        Rot3 Rr, Rp, Ry;
        Rr << cr,sr,0,-sr,cr,0,0,0,1;
        Rp << cp, 0, -sp, 0, 1, 0, sp, 0, cp;
        Ry << 1,0,0,0,cy,sy,0,-sy,cy;

        R = Ry*Rp*Rr;

        return R;
    };
};