#include "../include/geometry_library.h"

namespace geometry
{
    Vec4 rot2quat(const Rot3& R)
    {
        // Ref: Accurate Computation of Quaternions from Rotation Matrices
        // double eta = 0.0; // threshold

        // double r11 = R(0,0), r12 = R(0,1), r13 = R(0,2);
        // double r21 = R(1,0), r22 = R(1,1), r23 = R(1,2);
        // double r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);

        // double val1 =  r11 + r22 + r33;
        // double val2 =  r11 - r22 - r33;
        // double val3 = -r11 + r22 - r33;
        // double val4 = -r11 - r22 + r33;

        // Vec4 q;
        // // calculate q1
        // if(val1 > eta) q(0) = 0.5*sqrt(1+val1);
        // else q(0) = 0.5*sqrt((( r32-r23)*(r32-r23) + (r13-r31)*(r13-r31) + (r21-r12)*(r21-r12) )/(3.0-val1));

        // // calculate q2
        // if(val2 > eta) q(1) = 0.5*sqrt(1+val2);
        // else q(1) = 0.5*sqrt(( (r32-r23)*(r32-r23) + (r12+r21)*(r12+r21) + (r31+r13)*(r31+r13) )/(3.0-val2));

        // // calculate q3
        // if(val3 > eta) q(2) = 0.5*sqrt(1+val3);
        // else q(2) = 0.5*sqrt(( (r13-r31)*(r13-r31) + (r12+r21)*(r12+r21) + (r23+r32)*(r23+r32) )/(3.0-val3) );
        
        // // calculate q4
        // if(val4 > eta) q(3) = 0.5*sqrt(1+val4);
        // else q(3) = 0.5*sqrt(( (r21-r12)*(r21-r12) + (r31+r13)*(r31+r13) + (r32+r23)*(r32+r23) )/(3.0-val4));
       
        // q(0) = abs(q(0));
        // q(1) *= ((r32-r23 > 0) - (r32-r23 < 0));
        // q(2) *= ((r13-r31 > 0) - (r13-r31 < 0));
        // q(3) *= ((r21-r12 > 0) - (r21-r12 < 0));

        double m00 = R(0,0), m01 = R(0,1), m02 = R(0,2);
        double m10 = R(1,0), m11 = R(1,1), m12 = R(1,2);
        double m20 = R(2,0), m21 = R(2,1), m22 = R(2,2);

        double tr = m00 + m11 + m22;

        double qw,qx,qy,qz;
        
        if (tr > 0) { 
            double S = sqrt(tr+1.0) * 2.0; // S=4*qw 
            qw = 0.25 * S;
            qx = (m21 - m12) / S;
            qy = (m02 - m20) / S; 
            qz = (m10 - m01) / S; 
        }
        else if ((m00 > m11)&(m00 > m22)) { 
            double S = sqrt(1.0 + m00 - m11 - m22) * 2.0; // S=4*qx 
            qw = (m21 - m12) / S;
            qx = 0.25 * S;
            qy = (m01 + m10) / S; 
            qz = (m02 + m20) / S; 
        } 
        else if (m11 > m22) { 
            double S = sqrt(1.0 + m11 - m00 - m22) * 2.0; // S=4*qy
            qw = (m02 - m20) / S;
            qx = (m01 + m10) / S; 
            qy = 0.25 * S;
            qz = (m12 + m21) / S; 
        } 
        else { 
            double S = sqrt(1.0 + m22 - m00 - m11) * 2.0; // S=4*qz
            qw = (m10 - m01) / S;
            qx = (m02 + m20) / S;
            qy = (m12 + m21) / S;
            qz = 0.25 * S;
        }

        Vec4 q;
        q(0) = qw;
        q(1) = qx;
        q(2) = qy;
        q(3) = qz;

        q.normalize();

        return q;  
    };
    
    Rot3 quat2rot(const Vec4& q)
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
        R << 1.0-cc-dd, bc-ad, bd+ac, 
             bc+ad, 1.0-bb-dd, cd-ab,
             bd-ac, cd+ab, 1.0-bb-cc;
        
        return R;
    };

    Vec4 quat1_mul_quat2(const Vec4& q1, const Vec4& q2)
    {
        Vec4 q_res;
        
        double a1 = q1(0), b1 = q1(1), c1 = q1(2), d1 = q1(3);
        double a2 = q2(0), b2 = q2(1), c2 = q2(2), d2 = q2(3);

        q_res(0) = a1*a2-b1*b2-c1*c2-d1*d2;
        q_res(1) = a1*b2+b1*a2+c1*d2-d1*c2;
        q_res(2) = a1*c2-b1*d2+c1*a2+d1*b2;
        q_res(3) = a1*d2+b1*c2-c1*b2+d1*a2;

        q_res.normalize();

        return q_res;
    };

    Rot3 euler2rot(NumericType roll, NumericType pitch, NumericType yaw)
    {
        Rot3 R;
        double cr = cos((double)roll),  sr = sin((double)roll);
        double cp = cos((double)pitch), sp = sin((double)pitch);
        double cy = cos((double)yaw),   sy = sin((double)yaw);

        Rot3 Rr, Rp, Ry;
        Rr << cr,sr,0,-sr,cr,0,0,0,1.0;
        Rp << cp, 0, -sp, 0, 1.0, 0, sp, 0, cp;
        Ry << 1.0,0,0,0,cy,sy,0,-sy,cy;

        R = Ry*Rp*Rr;

        return R;
    };
};