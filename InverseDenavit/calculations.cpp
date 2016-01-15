#include "calculations.h"


Vector3d linkVectorAxisIntersect(Vector3d u_i, Vector3d u_i_plus_1)
{
    return crossNormalize(u_i, u_i_plus_1);
}


Vector3d linkVectorAxisParallel(Vector3d p_i, Vector3d p_i_plus_1, Vector3d u_i)
{
    Vector3d q = p_i_plus_1 - p_i;
    return q - (q.dot(u_i.normalized()))*u_i.normalized();
}


Vector3d crossNormalize(Vector3d a, Vector3d b)
{
    Vector3d cross = a.cross(b);
    return cross.normalized();
}



double linkTwist(Vector3d u_i, Vector3d u_i_plus_1, Vector3d a_i)
{
    Vector3d cross = u_i.cross(u_i_plus_1);
    double val = cross.dot(a_i);
    double sign;
    if (val >= 0) {
        sign = 1;
    } else {
        sign = -1;
    }
    double normmultiplied = u_i.norm() * u_i_plus_1.norm();
    double a = cross.norm()/normmultiplied;
    double b = u_i.dot(u_i_plus_1)/normmultiplied;
    return sign*atan2(a,b);
}


double jointAngle(Vector3d a_i_minus_1, Vector3d a_i, Vector3d z_i)
{
    Vector3d cross = a_i_minus_1.cross(a_i);
    double val = cross.dot(z_i);
    double sign;
    if (val >= 0) {
        sign = 1;
    } else {
        sign = -1;
    }
    double normmultiplied = a_i_minus_1.norm() * a_i.norm();
    double a = cross.norm()/normmultiplied;
    double b = a_i_minus_1.dot(a_i)/normmultiplied;
    return sign*atan2(a,b);
}
