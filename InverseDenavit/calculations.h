#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#define _USE_MATH_DEFINES ;
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;
/*
 * Só funções pra calcular coisas importantes
 */

Vector3d linkVectorAxisIntersect(Vector3d u_i, Vector3d u_i_plus_1);
Vector3d linkVectorAxisParallel(Vector3d p_i, Vector3d p_i_plus_1, Vector3d u_i);
Vector3d crossNormalize(Vector3d a, Vector3d b);
double linkTwist(Vector3d u_i, Vector3d u_i_plus_1, Vector3d a_i);
double jointAngle(Vector3d a_i_minus_1, Vector3d a_i, Vector3d z_i);

#endif // CALCULATIONS_H

