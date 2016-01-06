#ifndef FIGURE_H
#define FIGURE_H

#include "joint.h"

typedef Matrix<double, 6, 1> Vector6d;

/*
 * Classe que representa uma figura articulada. Ela guarda a junta base da articulação e tem a responsabilidade de calcular
 * o state vector do end effector
 */
class Figure
{
private:
    Joint* base;
    Vector6d state;
public:
    Figure(Joint* base);
    Matrix4d getEndToBaseTransform();
    void calcStateVector();
    Vector6d getState() const;
    Vector6d getDerivativeStateVector(Joint *j);
    MatrixXd getJacobian();
    MatrixXd getInverse(MatrixXd M);
    void setParams(VectorXd params);
    void iterationScheme(Vector6d target, double tolerance);
    VectorXd computeJointParameters(VectorXd parameters, Vector6d current, Vector6d target);
    void draw(Matrix4d mv);
};

#endif // FIGURE_H
