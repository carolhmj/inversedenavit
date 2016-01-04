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
};

#endif // FIGURE_H
