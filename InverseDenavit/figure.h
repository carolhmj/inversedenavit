#ifndef FIGURE_H
#define FIGURE_H

#include "joint.h"
/*
 * Classe que representa uma figura articulada. Ela guarda a junta base da articulação e tem a responsabilidade de calcular
 * o state vector do end effector
 */
class Figure
{
private:
    Joint* base;
public:
    Figure(Joint* base);

};

#endif // FIGURE_H
