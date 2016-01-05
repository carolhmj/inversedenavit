#include "mainwindow.h"
#include <QApplication>

#define _USE_MATH_DEFINES

#include "figure.h"
#include "joint.h"
#include <cmath>
#include <iostream>
int main(int argc, char *argv[])
{
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();

//    return a.exec();

    std::cout << "M_PI_2: " << M_PI_2 << "\n";
    std::cout << "arctan(0,0): " << std::atan2(0,0) << "\n";

    Vector3d i = Vector3d(1,0,0), j = Vector3d(0,1,0), k = Vector3d(0,0,1), m = Vector3d(-1,0,0);
    //Ordem dos parâmetros é: x, y, z, length, twist, offset, angle
    Joint* l0 = new Joint(i, Vector3d(0,0,-1), j, 0.0, 0.0, 0.0, 0.0);
    Joint* l1 = new Joint(i, Vector3d(0,0,-1), j, 0.0, M_PI_2, 0.0, 0.0);
    Joint* l2 = new Joint(j, m, k, 1.0, 0.0, 0.0, M_PI_2);
    Joint* l3 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0);
    Joint *l4 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0);
    Joint *l5 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0);
    Joint *l6 = new Joint(j, m, k, 0.0, 0.0, 0.0, 0.0);
    l0->setNext(l1);
    l1->setNext(l2);
    l2->setNext(l3);
    l3->setNext(l4);
    l4->setNext(l5);
    l5->setNext(l6);

    Figure* f = new Figure(l0);
    std::cout << "matriz transform: \n" << f->getEndToBaseTransform() << "\n";
    f->calcStateVector();
    std::cout << "state vector: \n" << f->getState() << "\n";
}
