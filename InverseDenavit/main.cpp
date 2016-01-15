#include "mainwindow.h"
#include <QApplication>

#include <iostream>
#include "calculations.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
//    Vector3d u_i(0,0,1);
//    Vector3d u_i_plus_1(0,0,1);

//    Vector3d p_i(2*M_SQRT1_2, 2*M_SQRT1_2, 0);
//    Vector3d p_i_plus_1(3*M_SQRT1_2, 3*M_SQRT1_2, 0);

//    Vector3d a_i_minus_1(M_SQRT1_2, M_SQRT1_2, 0);
//    Vector3d a_i = linkVectorAxisParallel(p_i, p_i_plus_1, u_i);
//    cout << "link vector: \n" << a_i << endl;
//    cout << "y axis: \n" << crossNormalize(u_i, a_i) << endl;
//    cout << "link twist: " << linkTwist(u_i, u_i_plus_1,a_i) << endl;
//    cout << "joint angle: " << jointAngle(a_i_minus_1, a_i, u_i) << endl;

}
