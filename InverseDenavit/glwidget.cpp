#include "glwidget.h"
#include <iostream>

using namespace std;
GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{

}

void GLWidget::initializeGL(){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    Vector3d i = Vector3d(1,0,0), j = Vector3d(0,1,0), k = Vector3d(0,0,1), m = Vector3d(-1,0,0);
    //Ordem dos parâmetros é: x, y, z, length, twist, offset, angle, origin, originNext
    Joint* l0 = new Joint(i, Vector3d(0,0,-1), j, 0.0, 0.0, 0.0, 0.0, Vector3d(0,0,0), Vector3d(0,0,0));
    Joint* l1 = new Joint(i, Vector3d(0,0,-1), j, 0.0, M_PI_2, 0.0, 0.0, Vector3d(0,0,0), Vector3d(0,0,0));
    Joint* l2 = new Joint(j, m, k, 1.0, 0.0, 0.0, M_PI_2, Vector3d(0,0,0), Vector3d(0,1,0));
    Joint* l3 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0, Vector3d(0,1,0), Vector3d(0,2,0));
    Joint *l4 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0, Vector3d(0,2,0), Vector3d(0,3,0));
    Joint *l5 = new Joint(j, m, k, 1.0, 0.0, 0.0, 0.0, Vector3d(0,3,0), Vector3d(0,4,0));
    Joint *l6 = new Joint(j, m, k, 0.0, 0.0, 0.0, 0.0, Vector3d(0,4,0), Vector3d(0,4,0));
    l0->setNext(l1);
    l1->setNext(l2);
    l2->setNext(l3);
    l3->setNext(l4);
    l4->setNext(l5);
    l5->setNext(l6);

    Figure* f = new Figure(l0);
    cout << "matriz transform: \n" << f->getEndToBaseTransform() << "\n";
    f->calcStateVector();
    cout << "state vector: \n" << f->getState() << "\n";

    obj = f;
}

void GLWidget::resizeGL(int w, int h){
    if(h<w) {
        glViewport((w-h)/2,0,h,h);
    }
    else {
        glViewport(0,(h-w)/2,w,w);
    }
}

void GLWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    obj->draw(ortographicMatrix(-6,6,-6,6,6,-6));
}

Matrix4d GLWidget::perspectiveMatrix(double fov, double far, double near)
{
    double S = 1/tan((fov/2)*(M_PI/180));
    Matrix4d M;
    M << S, 0,            0          ,  0,
         0, S,            0          ,  0,
         0, 0,     -far/(far-near)   , -1,
         0, 0, -(far*near)/(far-near),  0;
    return M;
}

Matrix4d GLWidget::ortographicMatrix(double far, double near, double left, double right, double top, double bottom)
{
    Matrix4d M;
    M <<        2/(right-left)     ,            0              ,             0         , 0,
                      0            ,        2/(top-bottom)     ,             0         , 0,
                      0            ,            0              ,       -2/(far-near)   , 0,
         -(right+left)/(right-left), -(top+bottom)/(top-bottom), -(far+near)/(far-near), 1;
    cout << "ortho matrix: \n" << M << "\n\n";
    return M;
}
