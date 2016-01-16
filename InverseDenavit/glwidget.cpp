#include "glwidget.h"
#include <iostream>
#include <QMouseEvent>

using namespace std;
GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{

}

void GLWidget::initializeGL(){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    Vector3d i = Vector3d(1,0,0), j = Vector3d(0,1,0), k = Vector3d(0,0,1), l = Vector3d(0,M_SQRT1_2,M_SQRT1_2), m = Vector3d(M_SQRT1_2,M_SQRT1_2,0), n = Vector3d(-M_SQRT1_2,M_SQRT1_2,0);
    //Ordem dos parâmetros é: x, y, z, length, twist, offset, angle, origin, originNext
    Joint* l0 = new Joint(i, j, k, 0., -M_PI_4, 0., 0., Vector3d(0,0,0), Vector3d(0,0,0));
    Joint* l1 = new Joint(i, Vector3d(0,M_SQRT1_2,-M_SQRT1_2), l, 0., M_PI_4, 0., 0., Vector3d(0,0,0), Vector3d(0,0,0));
    Joint* l2 = new Joint(m, n, k, 1., 0., 0., M_PI_4, Vector3d(0,0,0), m);
    Joint* l3 = new Joint(m, n, k, 1., 0., 0., 0., m, 2*m);
    Joint *l4 = new Joint(m, n, k, 1., 0., 0., 0., 2*m, 3*m);
    Joint *l5 = new Joint(m, n, k, 1., 0., 0., 0., 3*m, 4*m);
    Joint *l6 = new Joint(m, n, k, 1., 0., 0., 0., 4*m, 5*m);
    Joint *l7 = new Joint(m, n, k, 1., 0., 0., 0., 5*m, 6*m);
    l0->setNext(l1);
    l1->setNext(l2);
    l2->setNext(l3);
    l3->setNext(l4);
    l4->setNext(l5);
    l5->setNext(l6);
    l6->setNext(l7);

    Figure* f = new Figure(l0);
//    cout << "matriz transform: \n" << f->getEndToBaseTransform() << "\n\n\n";
//    cout << "calculando: \n" << f->getEndToBaseTransform()*Vector4d(1,0,0,1) << "\n\n\n";
    f->calcStateVector();
//    cout << "state vector: \n" << f->getState() << "\n\n\n";

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
    return M;
}

Matrix4d GLWidget::lookAtMatrix(Vector3d pos, Vector3d target, Vector3d up)
{
    Vector3d z = pos-target;
    z.normalize();
    Vector3d x = up.cross(z);
    x.normalize();
    Vector3d y = z.cross(x);
    y.normalize();

    Matrix4d R;
    //R << x;
}

void GLWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton){
        Vector6d ns;
        double ang = 25;
        AngleAxisd rot(ang * (M_PI / 180.0), Vector3d::UnitZ());
        Vector3d p = rot.toRotationMatrix() * 5*Vector3d::UnitX();
        ns << p, 0, 0, ang * (M_PI / 180.0);
        obj->iterationScheme(ns, 0.0001, 2);
        updateGL();
    }
}
