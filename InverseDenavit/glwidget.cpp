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
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    connect(&timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    timer.start(1);

    trackball(curquat, 0.0, 0.0, 0.0, 0.0);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(ortographicMatrix(-10000, 10000, -10,10, 10, -10).data());

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
//    cout << "state vector: \n" << f->getState() << "\n\n\n";

//    cout << "joint 2\n";
//    cout << "parent transform\n" << f->getBase()->getNext()->getNext()->getParentMatrix() << endl;
//    cout << "child transform\n" << f->getBase()->getNext()->getNext()->getChildMatrix() << endl;
//    cout << "jacobian\n" << f->getJacobian() << endl;
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

    float m[4][4];

    build_rotmatrix(m, curquat);

    Matrix4f viewMF = Map<Matrix4f>(m[0]);
    Matrix4d viewMD = viewMF.cast<double>();
    obj->draw(viewMD);
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

Matrix4d GLWidget::perspectiveMatrix(double fovY, double aspect, double near, double far)
{
  float theta = fovY*0.5;
  float range = far - near;
  float invtan = 1./tan(theta);

  Matrix4d mProjectionMatrix;

  mProjectionMatrix(0,0) = invtan / aspect;
  mProjectionMatrix(1,1) = invtan;
  mProjectionMatrix(2,2) = -(near + far) / range;
  mProjectionMatrix(3,2) = -1;
  mProjectionMatrix(2,3) = -2 * near * far / range;
  mProjectionMatrix(3,3) = 0;

  return mProjectionMatrix.transpose();
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

        double angX = countang++ * 2;
        AngleAxisd rotX(angX * (M_PI / 180.0), Vector3d::UnitX());
        Vector3d p = rotX.toRotationMatrix() * 5*Vector3d::UnitX();

        double angY = 0;//countang++ * 2;
        AngleAxisd rotY(angY * (M_PI / 180.0), Vector3d::UnitY());
        p = rotY.toRotationMatrix() * p;

        double angZ = 45; //- countang++*2;
        AngleAxisd rotZ(angZ * (M_PI / 180.0), Vector3d::UnitZ());
        p = rotZ.toRotationMatrix() * p;

        //ns << p, angX * (M_PI / 180.0), angY * (M_PI / 180.0), angZ * (M_PI / 180.0);
        ns << 0, 0, 0, 0, 0, 0;
        obj->iterationScheme(ns, 0.0001, 2);
        updateGL();
    }

    if (event->button() == Qt::RightButton)
    {
        beginx = event->x();
        beginy = event->y();
    }

    if (event->button() == Qt::MiddleButton) {
        trackball(curquat, 0.0, 0.0, 0.0, 0.0);
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event){
    float width = size().width();
    float height = size().height();

    trackball(lastquat,
      (2.0 * beginx - width) / width,
      (height - 2.0 * beginy) / height,
      (2.0 * event->x() - width) / width,
      (height - 2.0 * event->y()) / height
    );
    beginx = event->x();
    beginy = event->y();
    add_quats(lastquat, curquat, curquat);
    updateGL();
}
