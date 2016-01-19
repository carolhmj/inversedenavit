#include "glwidget.h"
#include <iostream>
#include <QMouseEvent>
#include <GL/gl.h>

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

    stateDesired << 0.0,0.0,0.0,0.0,0.0,0.0;
    cout << "state desired:\n" << stateDesired << endl;

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(ortographicMatrix(-10000, 10000, -10,10, 10, -10).data());

    Vector3d i = Vector3d(1,0,0), j = Vector3d(0,1,0), k = Vector3d(0,0,1), l = Vector3d(0,M_SQRT1_2,M_SQRT1_2), m = Vector3d(M_SQRT1_2,M_SQRT1_2,0), n = Vector3d(-M_SQRT1_2,M_SQRT1_2,0);
    //Ordem dos parâmetros é: x, y, z, length, twist, offset, angle, origin, originNext, color
    //Branco
    Joint* l0 = new Joint(i, j, k, 0., -M_PI_4, 0., 0., Vector3d(0,0,0), Vector3d(0,0,0), Vector3d(1,1,1));
    //Amarelo
    Joint* l1 = new Joint(i, Vector3d(0,M_SQRT1_2,-M_SQRT1_2), l, 0., M_PI_4, 0., 0., Vector3d(0,0,0), Vector3d(0,0,0), Vector3d(1,1,0));
    //Turquesa
    Joint* l2 = new Joint(m, n, k, 1., 0., 0., M_PI_4, Vector3d(0,0,0), m, Vector3d(0,1,1));
    //Roxo
    Joint* l3 = new Joint(m, n, k, 1., 0., 0., 0., m, 2*m, Vector3d(1,0,1));
    //Verde
    Joint *l4 = new Joint(m, n, k, 1., 0., 0., 0., 2*m, 3*m, Vector3d(0,1,0));
    //Azul
    Joint *l5 = new Joint(m, n, k, 1., 0., 0., 0., 3*m, 4*m, Vector3d(0,0,1));
    //Cinza
    Joint *l6 = new Joint(m, n, k, 1., 0., 0., 0., 4*m, 5*m, Vector3d(0.5,0.5,1));
    //Vermelho
    Joint *l7 = new Joint(m, n, k, 1., 0., 0., 0., 5*m, 6*m, Vector3d(1,0,0));
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
    double colors[3] = {0.5,0.,0.};
//    Vector3d pos(stateDesired(0),stateDesired(1),stateDesired(2));
//    drawCube(pos, 0.4, colors, viewMD);
//    drawTarget(stateDesired, viewMD);
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

void GLWidget::drawCube(Vector3d center, double size, double color[], Matrix4d mv)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(mv.data());

    double s = size/2;
    GLdouble vertices[] = {
        center(0)+s, center(1)+s, center(2)+s,
        center(0)+s, center(1)+s, center(2)-s,
        center(0)-s, center(1)+s, center(2)-s,
        center(0)-s, center(1)+s, center(2)+s,
        center(0)-s, center(1)-s, center(2)+s,
        center(0)+s, center(1)-s, center(2)+s,
        center(0)+s, center(1)-s, center(2)-s,
        center(0)-s, center(1)-s, center(2)-s
    };

    GLdouble colors[] = {
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
       color[0], color[1], color[2],
    };

    unsigned int indices[] = {
        1,2,3,4,
        5,6,7,8,
        1,6,7,2,
        2,7,8,3,
        3,8,5,4,
        4,5,6,1
    };

    glPolygonMode(GL_FRONT, GL_LINE);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColorPointer(3, GL_DOUBLE, 0, colors);
    glVertexPointer(3, GL_DOUBLE, 0, vertices);
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_INT, indices);
}

void GLWidget::drawTarget(Vector6d state, Matrix4d mv)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(mv.data());

    glPointSize(5);
    glBegin(GL_POINT);
        glColor3d(1,1,1);
        glVertex3d(state(0), state(1), state(2));
    glEnd();

    Matrix3d stateRotation;
    stateRotation = AngleAxisd(state(3), Vector3d::UnitX()) * AngleAxisd(state(4), Vector3d::UnitY()) * AngleAxisd(state(5), Vector3d::UnitZ());

    Vector3d x = stateRotation * obj->getBase()->getX();
    Vector3d y = stateRotation * obj->getBase()->getY();
    Vector3d z = stateRotation * obj->getBase()->getZ();
//    glBegin(GL_LINES);
//        glColor3d(1,0,0);
//        glVertex3d(state(0), state(1), state(2));
//        glVertex3d(state(0)+x(0), state(1)+x(1), state(2)+x(2));

//        glColor3d(0,1,0);
//        glVertex3d(state(0), state(1), state(2));
//        glVertex3d(state(0)+y(0), state(1)+y(1), state(2)+y(2));

//        glColor3d(0,0,1);
//        glVertex3d(state(0), state(1), state(2));
//        glVertex3d(state(0)+y(0), state(1)+y(1), state(2)+y(2));
//    glEnd();
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

        double angX = 0;//countang++ * 2;
        AngleAxisd rotX(angX * (M_PI / 180.0), Vector3d::UnitX());
        Vector3d p = rotX.toRotationMatrix() * 5*Vector3d::UnitX();

        double angY = 0;//countang++ * 2;
        AngleAxisd rotY(angY * (M_PI / 180.0), Vector3d::UnitY());
        p = rotY.toRotationMatrix() * p;

        double angZ = 90; //- countang++*2;
        AngleAxisd rotZ(angZ * (M_PI / 180.0), Vector3d::UnitZ());
        p = rotZ.toRotationMatrix() * p;

//        ns << p, angX * (M_PI / 180.0), angY * (M_PI / 180.0), angZ * (M_PI / 180.0);
//        ns << 0, 2.5, 0, 0, 0, M_PI;
        ns << 1+(M_SQRT2/2), 2+(M_SQRT2/2), 1, 0, 0, 0;
        stateDesired = ns;
        obj->iterationScheme(ns, 0.001, 1000);
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
