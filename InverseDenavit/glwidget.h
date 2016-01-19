#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QTimer>
#include "figure.h"
#include "trackball.h"

class GLWidget : public QGLWidget
{
private:
    Figure *obj = NULL;
    QTimer timer;

    float curquat[4];
    float lastquat[4];
    int beginx, beginy;
    float zoom = 0.10;
    float verticalIncrease, horizontalIncrease;
    Matrix4d mvMatrix;
    int countang = 1;
    Vector6d stateDesired;

public:
    explicit GLWidget(QWidget *parent = 0);
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    Matrix4d perspectiveMatrix(double fov, double far, double near);
    Matrix4d ortographicMatrix(double far, double near, double left, double right, double top, double bottom);
    Matrix4d lookAtMatrix(Vector3d pos, Vector3d target, Vector3d up);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    Matrix4d perspectiveMatrix(double fovY, double aspect, double near, double far);
    void drawCube(Vector3d center, double size, double color[], Matrix4d mv = Matrix4d::Identity());
    void drawTarget(Vector6d state, Matrix4d mv = Matrix4d::Identity());
};

#endif // GLWIDGET_H
