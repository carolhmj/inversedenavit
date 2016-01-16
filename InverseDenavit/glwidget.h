#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include "figure.h"

class GLWidget : public QGLWidget
{
private:
    Figure *obj = NULL;
public:
    explicit GLWidget(QWidget *parent = 0);
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    Matrix4d perspectiveMatrix(double fov, double far, double near);
    Matrix4d ortographicMatrix(double far, double near, double left, double right, double top, double bottom);
    Matrix4d lookAtMatrix(Vector3d pos, Vector3d target, Vector3d up);
    void mousePressEvent(QMouseEvent *event);
};

#endif // GLWIDGET_H
