#include "joint.h"
#include <cmath>
#include <GL/gl.h>
#include <iostream>

using namespace std;
Vector3d Joint::getX() const
{
    return x;
}

void Joint::setX(const Vector3d &value)
{
    x = value;
}

Vector3d Joint::getY() const
{
    return y;
}

void Joint::setY(const Vector3d &value)
{
    y = value;
}

Vector3d Joint::getZ() const
{
    return z;
}

void Joint::setZ(const Vector3d &value)
{
    z = value;
}

double Joint::getLength() const
{
    return length;
}

void Joint::setLength(double value)
{
    length = value;
}

double Joint::getTwist() const
{
    return twist;
}

void Joint::setTwist(double value)
{
    twist = value;
}

double Joint::getOffset() const
{
    return offset;
}

void Joint::setOffset(double value)
{
    offset = value;
}

double Joint::getAngle() const
{
    return angle;
}

void Joint::setAngle(double value)
{
    angle = value;
}

Joint *Joint::getPrev() const
{
    return prev;
}

void Joint::setPrev(Joint *value)
{
    prev = value;
}

Joint *Joint::getNext() const
{
    return next;
}

void Joint::setNext(Joint *value)
{
    next = value;
    value->setPrev(this);
}

//Função de comparação entre dois números, definida para corrigir um problema de arredondamento
bool Joint::almostEqual(double x, double y)
{
    double epsilon = 0.0000001;
    if (abs(x-y) < epsilon) {
        return true;
    } else {
        return false;
    }
}

Matrix4d Joint::getTransform()
{
    if (this->prev == NULL){
        return Matrix4d::Identity();
    }
    Matrix4d m;
    //Copiando os parâmetros aqui pra ficar mais fácil de ler
    double angle = this->angle;
    double prevtwist = this->prev->getTwist();
    double prevlength = this->prev->getLength();
    double offset = this->offset;
    /* Calculando os senos e os cosenos, considerando o erro de arredondamento que faz cos(M_PI_2) não retornar
     * zero como deveria, problema descrito em http://stackoverflow.com/questions/1605435/cosfm-pi-2-not-returning-zero
     */
    double cosAngle = almostEqual(cos(angle), 0) ? 0. : cos(angle);
    double cosPrev = almostEqual(cos(prevtwist), 0) ? 0. : cos(prevtwist);
    double sinAngle = sin(angle);
    double sinPrev = sin(prevtwist);
    //Vamos preencher os campos da matriz de acordo com os especificados no livro de referência, p. 44, Eq. 2.58
    m <<     cosAngle    ,    -sinAngle    ,     0   ,    prevlength  ,
         cosPrev*sinAngle, cosPrev*cosAngle, -sinPrev, -offset*sinPrev,
         sinPrev*sinAngle, sinPrev*cosAngle,  cosPrev,  offset*cosPrev,
                0        ,        0        ,     0   ,        1       ;
    return m;
}

Matrix4d Joint::getParentMatrix()
{
   if (this->prev == NULL) {
       return Matrix4d::Identity();
   }
   return this->prev->getParentMatrix() * this->prev->getTransform();
}

Matrix4d Joint::getChildMatrix()
{
    if (this->next == NULL){
        return Matrix4d::Identity();
    }
    return this->next->getTransform() * this->next->getChildMatrix();
}

void Joint::draw(Matrix4d mv)
{
    //Desenha os eixos
    drawPyramid(origin, originNext, 0.4, mv);

}

void Joint::drawPyramid(Vector3d bottom, Vector3d top, double base, Matrix4d mv)
{
//    Vector3d v1 = top;
//    Vector3d v2(bottom(0)+base/2, bottom(1), bottom(2)+base/2);
//    Vector3d v3(bottom(0)+base/2, bottom(1), bottom(2)-base/2);
//    Vector3d v4(bottom(0)-base/2, bottom(1), bottom(2)-base/2);
//    Vector3d v5(bottom(0)-base/2, bottom(1), bottom(2)+base/2);

//    glBegin(GL_TRIANGLES);
//        glVertex3f();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(mv.data());

    const GLdouble pyramidVertices[] = {
        top(0), top(1), top(2),
        bottom(0)+base/2, bottom(1), bottom(2)+base/2,
        bottom(0)+base/2, bottom(1), bottom(2)-base/2,
        bottom(0)-base/2, bottom(1), bottom(2)-base/2,
        bottom(0)-base/2, bottom(1), bottom(2)+base/2,
    };

    const GLdouble pyramidColors[] = {
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
        color(0), color(1), color(2),
    };

    unsigned int index[] = {
        0,2,1,
        0,3,2,
        0,4,3,
        0,1,4,
        3,1,4,
        1,3,2
    };
    glPolygonMode(GL_FRONT, GL_LINE);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColorPointer(3, GL_DOUBLE, 0, pyramidColors);
    glVertexPointer(3, GL_DOUBLE, 0, pyramidVertices);
    glDrawElements(GL_TRIANGLE_FAN, 18, GL_UNSIGNED_INT, index);
}


Vector3d Joint::getOrigin() const
{
    return origin;
}

void Joint::setOrigin(const Vector3d &value)
{
    origin = value;
}

Vector3d Joint::getOriginNext() const
{
    return originNext;
}

void Joint::setOriginNext(const Vector3d &value)
{
    originNext = value;
}
Joint::Joint()
{

}

Joint::Joint(Vector3d x, Vector3d y, Vector3d z, double length, double twist, double offset, double angle)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->length = length;
    this->twist = twist;
    this->offset = offset;
    this->angle = angle;
}

Joint::Joint(Vector3d x, Vector3d y, Vector3d z, double length, double twist, double offset, double angle, Vector3d origin, Vector3d originNext)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->length = length;
    this->twist = twist;
    this->offset = offset;
    this->angle = angle;
    this->origin = origin;
    this->originNext = originNext;

    Vector3d rand = Vector3d::Random();
    color = rand/rand.maxCoeff();
}

