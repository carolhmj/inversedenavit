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
    //Calcula a origem
    Vector4d drawOrigin;
    drawOrigin << origin, 1;
    drawOrigin = mv*drawOrigin;
    //cout << "draw origin:\n" << drawOrigin << "\n\n";
    //Calcula o eixo x
    Vector4d drawX;
    drawX << x, 0;
    drawX = mv*drawX;
    //cout << "draw X:\n" << drawX << "\n\n";
    //Calcula o eixo y
    Vector4d drawY;
    drawY << y, 0;
    drawY = mv*drawY;
    //cout << "draw Y:\n" << drawY << "\n\n";
    //Calcula o eixo z
    Vector4d drawZ;
    drawZ << z, 0;
    drawZ = mv*drawZ;
    //cout << "draw Z:\n" << drawZ << "\n\n";
    //Desenha os eixos
    glBegin(GL_LINES);
        glColor3d(1.0,0.0,0.0); //Eixo x é vermelho
        glVertex3d(drawOrigin(0), drawOrigin(1), drawOrigin(2));
        glVertex3d(drawOrigin(0)+drawX(0), drawOrigin(1)+drawX(1), drawOrigin(2)+drawX(2));
        glColor3d(0.0,1.0,0.0); //Eixo y é verde
        glVertex3d(drawOrigin(0), drawOrigin(1), drawOrigin(2));
        glVertex3d(drawOrigin(0)+drawY(0), drawOrigin(1)+drawY(1), drawOrigin(2)+drawY(2));
        glColor3d(0.0,0.0,1.0); //Eixo z é azul
        glVertex3d(drawOrigin(0), drawOrigin(1), drawOrigin(2));
        glVertex3d(drawOrigin(0)+drawZ(0), drawOrigin(1)+drawZ(1), drawOrigin(2)+drawZ(2));
    glEnd();

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
}

