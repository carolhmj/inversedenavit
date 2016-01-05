#include "joint.h"
#include <cmath>
#include <GL/gl.h>

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
    double cosPrev = almostEqual(cos(prevtwist), 0) ? 0. : cos(angle);
    double sinAngle = sin(angle);
    double sinPrev = sin(prevtwist);
    //Vamos preencher os campos da matriz de acordo com os especificados no livro de referência, p. 44, Eq. 2.58
//    m <<          cos(angle)      ,        -sin(angle)       ,        0       ,      prevlength       ,
//         cos(prevtwist)*sin(angle), cos(prevtwist)*cos(angle), -sin(prevtwist), -offset*sin(prevtwist),
//         sin(prevtwist)*sin(angle), sin(prevtwist)*cos(angle),  cos(prevtwist),  offset*cos(prevtwist),
//                      0           ,             0            ,        0       ,          1            ;
    m <<     cosAngle    ,    -sinAngle    ,     0   ,    prevlength  ,
         cosPrev*sinAngle, cosPrev*cosAngle, -sinPrev, -offset*sinPrev,
         sinPrev*sinAngle, sinPrev*cosAngle,  cosPrev,  offset*cosPrev,
                0        ,        0        ,     0   ,        1       ;
    return m;
}

void Joint::draw(Matrix4d mv)
{
    //Calcula o eixo x
    Vector4d drawX;
    drawX << x, 0;
    drawX = mv*drawX;
    //Calcula o eixo y
    Vector4d drawY;
    drawY << y, 0;
    drawY = mv*drawY;
    //Calcula o eixo z
    Vector4d drawZ;
    drawZ << z, 0;
    drawZ = mv*drawZ;
    //Desenha os eixos
    glBegin(GL_LINES);
        glColor3d(1.0,0.0,0.0);
        glColor3d(0.0,1.0,0.0);
        glColor3d(0.0,0.0,1.0);
    glEnd();

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

