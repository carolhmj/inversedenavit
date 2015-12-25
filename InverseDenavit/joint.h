#ifndef JOINT_H
#define JOINT_H

#include <eigen3/Eigen/Dense>
using namespace Eigen;

/*
 * Classe que representa uma junta. Ela guarda os parâmetros da junta na notação Denavit-Hartenbeg, e possui a responsabilidade de
 * gerar a matriz de transformação entre sistemas de coordenadas das juntas. Para isso, cada junta tem a informação da sua junta
 * anterior e posterior, podemos assim identificar a junta base como a junta que não possui anterior, e o end effector como a junta
 * que não possui junta posterior.
 */
class Joint
{
private:
    Vector3d x, y, z; //Eixos de coordenadas da junta
    Vector3d origin, originNext; //Origem e origem deslocada
    double length; //Comprimento do link vector
    double twist; //Link twist, em RADIANOS
    double offset; //Link offset
    double angle; //Joint angle, em RADIANOS
    Joint* prev = NULL; //Junta anterior
    Joint* next = NULL; //Junta posterior
public:
    //Construtores
    Joint();
    Joint(Vector3d x, Vector3d y, Vector3d z, double length, double twist, double offset, double angle);

    //Getters e setters
    Vector3d getX() const;
    void setX(const Vector3d &value);
    Vector3d getY() const;
    void setY(const Vector3d &value);
    Vector3d getZ() const;
    void setZ(const Vector3d &value);
    double getLength() const;
    void setLength(double value);
    double getTwist() const;
    void setTwist(double value);
    double getOffset() const;
    void setOffset(double value);
    double getAngle() const;
    void setAngle(double value);
    Joint *getPrev() const;
    void setPrev(Joint *value);
    Joint *getNext() const;
    void setNext(Joint *value);

    //Gera a matriz de transformação que transforma entidades do sistema atual para o sistema anterior
    Matrix4d getTransform();
};

#endif // JOINT_H
