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

    Vector3d color; //Cor de renderização
public:
    //Construtores
    Joint();
    Joint(Vector3d x, Vector3d y, Vector3d z, double length, double twist, double offset, double angle);
    Joint(Vector3d x, Vector3d y, Vector3d z, double length, double twist, double offset, double angle, Vector3d origin, Vector3d originNext);

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
    Vector3d getOrigin() const;
    void setOrigin(const Vector3d &value);
    Vector3d getOriginNext() const;
    void setOriginNext(const Vector3d &value);

    static bool almostEqual(double x, double y);
    //Gera a matriz de transformação que transforma entidades do sistema atual para o sistema anterior
    Matrix4d getTransform();

    //Retorna a matriz de transformação de todos os pais da junta
    Matrix4d getParentMatrix();

    //Retorna a matriz de transformação de todos os filhos da junta
    Matrix4d getChildMatrix();

    //Desenhar a junta
    void draw(Matrix4d mv = Matrix4d::Identity());

    //Desenha uma pirâmide com base (xb,yb,zb) e topo (xt,yt,zt), base b e cor c
    void drawPyramid(Vector3d bottom, Vector3d top, double base, Matrix4d mv = Matrix4d::Identity());
};

#endif // JOINT_H
