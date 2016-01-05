#include "figure.h"
#include <iostream>
#include <cmath>

using namespace std;

Vector6d Figure::getState() const
{
    return state;
}

void Figure::draw(Matrix4d mv)
{
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    while (curr != NULL){
        curr->draw(mv);
        curr = curr->getNext();
    }
}

Figure::Figure(Joint *base)
{
    this->base = base;
}

//Função que percorre as juntas da figura e gera a matriz de transformação do end effector para a base
Matrix4d Figure::getEndToBaseTransform()
{
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    int act = 0;
    Matrix4d T = Matrix4d::Identity();
    while (curr->getNext() != NULL){
        curr = curr->getNext();
        act++;
        cout << "Matriz da junta " << act << " para a junta " << act-1 << ":\n" << curr->getTransform() << "\n";
        T = T*curr->getTransform();
    }

    return T;
}

/* A partir da transformação do end effector para a base, calculamos o state vector, seguindo a explicação da
 * página 49-51 do livro de referência
 */
void Figure::calcStateVector()
{
    Matrix4d M = getEndToBaseTransform();
    //Calculando os valores de translação
    state(0) = M(0,3);
    state(1) = M(1,3);
    state(2) = M(2,3);

    //Calculando o yaw angle
    state(3) = atan2(M(2,1), M(2,2));

    //Calculando o pitch angle
    double sinState = sin(state(3));
    double cosState = Joint::almostEqual(cos(state(3)), 0.) ? 0. : cos(state(3));
    state(4) = atan2(-M(2,0), M(2,1)*sinState + M(2,2)*cosState);

    //Calculando o roll angle
    state(5) = atan2(M(1,0), M(0,0));
}

