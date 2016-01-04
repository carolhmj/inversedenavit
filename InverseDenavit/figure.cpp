#include "figure.h"
#include <iostream>
#include <cmath>

using namespace std;

Vector6d Figure::getState() const
{
    return state;
}
Figure::Figure(Joint *base)
{
    this->base = base;
}

//Função que percorre as juntas da figura e gera a matriz de transformação do end effector para a base
Matrix4d Figure::getEndToBaseTransform()
{
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    int act = 1;
    Matrix4d T = Matrix4d::Identity();
    while (curr->getNext() != NULL){
        curr = curr->getNext();
        cout << "Matriz da junta: " << act << ":\n" << curr->getTransform() << "\n";
        act++;
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
    state(4) = atan2(-M(2,0), M(2,1)*sin(state(3)) + M(2,2)*cos(state(3)));

    //Calculando o roll angle
    state(5) = atan2(M(1,0), M(0,0));
}

