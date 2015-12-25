#include "figure.h"

Figure::Figure(Joint *base)
{
    this->base = base;
}

//Função que percorre as juntas da figura e gera a matriz de transformação do end effector para a base
void Figure::getEndToBaseTransform()
{
    Joint *curr; //Guarda a junta que está sendo percorrida
    Matrix4d T = Matrix4d::Identity();
    while (curr->next != NULL){
        curr = curr->next;
        T = T*curr->getTransform();
    }

    return T;
}

