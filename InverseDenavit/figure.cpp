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

/*
 * Vamos calcular a derivada do state vector de acordo com o descrito no livro nas páginas
 * 65 a 69
 */
Vector6d Figure::getDerivativeStateVector(Joint *j)
{
    Vector6d s;
    Matrix4d P = j->getParentMatrix(), C = j->getChildMatrix();
    double cosAngle = Joint::almostEqual(cos(j->getAngle()), 0.0) ? 0.0 : cos(j->getAngle());
    //Ele precisa do twist anterior, logo a junta base não tem derivada?
    double cosPrev = Joint::almostEqual(cos(j->getPrev()->getTwist()), 0.0) ? 0.0 : cos(j->getPrev()->getTwist());
    double sinAngle =  sin(j->getAngle());
    double sinPrev = sin(j->getPrev()->getTwist());

    s(0) = cosAngle * (-P(0,0)*C(1,3) + P(0,1)*C(0,3)*cosPrev + P(0,2)*C(0,3)*sinPrev) -
           sinAngle * ( P(0,0)*C(0,3) + P(0,1)*C(1,3)*cosPrev + P(0,2)*C(1,3)*sinPrev);
    s(1) = cosAngle * (-P(1,0)*C(1,3) + P(1,1)*C(0,3)*cosPrev + P(1,2)*C(0,3)*sinPrev) -
           sinAngle * ( P(1,0)*C(0,3) + P(1,1)*C(1,3)*cosPrev + P(1,2)*C(1,3)*sinPrev);
    s(2) = cosAngle * (-P(2,0)*C(1,3) + P(2,1)*C(0,3)*cosPrev + P(2,2)*C(0,3)*sinPrev) -
           sinAngle * ( P(2,0)*C(0,3) + P(2,1)*C(1,3)*cosPrev + P(2,2)*C(1,3)*sinPrev);

    double x = -P(2,1)*C(2,1)*sinPrev + cosAngle*(P(2,0)*C(0,1) + P(2,2)*C(1,1)*sinPrev)
               -P(2,0)*C(1,1)*sinAngle + P(2,2)*C(0,1)*sinPrev*sinAngle
               +cosPrev*(P(2,2)*C(2,1) + P(2,1)*C(1,1)*cosAngle + P(2,1)*C(0,1)*sinAngle);
    double y = -P(2,1)*C(2,2)*sinPrev + cosAngle*(P(2,0)*C(0,2) + P(2,2)*C(1,2)*sinPrev)
               -P(2,0)*C(1,2)*sinAngle + P(2,2)*C(0,2)*sinPrev*sinAngle
               +cosPrev*(P(2,2)*C(2,2) + P(2,1)*C(1,2)*cosAngle + P(2,1)*C(0,2)*sinAngle);
    double dx = cosAngle * (-P(2,0)*C(1,1) + P(2,1)*C(0,1)*cosPrev + P(2,2)*C(0,1)*sinPrev ) -
                sinAngle * (P(2,0)*C(0,1) + P(2,1)*C(1,1)*cosPrev + P(2,2)*C(1,1)*sinPrev );
    double dy = cosAngle * (-P(2,0)*C(1,2) + P(2,1)*C(0,2)*cosPrev + P(2,2)*C(0,2)*sinPrev) -
                sinAngle * (P(2,0)*C(0,2) + P(2,1)*C(1,2)*cosPrev + P(2,2)*C(1,2)*sinPrev);

    s(3) = (1/(1+pow(x/y,2.0)))*((1/y)*dx - (x/pow(y,2.0))*dy);

    double a = -P(2,1)*C(2,0)*sinPrev + cosAngle*(P(2,0)*C(0,0) + P(2,2)*C(1,0)*sinPrev)
               -P(2,0)*C(1,0)*sinAngle + P(2,2)*C(0,0)*sinPrev*sinAngle
               +cosPrev*(P(2,2)*C(2,0) + P(2,1)*C(1,0)*cosAngle + P(2,1)*C(0,0)*sinAngle);
    double b = -P(2,1)*C(2,1)*sinPrev + cosAngle*(P(2,0)*C(0,1) + P(2,2)*C(1,1)*sinPrev)
               -P(2,0)*C(1,1)*sinAngle + P(2,2)*C(0,1)*sinPrev*sinAngle
               +cosPrev*(P(2,2)*C(2,1) + P(2,1)*C(1,1)*cosAngle + P(2,1)*C(0,1)*sinAngle);
    double c = -P(2,1)*C(2,2)*sinPrev + cosAngle*(P(2,0)*C(0,2) + P(2,2)*C(1,2)*sinPrev)
               -P(2,0)*C(2,1)*sinAngle + P(2,2)*C(0,2)*sinPrev*sinAngle
               +cosPrev*(P(2,2)*C(2,2) + P(2,1)*C(1,2)*cosAngle + P(2,1)*C(0,2)*sinAngle);
    double da = cosAngle * (-P(2,0)*C(1,0) + P(2,1)*C(0,0)*cosPrev + P(2,2)*C(0,0)*sinPrev)
                -sinAngle * (P(2,0)*C(0,0) + P(2,1)*C(1,0)*cosPrev + P(2,2)*C(1,0)*sinPrev);
    double db = cosAngle * (-P(2,0)*C(1,1) + P(3,1)*C(0,1)*cosPrev + P(2,2)*C(0,1)*sinPrev)
                -sinAngle * (P(2,0)*C(0,1) + P(2,1)*C(1,1)*cosPrev + P(2,2)*C(1,1)*sinPrev);
    double dc = cosAngle * (-P(2,0)*C(1,2) + P(2,1)*C(0,2)*cosPrev + P(2,2)*C(0,2)*sinPrev)
                -sinAngle * (P(2,0)*C(0,2) + P(2,1)*C(1,2)*cosPrev + P(2,2)*C(1,2)*sinPrev);
    double N = -a;

    double sins3 = sin(state(3));
    double coss3 = Joint::almostEqual(cos(state(3)), 0.0)? 0.0 : cos(state(3));

    double D = b*sins3 + c*coss3;

    double dN = da;
    double dD = dc*coss3 - c*sins3*s(3);

    s(4) = (1/(1+pow(N/D, 2.0)))*((1/D)*dN - (N/pow(D,2.0))*dD);


    double p = -P(0,1)*C(2,0)*sinPrev + cosAngle * (P(0,0)*C(0,0) + P(0,2)*C(1,0)*sinPrev)
               -P(0,0)*C(1,0)*sinAngle + P(0,2)*C(0,0)*sinPrev*sinAngle
               +cosPrev*(P(0,2)*C(2,0) + P(0,1)*C(1,0)*cosAngle + P(0,1)*C(0,0)*sinAngle);
    double q = -P(1,1)*C(2,0)*sinPrev + cosAngle * (P(1,0)*C(0,0) + P(1,2)*C(1,0)*sinPrev)
               -P(1,0)*C(1,0)*sinAngle + P(1,2)*C(0,0)*sinPrev*sinAngle
               +cosPrev*(P(1,2)*C(2,0) + P(1,1)*C(1,0)*cosAngle + P(1,1)*C(0,0)*sinAngle);
    double dp = cosAngle * (-P(0,0)*C(1,0) + P(0,1)*C(0,0)*cosPrev + P(0,2)*C(0,0)*sinPrev)
                -sinAngle * (P(0,0)*C(0,0) + P(0,1)*C(0,1)*cosPrev + P(0,2)*C(1,0)*sinPrev);
    double dq = cosAngle * (P(1,0)*C(1,0) + P(1,1)*C(0,0)*cosPrev + P(1,2)*C(0,0)*sinPrev)
                -sinAngle * (P(1,0)*C(0,0) + P(1,1)*C(1,0)*cosPrev + P(1,2)*C(1,0)*sinPrev);

    s(5) = (1/(1+pow(q/p, 2.0)))*((1/p)*dq - (q/pow(p,2.0))*dp);
    return s;
}

//Computa a Jacobiana do vetor de estados
MatrixXd Figure::getJacobian()
{
    MatrixXd J;
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    int n = 0;
    //Percorre as juntas da estrutura e adiciona as derivadas do vetor de estado como colunas da jacobiana
    while (curr != NULL){
        J.col(n) = getDerivativeStateVector(curr);
        n++;
        curr = curr->getNext();
    }
    return J;
}

//Retorna a inversa (ou pseudo-inversa) da matriz M
MatrixXd Figure::getInverse(MatrixXd M)
{
    //Caso invertível
    if (M.rows() > M.cols()) {
        return M.inverse();
    }
    //Caso over-determined
    if (M.rows() > M.cols()) {
        return (M.transpose() * M).inverse() * M.transpose();
    }
    //Caso under-determined
    if (M.rows() < M.cols()) {
        return M.transpose() * (M * M.transpose()).inverse();
    }
}

void Figure::setParams(VectorXd params)
{
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    int n = 0;
    //Percorre as juntas da estrutura e atualiza os parâmetros da junta
    while (curr != NULL){
        curr->setAngle(params(n));
        n++;
        curr = curr->getNext();
    }
}

void Figure::iterationScheme(Vector6d target, double tolerance)
{
    Joint *curr = this->base; //Guarda a junta que está sendo percorrida
    int n = 0;
    VectorXd currParams;
    //Percorre as juntas da estrutura e adiciona os parâmetros da junta em um vetor
    while (curr != NULL){
        currParams(n) = curr->getAngle();
        n++;
        curr = curr->getNext();
    }

    while (abs((state-target).norm()) > tolerance){
        VectorXd newParams = computeJointParameters(currParams, state, target);
        setParams(newParams);
        calcStateVector();
    }
}

/*
 * Função que representa um passo no esquema de iteração, calculando os novos parâmetros da junta
 * de acordo com a Eq. 3.38, p. 54
 */
VectorXd Figure::computeJointParameters(VectorXd parameters, Vector6d current, Vector6d target)
{
    return parameters - (getInverse(getJacobian()))*(current-target);
}
