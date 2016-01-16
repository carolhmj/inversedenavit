#ifndef FIGURE_H
#define FIGURE_H

#include "joint.h"

typedef Matrix<double, 6, 1> Vector6d;

/*
 * Classe que representa uma figura articulada. Ela guarda a junta base da articulação e tem a responsabilidade de calcular
 * o state vector do end effector
 */
class Figure
{
private:
    Joint* base;
    Vector6d state;
public:
    Figure(Joint* base);
    int getNumJoints();
    Matrix4d getEndToBaseTransform();
    void calcStateVector();
    Vector6d getState() const;
    Vector6d getDerivativeStateVector(Joint *j);
    MatrixXd getJacobian();
    MatrixXd getInverse(MatrixXd M);
    void setParams(VectorXd params);
    void recalculateAxis();
    void iterationScheme(Vector6d target, double tolerance, int maxSteps);
    VectorXd computeJointParameters(VectorXd parameters, Vector6d current, Vector6d target);
    void draw(Matrix4d mv);

    //Função para calcular a pseudo-inversa, fonte: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }
    Joint *getBase() const;
};

#endif // FIGURE_H
