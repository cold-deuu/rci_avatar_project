#include <rci_h12_controller/constraint/constraint_equality.hpp>

using namespace chanhqp::constraint;

// A_ * x = b_


ConstraintEquality::ConstraintEquality(const std::string & name):
    ConstraintBase(name){};

bool ConstraintEquality::setMatrix(Eigen::MatrixXd matrix)
{
    A_ = matrix;
    return true;
}

bool ConstraintEquality::setVector(Eigen::VectorXd vector)
{
    b_ = vector;
    return true;
}

bool ConstraintEquality::setLowerBound(Eigen::VectorXd )
{ 
    assert(false);
    return false;
}
bool ConstraintEquality::setUpperBound(Eigen::VectorXd )
{ 
    assert(false);
    return false; 
}

bool ConstraintEquality::checkConstraint(Eigen::VectorXd x, double tol) const
{
    return (A_*x - b_).norm() < tol;
}