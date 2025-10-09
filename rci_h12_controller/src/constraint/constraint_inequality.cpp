#include <rci_h12_controller/constraint/constraint_inequality.hpp>

using namespace chanhqp::constraint;

// A_ * x >= b_

ConstraintInequality::ConstraintInequality(const std::string & name):
    ConstraintBase(name){};

bool ConstraintInequality::setMatrix(Eigen::MatrixXd matrix)
{
    A_ = matrix;
    return true;
}

bool ConstraintInequality::setVector(Eigen::VectorXd vector)
{
    b_ = vector;
    return true;
}

bool ConstraintInequality::setLowerBound(Eigen::VectorXd )
{ 
    assert(false);
    return false;
}
bool ConstraintInequality::setUpperBound(Eigen::VectorXd )
{ 
    assert(false);
    return false; 
}

bool ConstraintInequality::checkConstraint(Eigen::VectorXd x, double tol) const
{
    return ((A_*x).array() >= -(b_.array()+tol)).all();
}