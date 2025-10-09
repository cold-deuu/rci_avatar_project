#ifndef __constraint_equality_hpp__
#define __constraint_equality_hpp__

#include <rci_h12_controller/constraint/constraint_base.hpp>



namespace chanhqp{
    namespace constraint{
        class ConstraintEquality : public ConstraintBase
        {
            public:
                ConstraintEquality(const std::string & name);
                virtual ~ConstraintEquality(){};
                
                bool isEquality() const {return true;}
                bool isInequality() const {return false;}
                bool isBound() const {return false;}

                bool setMatrix(Eigen::MatrixXd matrix);
                bool setVector(Eigen::VectorXd vector);
                bool setLowerBound(Eigen::VectorXd lower);
                bool setUpperBound(Eigen::VectorXd upper);

                bool checkConstraint(Eigen::VectorXd x, double tol=1e-6) const;


        };
    }
}
#endif