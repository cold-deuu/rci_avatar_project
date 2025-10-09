#ifndef __constraint_inequality_hpp__
#define __constraint_inequality_hpp__

#include <rci_h12_controller/constraint/constraint_base.hpp>

namespace chanhqp{
    namespace constraint{
        class ConstraintInequality : public ConstraintBase
        {
            public:
                ConstraintInequality(const std::string & name);
                virtual ~ConstraintInequality(){};
                
                bool isEquality() const {return false;}
                bool isInequality() const {return true;}
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