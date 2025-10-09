#ifndef __constraint_base_hpp__
#define __constraint_base_hpp__

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/QR>

// CPP Header
#include <string>

namespace chanhqp{
    namespace constraint{
        class ConstraintBase
        {
            public:
                ConstraintBase(const std::string & name);
                virtual ~ConstraintBase(){};

                virtual const std::string & name() const;

                virtual bool isEquality() const = 0;
                virtual bool isInequality() const = 0;
                virtual bool isBound() const = 0;

                virtual bool setMatrix(Eigen::MatrixXd matrix) = 0;
                virtual bool setVector(Eigen::VectorXd vector) = 0;
                virtual bool setLowerBound(Eigen::VectorXd lower) = 0;
                virtual bool setUpperBound(Eigen::VectorXd upper) = 0;

                virtual Eigen::MatrixXd getMatrix(){return A_;}
                virtual Eigen::VectorXd getVector(){return b_;}


                virtual bool checkConstraint(Eigen::VectorXd x, double tol=1e-6) const = 0;


            protected:
                std::string name_;
                Eigen::MatrixXd A_;
                Eigen::VectorXd b_;
                
        };
    }
}

#endif