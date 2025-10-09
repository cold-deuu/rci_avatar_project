#ifndef __task_base_hpp__
#define __task_base_hpp__
// Pinocchio Header
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/QR>

// CPP Header
#include <string>

// CHAN HQP CONTROLLER 
#include <rci_h12_controller/constraint/constraint_base.hpp>
#include <rci_h12_controller/constraint/constraint_equality.hpp>
#include <rci_h12_controller/constraint/constraint_inequality.hpp>
#include <rci_h12_controller/utils/math.hpp>

namespace chanhqp{
    namespace task{
        class TaskBase{
            public:
                TaskBase(const std::string & name);
                
                // Reference
                virtual bool getSE3Traj(pinocchio::SE3 trajSample) = 0;
                virtual bool getVectorTraj(Eigen::VectorXd trajSample) = 0;
     
                virtual const std::string & name() const;
                virtual const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const =0;

                // Gain 
                virtual void Kp(double Kp);
                virtual void Kd(double Kd);

                // Compute
                virtual std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state) = 0;

                // // Control
                // virtual bool isPositionControl() const = 0;
                // virtual bool isTorqueControl() const = 0;
            
            
            protected:
                std::string name_;
                double Kp_, Kd_;

            };
    };
};


#endif
