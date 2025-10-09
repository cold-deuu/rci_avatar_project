#ifndef __task_se3_equality_hpp__
#define __task_se3_equality_hpp__

#include <rci_h12_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskSE3Equality : public TaskBase
        {
            public:
                TaskSE3Equality(const std::string & name);
                
                bool getSE3Traj(pinocchio::SE3 trajSample);
                bool getVectorTraj(Eigen::VectorXd );

                // chanhqp::constraint::ConstraintBase & compute(State state);
                std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state);
                const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const;

            protected:
                std::shared_ptr<chanhqp::constraint::ConstraintEquality> constraint_;
                // chanhqp::constraint::ConstraintEquality constraint_;
                pinocchio::SE3 ref_se3_;
        };
    };
};

#endif
