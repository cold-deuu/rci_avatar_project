#ifndef __task_base_fix_equality_hpp__
#define __task_base_fix_equality_hpp__

#include <rci_h12_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskBaseFixEquality : public TaskBase
        {
            public:
                TaskBaseFixEquality(const std::string & name);
                
                bool getSE3Traj(pinocchio::SE3 trajSample);
                bool getVectorTraj(Eigen::VectorXd );

                // chanhqp::constraint::ConstraintBase & compute(State state);
                std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state);
                const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const;


            protected:
                // chanhqp::constraint::ConstraintEquality constraint_;
                std::shared_ptr<chanhqp::constraint::ConstraintEquality> constraint_;

        };
    };
};

#endif
