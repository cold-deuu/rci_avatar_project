#ifndef __task_sca_equality_hpp__
#define __task_sca_equality_hpp__

#include <rci_h12_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskSCAInequality : public TaskBase
        {
            public:
                TaskSCAInequality(const std::string & name, bool is_wholebody);
                
                bool getSE3Traj(pinocchio::SE3 trajSample);
                bool getVectorTraj(Eigen::VectorXd );

                // chanhqp::constraint::ConstraintBase & compute(State state);
                std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state);
                const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const;


            protected:
                // chanhqp::constraint::ConstraintInequality constraint_;
                std::shared_ptr<chanhqp::constraint::ConstraintInequality> constraint_;
                bool is_wholebody_;

        };
    };
};

#endif
