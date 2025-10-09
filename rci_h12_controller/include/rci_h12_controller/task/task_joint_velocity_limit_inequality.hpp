#ifndef __task_joint_velocity_limit_inequality_hpp__
#define __task_joint_velocity_limit_inequality_hpp__

#include <rci_h12_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskJointVelocityLimitInequality : public TaskBase
        {
            public:
                TaskJointVelocityLimitInequality(const std::string & name,  bool is_wholebody);
                
                bool getSE3Traj(pinocchio::SE3 );
                bool getVectorTraj(Eigen::VectorXd trajSample);

                // chanhqp::constraint::ConstraintBase & compute(State state);
                std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state);
                const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const;

                void setJointVelLimit(std::map<std::string, Eigen::VectorXd> joint_vel_limit);

            protected:
                std::shared_ptr<chanhqp::constraint::ConstraintInequality> constraint_;
                // chanhqp::constraint::ConstraintEquality constraint_;

                bool is_wholebody_;                
                std::map<std::string, Eigen::VectorXd> joint_vel_limit_;
        };
    };
};

#endif
