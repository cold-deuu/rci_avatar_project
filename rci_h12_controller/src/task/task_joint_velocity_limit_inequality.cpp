#include <rci_h12_controller/task/task_joint_velocity_limit_inequality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskJointVelocityLimitInequality::TaskJointVelocityLimitInequality(const std::string & name, bool is_wholebody):
    TaskBase(name), constraint_(std::make_shared<ConstraintInequality>(name)), is_wholebody_(is_wholebody){
    };

bool TaskJointVelocityLimitInequality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskJointVelocityLimitInequality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

void TaskJointVelocityLimitInequality::setJointVelLimit(std::map<std::string, Eigen::VectorXd> joint_vel_limit)
{
    joint_vel_limit_["upper"] = joint_vel_limit.at("upper");
    joint_vel_limit_["lower"] = joint_vel_limit.at("lower");
}

std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskJointVelocityLimitInequality::compute(State state)
{
    Eigen::MatrixXd joint_limit_mat = Eigen::MatrixXd::Zero(state.n_manip * 2, state.n_var);
    Eigen::VectorXd joint_limit_vec = Eigen::VectorXd::Zero(state.n_manip * 2);

    joint_limit_mat.topRightCorner(state.n_manip, state.n_manip).setIdentity(); // upper
    joint_limit_mat.topRightCorner(state.n_manip, state.n_manip) *= -1;
    joint_limit_mat.bottomRightCorner(state.n_manip, state.n_manip).setIdentity(); // lower

    joint_limit_vec.head(state.n_manip) = joint_vel_limit_.at("upper");
    joint_limit_vec.tail(state.n_manip) = joint_vel_limit_.at("lower") * -1;

    constraint_->setMatrix(joint_limit_mat);
    constraint_->setVector(joint_limit_vec);
    return constraint_;
}


// ConstraintBase & TaskSCAInequality::compute(State state)
// {
//     // Eigen::MatrixXd sca_ineq_mat(1,9);
//     // Eigen::VectorXd sca_ineq_vec(1);
//     // sca_ineq_mat.setZero();
//     // sca_ineq_mat.block<1,6>(0,3) = state.sca_dgamma_dq;
//     // sca_ineq_mat(0,9) = 1;
//     // sca_ineq_vec(0) = 0.1 * std::log(state.sca_gamma+1);

//     Eigen::MatrixXd sca_ineq_mat(2,9);
//     Eigen::VectorXd sca_ineq_vec(2);
//     sca_ineq_mat.setZero();
//     sca_ineq_mat.block<1,6>(0,3) = state.sca_dgamma_dq["mobile"];
//     // sca_ineq_mat(0,9) = 1;
//     sca_ineq_vec(0) = 0.01 * std::log(state.sca_gamma["mobile"]+1);

//     sca_ineq_mat.block<1,6>(1,3) = state.sca_dgamma_dq["manip"];
//     // sca_ineq_mat(0,9) = 1;
//     sca_ineq_vec(1) = 0.1 * std::log(state.sca_gamma["manip"]+1);

//     constraint_.setMatrix(sca_ineq_mat);
//     constraint_.setVector(sca_ineq_vec);
//     return constraint_;
// }


const std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskJointVelocityLimitInequality::constraint() const
{
  return constraint_;
}
