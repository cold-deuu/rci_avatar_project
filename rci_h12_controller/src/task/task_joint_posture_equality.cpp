#include <rci_h12_controller/task/task_joint_posture_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskJointPostureEquality::TaskJointPostureEquality(const std::string & name):
    TaskBase(name), constraint_(std::make_shared<ConstraintEquality>(name)){};

bool TaskJointPostureEquality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskJointPostureEquality::getVectorTraj(Eigen::VectorXd trajSample) {
    ref_joint_ = trajSample;
    return true;
}

std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskJointPostureEquality::compute(State state)
{
    Eigen::VectorXd qdes_dot = (ref_joint_ - state.q.tail(state.n_manip)) * Kp_;
    Eigen::MatrixXd manip_eye = Eigen::MatrixXd::Zero(state.n_manip, state.n_var);
    // Eigen::VectorXd joint_pos_vec = Eigen::VectorXd::Zero(state.n_var);
    manip_eye.topRightCorner(state.n_manip, state.n_manip).setIdentity();

    constraint_ -> setMatrix(manip_eye);
    constraint_ -> setVector(qdes_dot);
    return constraint_;
}


const std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskJointPostureEquality::constraint() const
{
  return constraint_;
}
