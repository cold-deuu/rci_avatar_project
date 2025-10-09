#include <rci_h12_controller/task/task_base_fix_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskBaseFixEquality::TaskBaseFixEquality(const std::string & name):
    TaskBase(name), constraint_(std::make_shared<ConstraintEquality>(name)){};

bool TaskBaseFixEquality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskBaseFixEquality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

// ConstraintBase & TaskBaseFixEquality::compute(State state)
// {
//     Eigen::MatrixXd J_base = Eigen::MatrixXd::Zero(6,state.n_var);
//     Eigen::VectorXd vel_base = Eigen::VectorXd::Zero(6);
//     J_base(0,0) = J_base(1,1) = J_base(5,2) = 1;

//     // J_base.topLeftCorner(6,3) = state.J.topLeftCorner(6,3);
    
//     // J_base * qdot = 0
//     constraint_.setMatrix(J_base);
//     constraint_.setVector(vel_base);

//     return constraint_;
// }

std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskBaseFixEquality::compute(State state)
{
    // Eigen::MatrixXd J_base = Eigen::MatrixXd::Zero(6,state.n_var);
    // Eigen::VectorXd vel_base = Eigen::VectorXd::Zero(6);
    // J_base(0,0) = J_base(1,1) = J_base(5,2) = 1;

    // J_base.topLeftCorner(6,3) = state.J.topLeftCorner(6,3);
    
    // J_base * qdot = 0

    Eigen::MatrixXd base_mat = Eigen::MatrixXd::Zero(state.n_mobile,state.n_var);
    Eigen::VectorXd vel_base = Eigen::VectorXd::Zero(6);
    base_mat.topLeftCorner(state.n_mobile, state.n_mobile).setIdentity();

    constraint_ -> setMatrix(base_mat);
    constraint_ -> setVector(vel_base);

    return constraint_;
}


const std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskBaseFixEquality::constraint() const
{
  return constraint_;
}
