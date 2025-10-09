#include <rci_h12_controller/task/task_sca_inequality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskSCAInequality::TaskSCAInequality(const std::string & name, bool is_wholebody):
    TaskBase(name), constraint_(std::make_shared<ConstraintInequality>(name)), is_wholebody_(is_wholebody){};

bool TaskSCAInequality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskSCAInequality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskSCAInequality::compute(State state)
{
    // Eigen::MatrixXd sca_ineq_mat(1,9);
    // Eigen::VectorXd sca_ineq_vec(1);
    // sca_ineq_mat.setZero();
    // sca_ineq_mat.block<1,6>(0,3) = state.sca_dgamma_dq;
    // sca_ineq_mat(0,9) = 1;
    // sca_ineq_vec(0) = 0.1 * std::log(state.sca_gamma+1);

    // int manip = state_.n_var - 3;

    Eigen::MatrixXd sca_ineq_mat(1,state.n_var);
    Eigen::VectorXd sca_ineq_vec(1);
    sca_ineq_mat.setZero();

    sca_ineq_mat.block(0, state.n_mobile, 1, state.n_manip) = state.sca_dgamma_dq["manip"];
    sca_ineq_vec(0) = 0.1 * std::log(state.sca_gamma["manip"]+1);

    if (is_wholebody_)
    {
        sca_ineq_mat.conservativeResize(2,state.n_var);
        sca_ineq_vec.conservativeResize(2);
        sca_ineq_mat.block(1, state.n_mobile, 1, state.n_manip) = state.sca_dgamma_dq["mobile"];
        sca_ineq_vec(1) = 0.1 * std::log(state.sca_gamma["mobile"]+1);
    }

    constraint_->setMatrix(sca_ineq_mat);
    constraint_->setVector(sca_ineq_vec);
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


const std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskSCAInequality::constraint() const
{
  return constraint_;
}
