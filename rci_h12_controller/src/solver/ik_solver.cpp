#include "rci_h12_controller/solver/ik_solver.hpp"

using namespace pinocchio;


namespace rci_ik_solver
{
    IkSolver::IkSolver(int nq, int nv, double dt)
    : nq_(nq), nv_(nv), dt_(dt)
    {
        Kp_.resize(6);
        Kd_.resize(6);
        Kp_ << 100, 100, 100, 100, 100, 100;
        Kd_ << 40, 40, 40, 30, 30, 30;

    };
    void IkSolver::update_parameter(std::vector<Eigen::MatrixXd> jacobs, Eigen::VectorXd q, Eigen::VectorXd v)
    {
        assert((jacobs.size() == 1 || jacobs.size() == 2) && "robot must be single/dual arm");
        q_list_.resize(jacobs.size());
        v_list_.resize(jacobs.size());
        jacob_list_.resize(jacobs.size());

        jacob_list_[0] = jacobs[0];
        if (jacobs.size() == 2)
        {
            jacob_list_[1] = jacobs[1];

            q_list_[0] = q.head(nq_/2);
            q_list_[1] = q.tail(nq_/2);
            v_list_[0] = v.head(nv_/2);
            v_list_[1] = v.tail(nv_/2);
        }
        else{
            q_list_[0] = q;
            v_list_[0] = v;
        }
    }

    std::vector<Eigen::VectorXd> IkSolver::compute_ikvel(std::vector<pinocchio::SE3> target, std::vector<pinocchio::SE3> oMi)
    {
        assert((target.size() == 1 || target.size() == 2) && "robot must be single/dual arm");
        assert((oMi.size() == 1 || oMi.size() == 2) && "robot must be single/dual arm");

        SE3 target_l = target[0];
        SE3 oMi_l = oMi[0];
        SE3 dMi_l = oMi_l.inverse() * target_l;
        Eigen::VectorXd dMi_6d_l = pinocchio::log6(dMi_l).toVector(); // Local
        Eigen::VectorXd des_vel_l = Kp_.cwiseProduct(dMi_6d_l);
        Eigen::MatrixXd Jinv_l = rci_utils::pinv_svd(jacob_list_[0]);
        Eigen::VectorXd qdot_des_l = Jinv_l * des_vel_l;
        Eigen::VectorXd qdes_l = q_list_[0] + qdot_des_l * dt_;

        std::vector<Eigen::VectorXd> qdes_list;
        qdes_list.push_back(qdes_l);
        if (jacob_list_.size()==2)
        {
            SE3 target_r = target[1];
            SE3 oMi_r = oMi[1];
            SE3 dMi_r = oMi_r.inverse() * target_r;
            Eigen::VectorXd dMi_6d_r = pinocchio::log6(dMi_r).toVector(); // Local
            Eigen::VectorXd des_vel_r = Kp_.cwiseProduct(dMi_6d_r);
            Eigen::MatrixXd Jinv_r = rci_utils::pinv(jacob_list_[1]);
            Eigen::VectorXd qdot_des_r = Jinv_r * des_vel_r;
            Eigen::VectorXd qdes_r = q_list_[1] + qdot_des_r * dt_;
            qdes_list.push_back(qdes_r);
        }

        return qdes_list;
    }

}
