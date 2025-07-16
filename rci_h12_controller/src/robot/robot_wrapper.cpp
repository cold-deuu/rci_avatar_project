#include "rci_h12_controller/robot/robot_wrapper.hpp"

using namespace pinocchio;

namespace rci_robot{
    H12Wrapper::H12Wrapper(const std::string & filename, const std::vector<std::string> & pkg_dir, std::vector<std::string> joint_to_lock_list)
    {
        pinocchio::urdf::buildModel(filename, model_);
        std::vector<Model::JointIndex> jointsToLock;
        Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(model_.nq);

        for (const auto& joint_name : joint_to_lock_list)
        {
            if (model_.existJointName(joint_name))
            {
                jointsToLock.push_back(model_.getJointId(joint_name));
            }
            else
            {
                std::cerr << "Joint not found in model: " << joint_name << std::endl;
            }
        }

        reduced_model_ = pinocchio::buildReducedModel(model_, jointsToLock, q_ref);
        reduced_data_ = pinocchio::Data(reduced_model_);

        // h12 head pose
        Eigen::Vector3d p_head = {0.04874, 0.0, 0.67980};
        Eigen::Quaterniond quat_head = {0,0,0,1};
        head_se3_ = pinocchio::SE3(quat_head, p_head);

    }

    int H12Wrapper::nq(bool is_reduced) const 
    {
        if (is_reduced)
        {
            return reduced_model_.nq;
        }
        else
            return model_.nq;
    }
    int H12Wrapper::nv(bool is_reduced) const
    {
        if (is_reduced)
        {
            return reduced_model_.nv;
        }
        else
            return model_.nv;
    }
    void H12Wrapper::compute_all_terms(pinocchio::Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v, bool is_reduced)
    {
        if (is_reduced)
        {
            pinocchio::computeAllTerms(reduced_model_, data, q, v);
            pinocchio::updateFramePlacements(reduced_model_, data);
            pinocchio::centerOfMass(reduced_model_, data, q, v, Eigen::VectorXd::Zero(reduced_model_.nv));
            pinocchio::ccrba(reduced_model_, data, q, v);
        }
        else
        {
            pinocchio::computeAllTerms(model_, data, q, v);
            pinocchio::updateFramePlacements(model_, data);
            pinocchio::centerOfMass(model_, data, q, v, Eigen::VectorXd::Zero(reduced_model_.nv));
            pinocchio::ccrba(model_, data, q, v);
        }   
    }


    const pinocchio::SE3 & H12Wrapper::compute_position(const Data & data, const Model::JointIndex index) const
    {
        assert(index<data.oMi.size());
        return data.oMi[index];
    }

    void H12Wrapper::compute_jacobian_local(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J)
    {
        Data::Matrix6x J_tmp(6, J.cols());
        return pinocchio::getJointJacobian(reduced_model_, data, index, pinocchio::LOCAL, J);
    }

    void H12Wrapper::compute_jacobian_world(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J)
    {
        Data::Matrix6x J_tmp(6, J.cols());
        return pinocchio::getJointJacobian(reduced_model_, data, index, pinocchio::WORLD, J);
    }

    void H12Wrapper::compute_jacobian_aligned(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J)
    {
        Data::Matrix6x J_tmp(6, J.cols());
        return pinocchio::getJointJacobian(reduced_model_, data, index, pinocchio::LOCAL_WORLD_ALIGNED, J);
    }

    std::vector<Eigen::MatrixXd> H12Wrapper::compute_dual_jacobian_local(const pinocchio::Data data, const pinocchio::Model model)
    {
        assert((eef_list_.size()==2) && "must regist dual arm eef");
        
        Eigen::MatrixXd Jl(6, model.nq);
        Eigen::MatrixXd Jr(6, model.nq);
        this->compute_jacobian_local(data, model.getJointId(eef_list_[0]), Jl);
        this->compute_jacobian_local(data, model.getJointId(eef_list_[1]), Jr);
        Jl = Eigen::MatrixXd(Jl.topLeftCorner(6, 7));
        Jr = Eigen::MatrixXd(Jr.topRightCorner(6, 7));
        

        std::vector<Eigen::MatrixXd> jacob_list = {Jl, Jr};

        return jacob_list;
    }

    std::vector<pinocchio::SE3> H12Wrapper::compute_dual_oMi(const pinocchio::Data data, const pinocchio::Model model, bool head_rel)
    {
        assert((eef_list_.size()==2) && "must regist dual arm eef");
        pinocchio::SE3 oMi_l = this->compute_position(data, model.getJointId(eef_list_[0]));
        pinocchio::SE3 oMi_r = this->compute_position(data, model.getJointId(eef_list_[1]));
            
        if(head_rel)
        {
            oMi_l = head_se3_.inverse() * oMi_l;
            oMi_r = head_se3_.inverse() * oMi_r;
        }
        std::vector<pinocchio::SE3> oMi_list = {oMi_l, oMi_r};

        return oMi_list;
    }

}