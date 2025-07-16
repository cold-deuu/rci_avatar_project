#pragma once

// pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

// standard cpp
#include <string>
#include <vector>

using namespace pinocchio;

namespace rci_robot{
    class H12Wrapper
    {
        public:
            H12Wrapper(const std::string & filename, const std::vector<std::string> & pkg_dir, std::vector<std::string> joint_to_lock_list);
            ~H12Wrapper(){};

            int nq_, nv_;

            pinocchio::Model model_, reduced_model_;
            pinocchio::Data data_, reduced_data_;
            pinocchio::SE3 head_se3_;

            std::vector<std::string> eef_list_;

            int nq(bool is_reduced) const;
            int nv(bool is_reduced) const;

            // Compute-All-Terms
            void compute_all_terms(pinocchio::Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v, bool is_reduced);

            // Jacobian
            void compute_jacobian_local(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J);
            void compute_jacobian_world(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J);
            void compute_jacobian_aligned(const pinocchio::Data data, const Model::JointIndex index, Eigen::MatrixXd & J);

            // Poisition
            const pinocchio::SE3 & compute_position(const pinocchio::Data & data, const Model::JointIndex index) const;

            // dual copmute
            std::vector<Eigen::MatrixXd> compute_dual_jacobian_local(const pinocchio::Data data, const pinocchio::Model model);
            std::vector<pinocchio::SE3> compute_dual_oMi(const pinocchio::Data data, const pinocchio::Model model, bool head_rel = false);


            // Return Data and Model
            pinocchio::Data data(bool is_reduced)
            {
                if (!is_reduced)
                    return data_;
                else return reduced_data_;
            };
            pinocchio::Model model(bool is_reduced)
            {
                if (!is_reduced)
                    return model_;
                else return reduced_model_;
            };

            void regist_dual_eef(std::vector<std::string> dual_eef)
            {
                eef_list_ = dual_eef;
            }



    };
}
