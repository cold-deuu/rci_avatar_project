#pragma once

#include "rci_h12_controller/utils/utils.hpp"

namespace rci_utils
{
    inline Eigen::MatrixXd pinv(Eigen::MatrixXd mat)
    {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(mat);
        return cod.pseudoInverse();
    }

    inline Eigen::MatrixXd pinv_svd(const Eigen::MatrixXd &mat, double epsilon = 1e-6)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const auto &singular_values = svd.singularValues();
        Eigen::VectorXd inv_singular_values = singular_values;

        for (int i = 0; i < singular_values.size(); ++i)
        {
            if (singular_values(i) > epsilon)
                inv_singular_values(i) = 1.0 / singular_values(i);
            else
                inv_singular_values(i) = 0.0; // 제거 (널스페이스 방지)
        }

        return svd.matrixV() * inv_singular_values.asDiagonal() * svd.matrixU().transpose();
    }

    inline pinocchio::SE3 pose_msgs_to_se3(const geometry_msgs::msg::Pose pose)
    {
        Eigen::Quaterniond quat;
        quat.x() = pose.orientation.x;
        quat.y() = pose.orientation.y;
        quat.z() = pose.orientation.w;
        quat.w() = pose.orientation.z;

        Eigen::Vector3d p;
        p<< pose.position.x, pose.position.y, pose.position.z;
        
        return pinocchio::SE3(quat, p);
    }

    inline Eigen::Matrix3d AngleAngle_to_Rot(Eigen::Vector3d axis, double angle) {
        Eigen::Matrix3d Rot;
        double kx, ky, kz, theta, vt;
        kx = axis(0);
        ky = axis(1);
        kz = axis(2);
        theta = angle;
        vt = 1.0 - cos(theta);


        Rot(0, 0) = kx * kx*vt + cos(theta);
        Rot(0, 1) = kx * ky*vt - kz * sin(theta);
        Rot(0, 2) = kx * kz*vt + ky * sin(theta);
        Rot(1, 0) = kx * ky*vt + kz * sin(theta);
        Rot(1, 1) = ky * ky*vt + cos(theta);
        Rot(1, 2) = ky * kz*vt - kx * sin(theta);
        Rot(2, 0) = kx * kz*vt - ky * sin(theta);
        Rot(2, 1) = ky * kz*vt + kx * sin(theta);
        Rot(2, 2) = kz * kz*vt + cos(theta);

        return Rot;
    }

    inline Eigen::Vector3d GetPhi(Eigen::Matrix3d Rot, Eigen::Matrix3d Rotd)
    {
        Eigen::Vector3d phi;
        Eigen::Vector3d s[3], v[3], w[3];
        for (int i = 0; i < 3; i++) {
        v[i] = Rot.block(0, i, 3, 1);
        w[i] = Rotd.block(0, i, 3, 1);
        s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5* phi;
        return phi;
    }
}