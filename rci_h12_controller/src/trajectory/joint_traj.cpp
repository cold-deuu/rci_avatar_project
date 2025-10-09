#include "rci_h12_controller/trajectory/joint_traj.hpp"

using namespace Eigen;
using namespace pinocchio;

namespace rci_trajectory{
    JointCubicTrajectory::JointCubicTrajectory(const std::string & filename, int armjoint_num){
        m_na = armjoint_num;
        m_init.setZero(m_na);
        m_goal.setZero(m_na);

    }

    void JointCubicTrajectory::SetCurrentTime(rclcpp::Time ctime){
        m_ctime = ctime;
    }
    
    void JointCubicTrajectory::SetStartTime(rclcpp::Time stime){
        m_stime = stime;
    }

    void JointCubicTrajectory::SetDuration(rclcpp::Duration duration){
        m_duration = duration;
    }
    
    void JointCubicTrajectory::SetInitSample(Eigen::VectorXd init_sample){
        m_init = init_sample;
    }

    void JointCubicTrajectory::SetGoalSample(Eigen::VectorXd goal_sample){
        m_goal = goal_sample;
    }

    
    Eigen::VectorXd JointCubicTrajectory::computeNext(){
        //Property
        Eigen::VectorXd q_cubic = m_init;
        
        double a0,a1,a2,a3;
        double stime = static_cast<double>(m_stime.nanoseconds()) * 1e-9;
        double time = static_cast<double>(m_ctime.nanoseconds()) * 1e-9;
        double duration = m_duration.seconds();

        if(time<stime){
            return m_init;
        }
        else if(time> stime + duration){
            return m_goal;
        }
        else{
            for(int i=0;i<q_cubic.size();i++){
                a0 = m_init(i);
                a1 = 0;
                a2 = 3/pow(duration,2) *(m_goal(i)-m_init(i));
                a3 = -1.0 *2.0/pow(duration,3) *(m_goal(i)-m_init(i));
                q_cubic(i) = a0 + a1 * (time - stime) + a2*pow(time-stime,2) + a3 * pow(time-stime,3);
            }
            return q_cubic;
        }
    }
}