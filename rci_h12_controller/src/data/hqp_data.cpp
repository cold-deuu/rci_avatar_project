#include "rci_h12_controller/data/hqp_data.hpp"

using namespace chanhqp::data;

HQPData::HQPData(){

}
void HQPData::qp_initialize()
{
    // Initialize QP Problem
    Q.resize(0,0);
    C.resize(0);
    A_eq.resize(0,0);
    b_eq.resize(0);
    A_ineq.resize(0,0);
    b_ineq.resize(0);
    slack_eq_.push_back(Eigen::VectorXd(0));
    slack_ineq_.push_back(Eigen::VectorXd(0));
}

void HQPData::initialize_all_data()
{
    tasks_.clear();
    safety_tasks_.clear();
    constraint_.clear();
    safety_constraint_.clear();
}

void HQPData::addTask(std::shared_ptr<TaskBase> task, int priority)
{
    if (priority == 0) safety_tasks_.push_back(task);
    else
    {
        auto [it, inserted] = tasks_.emplace(priority, task);
        assert(inserted && "Duplicate priority!");   
    }
    
    std::cout<<"[HQP] Task Name : "<<task->name()<<" with Priority : "<<priority<<" Added!"<<std::endl;

}

void HQPData::compute_problem_data(State state)
{
    constraint_.clear();
    safety_constraint_.clear();
    // 범위 기반 for
    for (const auto& task_ptr : safety_tasks_) {
        safety_constraint_.push_back(task_ptr->compute(state));
    }


    for (const auto& [priority, task_ptr] : tasks_) {
        // Safety Constrant : Joint Limit, Self Collisio, etc
        std::shared_ptr<chanhqp::constraint::ConstraintBase> c = task_ptr->compute(state);

        if(priority == 0)
        {
            assert(false && "Priority 0 is not allowed!");
        }
        else
        {
            auto [it, inserted] = constraint_.emplace(priority, c);
            assert(inserted && "Duplicate priority!");

        }
    }

    // assert(!constraint_.empty());
    auto max_priority = constraint_.rbegin()->first;
    num_task_ = static_cast<int>(constraint_.size());

    // std::cout<<"Max :"<<max_priority<<"Num Task :"<<num_task_<<std::endl;
    assert(max_priority == num_task_);

}

Eigen::VectorXd HQPData::solve()
{
    this->qp_initialize();   
    Eigen::VectorXi activeSet(0);
    size_t activeSetSize;


    bool check_eq_init = false;
    bool check_ineq_init = false;

    for(int i=0; i<safety_constraint_.size(); i++)
    {
        std::shared_ptr<chanhqp::constraint::ConstraintBase> cs = safety_constraint_[i];
        if (cs->isEquality())
        {
            int n_row_old = A_eq.rows();
            int n_col_old = A_eq.cols();
            int n_row_new = cs->getMatrix().rows();
            int n_col_new = cs->getMatrix().cols();
            
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            mat.topLeftCorner(n_row_old, n_col_old) = A_eq;
            mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
            vec.head(n_row_old) = b_eq;
            vec.tail(n_row_new) = cs->getVector();

            A_eq = mat;
            b_eq = vec;

            check_eq_init = true;

        }
        else if(cs->isInequality())
        {
            int n_row_old = A_ineq.rows();
            int n_col_old = A_ineq.cols();
            int n_row_new = cs->getMatrix().rows();
            int n_col_new = cs->getMatrix().cols();  
            
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            mat.topLeftCorner(n_row_old, n_col_old) = A_ineq;
            mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
            vec.head(n_row_old) = b_ineq;
            vec.tail(n_row_new) = cs->getVector();

            A_ineq = mat;
            b_ineq = vec;

            check_ineq_init = true;
        }
    }

    for(int i=1; i<num_task_+1; i++)
    {
        std::shared_ptr<chanhqp::constraint::ConstraintBase> c = constraint_.at(i);
        int n_row_new = c->getMatrix().rows();
        int n_col_new = c->getMatrix().cols();
        int n_slack = n_row_new;

        // Check Initialize for QP
        if(i==1)
        {
            if(!check_eq_init)
            {
                A_eq.resize(0, n_col_new);
                check_eq_init = true;
            }
            if(!check_ineq_init)
            {
                A_ineq.resize(0, n_col_new);
                check_ineq_init = true;
            }
        }

        if (c->isEquality())
        {
            int n_row_old = A_eq.rows();
            int n_col_old = A_eq.cols() - slack_eq_[i-1].size();

            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new + n_slack);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
            Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
            C.setZero(n_col_new + n_slack);

            mat.topLeftCorner(n_row_old, n_col_new) = A_eq.topLeftCorner(n_row_old, n_col_new);
            mat.bottomLeftCorner(n_row_new, n_col_new) = c->getMatrix();
            mat.bottomRightCorner(n_row_new, n_slack).setIdentity();
            vec.head(n_row_old) = b_eq;
            vec.tail(n_row_new) = c->getVector();

            A_eq = mat;
            b_eq = vec;

            A_ineq.conservativeResize(A_ineq.rows(), n_col_new + n_slack);
            A_ineq.rightCols(n_slack).setZero();
            Eigen::VectorXd x(n_col_new + n_slack);

            double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);
            sol_ = x.head(n_col_new);
            slack_eq_.push_back(x.tail(n_slack));
            slack_ineq_.push_back(Eigen::VectorXd(0));
            A_ineq = A_ineq.leftCols(n_col_new);


        }
        else if(c->isInequality())
        {
            int n_row_old = A_ineq.rows();
            int n_col_old = A_ineq.cols() - slack_ineq_[i-1].size();

            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new + n_slack);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
            Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
            C.setZero(n_col_new + n_slack);

            mat.topLeftCorner(n_row_old, n_col_new) = A_ineq.topLeftCorner(n_row_old, n_col_new);
            mat.bottomLeftCorner(n_row_new, n_col_new) = c->getMatrix();
            vec.head(n_row_old) = b_ineq;
            vec.tail(n_row_new) = c->getVector();

            A_ineq = mat;
            b_ineq = vec;

            A_eq.conservativeResize(A_eq.rows(), n_col_new + n_slack);
            A_eq.rightCols(n_slack).setZero();

            Eigen::VectorXd x(n_col_new + n_slack);
            double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);

            sol_ = x.head(n_col_new);
            slack_ineq_.push_back(x.tail(n_slack));
            slack_eq_.push_back(Eigen::VectorXd(0));

            A_eq = A_eq.leftCols(n_col_new);
        }

        // i start from 1
        b_eq.tail(slack_eq_[i-1].size()) -= slack_eq_[i-1];
        b_ineq.tail(slack_ineq_[i-1].size()) -= slack_ineq_[i-1];
    }
    return sol_;
}