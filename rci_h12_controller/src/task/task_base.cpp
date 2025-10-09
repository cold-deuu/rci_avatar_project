#include <rci_h12_controller/task/task_base.hpp>

using namespace chanhqp::task;

TaskBase::TaskBase(const std::string & name):
    name_(name)
{

}

void TaskBase::Kp(double Kp)
{
    Kp_ = Kp;
}

void TaskBase::Kd(double Kd)
{
    Kd_ = Kd;
}

const std::string & TaskBase::name() const
{
  return name_;
}
