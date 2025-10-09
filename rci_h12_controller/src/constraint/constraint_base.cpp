#include <rci_h12_controller/constraint/constraint_base.hpp>

using namespace chanhqp::constraint;

ConstraintBase::ConstraintBase(const std::string & name):
name_(name){}

const std::string & ConstraintBase::name() const
{
  return name_;
}
