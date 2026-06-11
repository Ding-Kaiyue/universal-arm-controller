#include "chassis_controller/model/chassis_joint_command.hpp"

namespace chassis_controller {

bool ChassisJointCommandSet::empty() const {
    return joints.empty();
}

}  // namespace chassis_controller
