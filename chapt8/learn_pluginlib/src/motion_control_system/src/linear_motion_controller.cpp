#include "iostream"
#include "motion_control_system/linear_motion_controller.hpp"

namespace motion_control_system
{
    void LinearMotionController::start()
    {
        // Implement linear motion control logic
        std::cout << "LinearMotionController::start" << std::endl;
    }
    void LinearMotionController::stop()
    {
        // Stop motion control
        std::cout << "LinearMotionController::stop" << std::endl;
    }
} // namespace motion_control_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motion_control_system::LinearMotionController, motion_control_system::MotionController)