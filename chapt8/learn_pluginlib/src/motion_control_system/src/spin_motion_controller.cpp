#include "iostream"
#include "motion_control_system/spin_motion_controller.hpp"

namespace motion_control_system
{
    void SpinMotionController::start()
    {
        // Implement rotation motion control logic
        std::cout << "SpinMotionController::start" << std::endl;
    }
    void SpinMotionController::stop()
    {
        // Stop motion control
        std::cout << "SpinMotionController::stop" << std::endl;
    }
} // namespace motion_control_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)