#ifndef LINEAR_MOTION_CONTROLLER_HPP
#define LINEAR_MOTION_CONTROLLER_HPP
#include "motion_control_system/motion_control_interface.hpp"

namespace motion_control_system {
    class LinearMotionController : public MotionController
    {
    private:
        /* data */
    public:
        void start() override;
        void stop() override;
    };
    
}

#endif // LINEAR_MOTION_CONTROLLER_HPP