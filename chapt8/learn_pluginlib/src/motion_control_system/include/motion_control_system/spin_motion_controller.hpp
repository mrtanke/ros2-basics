#ifndef SPIN_MOTION_CONTROLLER_HPP
#define SPIN_MOTION_CONTROLLER_HPP
#include "motion_control_system/motion_control_interface.hpp"

namespace motion_control_system {
    class SpinMotionController : public MotionController
    {
    private:
        /* data */
    public:
        void start() override;
        void stop() override;
    };
    
}

#endif // SPIN_MOTION_CONTROLLER_HPP