#include "motion_control_system/motion_control_interface.hpp"
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv) {
  // judge command line args
  if (argc != 2)
    return 0;

  // Select the loader through command line argument, argv[0] is the executable name, argv[1] is the parameter name
  std::string controller_name = argv[1];

  // 1. Create a controller loader through the package name and base class name
  pluginlib::ClassLoader<motion_control_system::MotionController>
      controller_loader("motion_control_system",
                        "motion_control_system::MotionController");

  // 2. Use the loader to load the specified plugin by name, returning a pointer to the plugin class object
  auto controller = controller_loader.createSharedInstance(controller_name);

  // 3. Call the plugin's methods
  controller->start();
  controller->stop();
  return 0;
}