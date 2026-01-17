cmake_minimum_required(VERSION 3.8)
project(nav2_custom_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# include header files directory
include_directories(include)
# define library name
set(library_name ${PROJECT_NAME}_plugin)
# create shared library
add_library(${library_name} SHARED  src/nav2_custom_planner.cpp)
# specify library dependencies
ament_target_dependencies(${library_name} nav2_core pluginlib)
# install library to specified directory
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install header files to specified directory
install(DIRECTORY include/
  DESTINATION include/ )
# export plugin description file
pluginlib_export_plugin_description_file(nav2_core custom_planner_plugin.xml)

ament_package()
