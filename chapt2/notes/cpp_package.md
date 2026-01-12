
## C++ function package structure analysis (ROS 2 / ament_cmake)

This note describes the common folder/file layout for a ROS 2 C++ package (built with `ament_cmake`) and what each part is used for.

---

### Package root (one folder per package)

This folder contains the package manifest and build configuration.

- `package.xml`
	- ROS 2 manifest: package name/version, dependencies, maintainers.
	- Build/run dependencies for C++ typically include `ament_cmake`, `rclcpp`.
- `CMakeLists.txt`
	- Build instructions for libraries/nodes.
	- Defines executables, libraries, include dirs, dependencies, install rules.
- `include/<pkg_name>/`
	- Public headers (things other packages may `#include`).
	- Put reusable functions/classes here when they form a library API.
- `src/`
	- C++ implementation files (`.cpp`).
	- Common patterns:
		- `src/<node>.cpp` for node executables
		- `src/<lib>.cpp` for library implementation
- `launch/`
	- Launch files (`.py`) for running nodes with parameters/remaps.
- `config/`
	- YAML parameter files and other runtime configuration.
- `resource/`
	- ament resource index files (sometimes present depending on tooling).
- `msg/`, `srv/`, `action/` (optional)
	- Interface definitions. If present, the package also runs code generation.
- `test/` (optional)
	- Unit/integration tests (often `gtest` / `ament_cmake_gtest`).

---

### Generated workspace folders (colcon)

At the workspace level after `colcon build`:

- `build/`  Intermediate build artifacts (do not edit)
- `install/` Installed outputs (what `ros2 run` uses after sourcing setup)
- `log/` Build/test logs

---

### How C++ “functions” become reusable code vs. runnable nodes

In C++ ROS 2, you usually do both:

1) **Reusable functions/classes** as a library
- Headers in `include/<pkg_name>/...`
- Implementations in `src/...`
- Built with `add_library(...)` and linked into nodes with `target_link_libraries(...)`

2) **Runnable nodes** as executables
- A node source file (e.g., `src/my_node.cpp`) contains `main()` and spins an `rclcpp::Node`.
- Built with `add_executable(...)`.
- Installed so `ros2 run <pkg> <executable>` works.

---

### Minimal CMake pattern (typical)

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS my_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

### Build + run instructions (workspace)

From the workspace root (the folder that contains `src/<your_pkg>` or your packages):

- Build:
	- `colcon build --symlink-install`
- Source the overlay (new terminal or same shell):
	- `source install/setup.bash`
- Run a node:
	- `ros2 run <pkg_name> <executable_name>`

Useful debug commands:

- List packages: `ros2 pkg list | grep <pkg_name>`
- List executables in a package: `ros2 pkg executables <pkg_name>`

