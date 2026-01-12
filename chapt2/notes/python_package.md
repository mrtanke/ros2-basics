## Python function package structure analysis

This workspace contains a ROS 2 Python package (an `ament_python` package). Below is a simple “what goes where” guide for the package and the most common folders you’ll see.

---

### Top-level workspace (this folder)

Typical purpose: holds one or more ROS 2 packages plus build outputs.

- `demo_python_pkg/`
	- The *source* package folder (what you edit).
- `build/`, `install/`, `log/`
	- Colcon outputs (generated). Usually you don’t edit these.
- `ros2_python_node.py`, `ros2_cpp_node.cpp`
	- Standalone examples (not necessarily part of the installable package unless wired into a package).

---

### Package folder: `demo_python_pkg/`

This is the ROS 2 package root (the one that has `package.xml`).

- `package.xml`
	- ROS 2 package manifest.
	- Declares dependencies (e.g., `rclpy`), package name/version, maintainer, etc.
- `setup.py`
	- Python packaging entry point for `setuptools`.
	- Common jobs:
		- declare which Python modules get installed
		- declare console scripts (executables) via `entry_points`
- `setup.cfg`
	- Packaging configuration (often holds `entry_points`, metadata, lint config).
	- In many ROS 2 examples, the `console_scripts` entry points live here.
- `LICENSE`
	- License text.

#### Python module source: `demo_python_pkg/demo_python_pkg/`

This inner folder is the *actual Python package/module* that gets imported.

- `__init__.py`
	- Marks this directory as a Python package.
	- Can be empty, or can re-export functions/classes for convenient imports.
- `python_node.py`
	- Your Python code.
	- In ROS 2 terms, this typically defines:
		- node class(es)
		- helper functions
		- a `main()` function that spins the node

**Rule of thumb:** if you want code to be importable as `import demo_python_pkg`, it should live under `demo_python_pkg/demo_python_pkg/`.

#### Resource index: `demo_python_pkg/resource/`

- `demo_python_pkg` (a plain file)
	- Used by `ament_index` so ROS 2 can discover the package at runtime.
	- This is how tooling knows your package exists after install.

#### Tests: `demo_python_pkg/test/`

ROS 2 Python packages often include standard quality tests:

- `test_flake8.py`
	- Style/lint checks.
- `test_pep257.py`
	- Docstring style checks.
- `test_copyright.py`
	- License header checks.

---

### Generated folders (colcon output)

You will see these after building with `colcon build`:

- `build/`
	- Intermediate build products.
	- For Python packages, you’ll often see generated wrappers, egg-info, and staged build outputs.
- `install/`
	- The installed result of your workspace.
	- Important locations:
		- `install/<pkg>/lib/<pkg>/...` for installed executables
		- `install/<pkg>/lib/pythonX.Y/site-packages/<pkg>/...` for importable Python modules
- `log/`
	- Build/test logs.

---

### How “functions” become a runnable ROS 2 command

In ROS 2 Python packages, you usually expose a runnable command using `console_scripts`.

Conceptually:

- You write a `main()` function in something like `demo_python_pkg/python_node.py`.
- You register it as an entry point, e.g.:
	- `demo_python_pkg.python_node:main`
- After `colcon build` + sourcing `install/setup.bash`, you can run:
	- `ros2 run demo_python_pkg <entry_point_name>`

---

### Quick checklist (where to put what)

- Reusable helper functions/classes: `demo_python_pkg/demo_python_pkg/` (e.g., `helpers.py`)
- ROS 2 node executable code: `demo_python_pkg/demo_python_pkg/python_node.py` (or similar)
- Runtime discoverability file: `demo_python_pkg/resource/demo_python_pkg`
- Dependencies/metadata: `demo_python_pkg/package.xml`, `demo_python_pkg/setup.py`, `demo_python_pkg/setup.cfg`
- Tests: `demo_python_pkg/test/`

