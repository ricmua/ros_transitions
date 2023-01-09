<!-- License

Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: a.whit (nml@whit.contact)
-->


## Testing

A number of unit, integration, and/or regression tests are included in this 
repository, as recommended in the [ROS2 Developer Guide]. All tests should be 
invoked from within a [configured ROS2 environment]. The ROS2 package must be 
[installed](doc/markdown/installation.md) before attempting to run 
tests. There are three potential mechanisms for initiating tests.

### colcon

Testing can be initiated via [colcon], after it has been used to build the ROS2 
workspace (see the [installation](doc/markdown/installation.md) documentation).

```bash
cd path/to/workspace
source path/to/ros/setup.bash
source install/local_setup.bash
colcon test
```

At present, the standard ROS2 [PEP257], [Flake8], and [ament_copyright] tests 
fail. To exclude these tests, invoke the `colcon` [test verb] with 
[pytest keyword expression] arguments.

```bash
colcon test --pytest-args -k 'not flake8 and not pep257 and not copyright'
```

The `event-handlers` `colcon` flag can be used to obtain more verbose feedback.

```bash
colcon test --event-handlers console_cohesion+
```

See the [ROS2 Python testing] documentation for further information.

### doctest

Tests written into the documentation can be invoked via the [doctest] package 
and testing framework. This is a **recommended first step**, to ensure basic 
functionality. Doctests are included in many of the Python source code files, 
as well as the package README and some of the Markdown documentation files.

```bash
python -m doctest path/to/ros_transitions/README.md
```

As mentioned, this must be done from within a [configured ROS2 environment]. 
The output of this command should indicate that all tests passed. 

If desired, the doctests can also be run directly from a Python environment.

```python
import doctest
doctest.testfile('path/to/README.md', module_relative=False)

```

### pytest

The [pytest] framework can also be invoked directly, independently of `colcon`. 
This must be done from within a [configured ROS2 environment].

```bash
python3 -m pytest path/to/ros_transitions/test
```

The `-k` and `-vv` command line flags for `pytest` might be useful. The former 
can be used to run specific tests (e.g., `-k test_object_initialization`). The 
latter requests verbose output. See the documentation for [pytest invocations] 
for further information.

```bash
python3 -m pytest path/to/ros_transitions/test/ -k test_state_sequence -vv
```


<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Python path]: https://docs.python.org/3/tutorial/modules.html#the-module-search-path

[doctest]: https://docs.python.org/3/library/doctest.html

[pytest]: https://docs.pytest.org/

[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[ROS2 Developer Guide]: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#testing

[ROS2 testing]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html#

[ros2_basic_python_tests]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

[ROS2 Python testing]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

[PEP257]: https://peps.python.org/pep-0257/

[Flake8]: https://flake8.pycqa.org/en/latest/

[test verb]: https://colcon.readthedocs.io/en/released/reference/verb/test.html

[pytest keyword expression]: https://docs.pytest.org/en/7.2.x/how-to/usage.html#specifying-which-tests-to-run

[ament_copyright]: https://index.ros.org/p/ament_copyright/

