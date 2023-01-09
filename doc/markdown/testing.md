<!-- License

Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: a.whit (nml@whit.contact)
-->

## Testing

Tests are provided, in order to verify the functionality of the package, 
as recommended in the [ROS2 Developer Guide].

At present, the only successful test is the [doctest] of the 
[example](doc/markdown/simple_example.md), which can be invoked directly via 
[pytest]. For this test to succeed, the `ros_transitions` package must be 
available via the [Python path].

```bash
python -m pytest -k 'example'
```

If this package is installed as part of a [ROS2 workspace], then colcon can be 
used to run tests, in a [configured ROS2 environment]. 
At present, the standard ROS2 [PEP257], [Flake8], and [ament_copyright] tests 
fail. To exclude these tests, invoke the `colcon` [test verb] with 
[pytest keyword expression] arguments.

```bash
cd path/to/workspace
source path/to/ros/setup.bash
source install/local_setup.bash
colcon test --pytest-args -k 'not flake8 and not pep257 and not copyright'
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

