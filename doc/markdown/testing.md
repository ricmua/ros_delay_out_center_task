<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Testing

A number of unit, integration, and/or regression tests are included in this 
package, as recommended in the [ROS2 Developer Guide]. All tests should be 
invoked from within a [configured ROS2 environment]. The ROS2 package must be 
[installed](doc/markdown/installation.md) and built before attempting to run 
tests. Testing can be initiated via `colcon`.[^failure]

[^failure]: At present, the standard ROS2 [PEP257] and [Flake8] tests fail.

```bash
cd path/to/workspace
colcon test
```

See the [ROS2 Python testing] documentation for further information.

Should tests fail, the `event-handler` `colcon` flag can be used to obtain more 
verbose feedback.

```bash
colcon test --event-handler console_cohesion+
```

### Testing the `delay_out_center_task` package

The [delay_out_center_task] includes a conventional [pytest] framework.  These tests
can be invoked directly, if desired.

```bash
python -m pytest -s src/ros_delay_out_center_task/delay_out_center_task
```

See the [documentation][delay_out_center_task] for futher information.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Python path]: https://docs.python.org/3/tutorial/modules.html#the-module-search-path

[doctest]: https://docs.python.org/3/library/doctest.html

[ROS2]: https://docs.ros.org/en/humble/index.html


[pytest]: https://docs.pytest.org/


[configured ROS2 environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

[ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

[configure_ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html


[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html


[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task


[ros2_developer_guide-testing]: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#testing

[ROS2 testing]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html#

[ros2_basic_python_tests]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

[ROS2 Python testing]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

[ROS2 Developer Guide]: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#testing

[PEP257]: https://peps.python.org/pep-0257/

[Flake8]: https://flake8.pycqa.org/en/latest/


