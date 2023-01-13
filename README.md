---
title: README
author: a.whit ([email](mailto:nml@whit.contact))
date: November 2022
---

<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors: 
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

A [ROS2 package] for interfacing with the "delay-out-center" behavioral task, 
as implemented in the [delay_out_center_task] Python package. This package 
links the behavioral task to the [ROS2 graph], in order to facilitate 
interaction with other ROS2 packages. For example, this might be useful for 
connecting the task to a GUI managed by a remote [ROS2 node] -- such as the one 
provided by the [ros_tkinter_spheres_environment_gui] package.

This package primarily consists of code and infrastructure for integrating 
several other distinct Python and ROS2 packages. See the 
[integration documentation](doc/markdown/integration.md) for further 
information about these component packages.

## Installation

This package can be added to any [ROS2 workspace]. ROS2 workspaces are built 
using [colcon]. See the 
[installation documentation](doc/markdown/installation.md) for further 
information.

### Testing

See the [testing documentation](doc/markdown/testing.md) for further 
information. Also see notes about [integration](doc/markdown/integration.md) 
testing.

## License

Copyright 2022-2023 [Neuromechatronics Lab], Carnegie Mellon University

Contributors: 
* a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[Python path]: https://docs.python.org/3/tutorial/modules.html#the-module-search-path

[doctest]: https://docs.python.org/3/library/doctest.html

[pytransitions]: https://github.com/pytransitions/transitions

[ros_transitions]: https://github.com/ricmua/ros_transitions

[ROS2]: https://docs.ros.org/en/humble/index.html

[setuptools]: https://setuptools.pypa.io/en/latest/userguide/quickstart.html#basic-use

[Neuromechatronics Lab]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[pip install]: https://pip.pypa.io/en/stable/cli/pip_install/

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[colcon]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task
[ros_spheres_environment]: https://github.com/ricmua/ros_spheres_environment
[ros_transitions]: https://github.com/ricmua/ros_transitions
[ros_parameter_collections]: https://github.com/ricmua/ros_parameter_collections
[ros_threading_timer]: https://github.com/ricmua/ros_threading_timer

[ROS2 package]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-is-a-ros-2-package

[ros_tkinter_spheres_environment_gui]: https://github.com/ricmua/ros_tkinter_spheres_environment_gui

[ROS2 graph]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background

[ROS2 node]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background

